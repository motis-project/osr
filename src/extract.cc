#include "osr/extract/extract.h"

#include "boost/thread/tss.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "osmium/area/assembler.hpp"
#include "osmium/area/multipolygon_manager.hpp"
#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/index/map/flex_mem.hpp"
#include "osmium/io/pbf_input.hpp"
#include "osmium/io/xml_input.hpp"

#include "utl/helpers/algorithm.h"
#include "utl/parser/arg_parser.h"
#include "utl/progress_tracker.h"
#include "utl/to_vec.h"

#include "tiles/osm/hybrid_node_idx.h"
#include "tiles/osm/tmp_file.h"
#include "tiles/util_parallel.h"

#include "osr/extract/tags.h"
#include "osr/ways.h"
#include "utl/enumerate.h"

namespace osm = osmium;
namespace osm_io = osmium::io;
namespace osm_rel = osmium::relations;
namespace osm_eb = osmium::osm_entity_bits;
namespace osm_area = osmium::area;
namespace osm_mem = osmium::memory;
namespace fs = std::filesystem;
using namespace std::string_view_literals;

namespace osr {

struct osm_restriction {
  bool valid() const {
    return from_ != osm_way_idx_t::invalid() &&
           to_ != osm_way_idx_t::invalid() && via_ != osm_node_idx_t::invalid();
  }

  osm_way_idx_t from_{osm_way_idx_t::invalid()};
  osm_way_idx_t to_{osm_way_idx_t::invalid()};
  osm_node_idx_t via_{osm_node_idx_t::invalid()};
};

bool is_number(std::string_view s) {
  return utl::all_of(s, [](char const c) { return std::isdigit(c); });
}

constexpr speed_limit get_speed_limit(unsigned const x) {
  if (x >= 120U) {
    return speed_limit::kmh_120;
  } else if (x >= 100) {
    return speed_limit::kmh_100;
  } else if (x >= 70) {
    return speed_limit::kmh_70;
  } else if (x >= 50) {
    return speed_limit::kmh_50;
  } else if (x >= 30) {
    return speed_limit::kmh_30;
  } else {
    return speed_limit::kmh_10;
  }
}

speed_limit get_speed_limit(tags const& t) {
  if (!t.max_speed_.empty() &&
      is_number(t.max_speed_) /* TODO: support units (kmh/mph) */) {
    return get_speed_limit(utl::parse<unsigned>(t.max_speed_));
  } else {
    switch (cista::hash(t.highway_)) {
      case cista::hash("motorway"): return get_speed_limit(90);
      case cista::hash("motorway_link"): return get_speed_limit(45);
      case cista::hash("trunk"): return get_speed_limit(85);
      case cista::hash("trunk_link"): return get_speed_limit(40);
      case cista::hash("primary"): return get_speed_limit(65);
      case cista::hash("primary_link"): return get_speed_limit(30);
      case cista::hash("secondary"): return get_speed_limit(55);
      case cista::hash("secondary_link"): return get_speed_limit(25);
      case cista::hash("tertiary"): return get_speed_limit(40);
      case cista::hash("tertiary_link"): return get_speed_limit(20);
      case cista::hash("unclassified"): return get_speed_limit(25);
      case cista::hash("residential"): return get_speed_limit(25);
      case cista::hash("living_street"): return get_speed_limit(10);
      case cista::hash("service"): return get_speed_limit(15);
      case cista::hash("track"): return get_speed_limit(12);
      case cista::hash("path"): return get_speed_limit(13);
      default: return speed_limit::kmh_10;
    }
  }
}

way_properties get_way_properties(tags const& t) {
  return {
      .is_foot_accessible_ = is_accessible<foot_profile>(t, osm_obj_type::kWay),
      .is_bike_accessible_ = is_accessible<bike_profile>(t, osm_obj_type::kWay),
      .is_car_accessible_ = is_accessible<car_profile>(t, osm_obj_type::kWay),
      .is_oneway_car_ = t.oneway_,
      .is_oneway_bike_ = t.oneway_ && !t.not_oneway_bike_,
      .is_elevator_ = t.is_elevator_,
      .is_steps_ = (t.highway_ == "steps"sv),
      .speed_limit_ = get_speed_limit(t),
      .level_ = to_idx(t.level_),
      .to_level_ = to_idx(t.level_to_)};
}

node_properties get_node_properties(osm::Node const& n) {
  auto const t = tags{n};
  return {
      .from_level_ = to_idx(t.level_),
      .is_foot_accessible_ =
          is_accessible<foot_profile>(t, osm_obj_type::kNode),
      .is_bike_accessible_ =
          is_accessible<bike_profile>(t, osm_obj_type::kNode),
      .is_car_accessible_ = is_accessible<car_profile>(t, osm_obj_type::kNode),
      .is_elevator_ = t.is_elevator_,
      .is_entrance_ = t.is_entrance_,
      .to_level_ = to_idx(t.level_to_)};
}

struct way_handler : public osm::handler::Handler {
  way_handler(ways& w, hash_map<osm_way_idx_t, way_properties>& rel_ways)
      : w_{w}, rel_ways_{rel_ways} {}

  void way(osm::Way const& w) {
    auto const osm_way_idx = osm_way_idx_t{w.positive_id()};
    auto const it = rel_ways_.find(osm_way_idx);
    auto const t = tags{w};
    if ((it == end(rel_ways_) && t.highway_.empty() && !t.is_platform_) ||
        (t.highway_.empty() && !t.is_platform_ && it != end(rel_ways_) &&
         t.landuse_)) {
      return;
    }

    auto const p = (t.is_platform_ || !t.highway_.empty())
                       ? get_way_properties(t)
                       : it->second;
    if (!p.is_accessible()) {
      return;
    }

    auto const get_point = [](osmium::NodeRef const& n) {
      return point::from_location(n.location());
    };

    auto const get_node_id = [&](osmium::NodeRef const& n) {
      w_.node_way_counter_.increment(n.positive_ref());
      return osm_node_idx_t{n.positive_ref()};
    };

    auto l = std::scoped_lock<std::mutex>{mutex_};
    w_.way_osm_idx_.push_back(osm_way_idx_t{w.positive_id()});
    w_.way_polylines_.emplace_back(w.nodes() |
                                   std::views::transform(get_point));
    w_.way_osm_nodes_.emplace_back(w.nodes() |
                                   std::views::transform(get_node_id));
    w_.way_properties_.emplace_back(p);
  }

  std::mutex mutex_;
  ways& w_;
  hash_map<osm_way_idx_t, way_properties>& rel_ways_;
};

struct node_handler : public osm::handler::Handler {
  struct cache {
    void clear() {
      from_.clear();
      to_.clear();
    }
    std::vector<way_idx_t> from_, to_;
  };

  node_handler(ways& w, std::vector<resolved_restriction>& r) : r_{r}, w_{w} {
    w_.node_properties_.resize(w_.n_nodes());
  }

  void node(osm::Node const& n) {
    if (auto const node_idx = w_.find_node_idx(osm_node_idx_t{n.id()});
        node_idx.has_value()) {
      w_.node_properties_[*node_idx] = get_node_properties(n);
    }
  }

  void relation(osm::Relation const& r) {
    static auto c = boost::thread_specific_ptr<cache>{};
    if (c.get() == nullptr) {
      c.reset(new cache{});
    }
    c->clear();

    auto const type = r.tags()["type"];
    if (type == nullptr || type != "restriction"sv) {
      return;
    }

    auto const restriction_ptr = r.tags()["restriction"];
    if (restriction_ptr == nullptr) {
      return;
    }

    auto const restriction_sv = std::string_view{restriction_ptr};
    auto restriction_type = resolved_restriction::type::kNo;
    if (restriction_sv.starts_with("no")) {
      restriction_type = resolved_restriction::type::kNo;
    } else if (restriction_sv.starts_with("only")) {
      restriction_type = resolved_restriction::type::kOnly;
    } else {
      return;
    }

    auto via = node_idx_t::invalid();
    for (auto const& m : r.members()) {
      switch (cista::hash(std::string_view{m.role()})) {
        case cista::hash("to"): {
          auto const to = w_.find_way(osm_way_idx_t{m.positive_ref()});
          if (to.has_value()) {
            c->to_.emplace_back(*to);
          }
          break;
        }

        case cista::hash("from"): {
          auto const from = w_.find_way(osm_way_idx_t{m.positive_ref()});
          if (from.has_value()) {
            c->from_.emplace_back(*from);
          }
          break;
        }

        case cista::hash("via"):
          if (m.type() == osmium::item_type::node) {
            auto const v = w_.find_node_idx(osm_node_idx_t{m.positive_ref()});
            if (v.has_value()) {
              via = *v;
            }
          }
          break;
      }
    }

    if (via == node_idx_t::invalid() || c->from_.empty() || c->to_.empty()) {
      return;
    }

    auto const l = std::scoped_lock{r_mutex_};
    for (auto const& from : c->from_) {
      for (auto const& to : c->to_) {
        r_.emplace_back(resolved_restriction{restriction_type, from, to, via});
      }
    }
  }

  std::vector<resolved_restriction>& r_;
  std::mutex r_mutex_;

  ways& w_;
};

struct mark_inaccessible_handler : public osm::handler::Handler {
  mark_inaccessible_handler(ways& w) : w_{w} {}

  void node(osm::Node const& n) {
    auto const t = tags{n};
    auto const accessible =
        is_accessible<car_profile>(t, osm_obj_type::kNode) &&
        is_accessible<bike_profile>(t, osm_obj_type::kNode) &&
        is_accessible<foot_profile>(t, osm_obj_type::kNode);
    if (!accessible) {
      w_.node_way_counter_.increment(n.positive_id());
    }
  }

  ways& w_;
};

struct rel_ways_handler : public osm::handler::Handler {
  rel_ways_handler(hash_map<osm_way_idx_t, way_properties>& rel_ways)
      : rel_ways_{rel_ways} {}

  void relation(osm::Relation const& r) {
    auto const p = get_way_properties(tags{r});
    if (!p.is_accessible()) {
      return;
    }

    for (auto const& m : r.members()) {
      if (m.type() == osm::item_type::way) {
        rel_ways_.emplace(osm_way_idx_t{m.positive_ref()}, p);
      }
    }
  }

  hash_map<osm_way_idx_t, way_properties>& rel_ways_;
};

void extract(fs::path const& in, fs::path const& out) {
  auto ec = std::error_code{};
  fs::remove_all(out, ec);
  if (!fs::is_directory(out)) {
    fs::create_directories(out);
  }

  auto input_file = osm_io::File{};
  auto file_size = std::size_t{0U};
  try {
    input_file = osm_io::File{in.generic_string()};
    file_size =
        osm_io::Reader{input_file, osmium::io::read_meta::no}.file_size();
  } catch (...) {
    fmt::println("load_osm failed [file={}]", in);
    throw;
  }

  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  auto const node_idx_file =
      tiles::tmp_file{(out / "idx.bin").generic_string()};
  auto const node_dat_file =
      tiles::tmp_file{(out / "dat.bin").generic_string()};
  auto node_idx =
      tiles::hybrid_node_idx{node_idx_file.fileno(), node_dat_file.fileno()};

  auto rel_ways = hash_map<osm_way_idx_t, way_properties>{};
  auto w = ways{std::move(out), cista::mmap::protection::WRITE};
  w.node_way_counter_.reserve(12000000000);
  {  // Collect node coordinates.
    pt->status("Load OSM / Coordinates").in_high(file_size).out_bounds(0, 20);

    auto node_idx_builder = tiles::hybrid_node_idx_builder{node_idx};

    auto inaccessilbe_handler = mark_inaccessible_handler{w};
    auto rel_ways_h = rel_ways_handler{rel_ways};
    auto reader = osm_io::Reader{input_file, osm_eb::node | osm_eb::relation,
                                 osmium::io::read_meta::no};
    while (auto buffer = reader.read()) {
      pt->update(reader.offset());
      osm::apply(buffer, node_idx_builder, inaccessilbe_handler, rel_ways_h);
    }
    reader.close();
    node_idx_builder.finish();
  }

  {  // Extract streets, places, and areas.
    pt->status("Load OSM / Ways").in_high(file_size).out_bounds(20, 50);

    auto const thread_count = std::max(2U, std::thread::hardware_concurrency());

    // pool must be destructed before handlers!
    auto pool =
        osmium::thread::Pool{static_cast<int>(thread_count), thread_count * 8U};

    auto reader = osm_io::Reader{input_file, pool, osm_eb::way,
                                 osmium::io::read_meta::no};
    auto seq_reader = tiles::sequential_until_finish<osm_mem::Buffer>{[&] {
      pt->update(reader.offset());
      return reader.read();
    }};
    auto has_exception = std::atomic_bool{false};
    auto workers = std::vector<std::future<void>>{};
    workers.reserve(thread_count / 2U);
    auto h = way_handler{w, rel_ways};
    auto queue = tiles::in_order_queue<osm_mem::Buffer>{};
    for (auto i = 0U; i < thread_count / 2U; ++i) {
      workers.emplace_back(pool.submit([&] {
        try {
          while (true) {
            auto opt = seq_reader.process();
            if (!opt.has_value()) {
              break;
            }

            auto& [idx, buf] = *opt;
            tiles::update_locations(node_idx, buf);

            queue.process_in_order(idx, std::move(buf),
                                   [&](auto buf2) { osm::apply(buf2, h); });
          }
        } catch (std::exception const& e) {
          fmt::print(std::clog, "EXCEPTION CAUGHT: {} {}\n",
                     std::this_thread::get_id(), e.what());
          has_exception = true;
        } catch (...) {
          fmt::print(std::clog, "UNKNOWN EXCEPTION CAUGHT: {} \n",
                     std::this_thread::get_id());
          has_exception = true;
        }
      }));
    }

    utl::verify(!workers.empty(), "have no workers");
    for (auto& worker : workers) {
      worker.wait();
    }

    utl::verify(!has_exception, "load_osm: exception caught!");

    reader.close();
    pt->update(pt->in_high_);

    reader.close();
  }

  w.connect_ways();

  auto r = std::vector<resolved_restriction>{};
  {
    pt->status("Load OSM / Node Properties")
        .in_high(file_size)
        .out_bounds(90, 100);
    auto const thread_count = std::max(2U, std::thread::hardware_concurrency());

    // pool must be destructed before handlers!
    auto pool =
        osmium::thread::Pool{static_cast<int>(thread_count), thread_count * 8U};

    auto reader =
        osm_io::Reader{input_file, pool, osm_eb::node | osm_eb::relation,
                       osmium::io::read_meta::no};
    auto seq_reader = tiles::sequential_until_finish<osm_mem::Buffer>{[&] {
      pt->update(reader.offset());
      return reader.read();
    }};
    auto h = node_handler{w, r};
    auto has_exception = std::atomic_bool{false};
    auto workers = std::vector<std::future<void>>{};
    workers.reserve(thread_count / 2U);
    for (auto i = 0U; i < thread_count / 2U; ++i) {
      workers.emplace_back(pool.submit([&] {
        try {
          while (true) {
            auto opt = seq_reader.process();
            if (!opt.has_value()) {
              break;
            }

            auto& [idx, buf] = *opt;
            osm::apply(buf, h);
          }
        } catch (std::exception const& e) {
          fmt::print(std::clog, "EXCEPTION CAUGHT: {} {}\n",
                     std::this_thread::get_id(), e.what());
          has_exception = true;
        } catch (...) {
          fmt::print(std::clog, "UNKNOWN EXCEPTION CAUGHT: {} \n",
                     std::this_thread::get_id());
          has_exception = true;
        }
      }));
    }

    utl::verify(!workers.empty(), "have no workers");
    for (auto& worker : workers) {
      worker.wait();
    }

    utl::verify(!has_exception, "load_osm: exception caught!");

    reader.close();
    pt->update(pt->in_high_);

    reader.close();
  }

  w.add_restriction(r);
}

}  // namespace osr
