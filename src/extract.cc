#include "osr/extract.h"

#include "fmt/core.h"
#include "fmt/std.h"

#include "osmium/area/assembler.hpp"
#include "osmium/area/multipolygon_manager.hpp"
#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/index/map/flex_mem.hpp"
#include "osmium/io/pbf_input.hpp"
#include "osmium/io/xml_input.hpp"

#include "utl/helpers/algorithm.h"
#include "utl/progress_tracker.h"
#include "utl/to_vec.h"

#include "tiles/osm/hybrid_node_idx.h"
#include "tiles/osm/tmp_file.h"
#include "tiles/util_parallel.h"

#include "osr/ways.h"
#include "utl/parser/arg_parser.h"

namespace osm = osmium;
namespace osm_io = osmium::io;
namespace osm_rel = osmium::relations;
namespace osm_eb = osmium::osm_entity_bits;
namespace osm_area = osmium::area;
namespace osm_mem = osmium::memory;
namespace fs = std::filesystem;
using namespace std::string_view_literals;

namespace osr {

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

speed_limit get_speed_limit(std::string_view highway) {
  switch (cista::hash(highway)) {
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

speed_limit get_speed_limit(osm::Way const& w) {
  auto const& tags = w.tags();
  if (auto const max_speed = tags["maxspeed"];
      max_speed != nullptr &&
      is_number(max_speed) /* TODO: support units (kmh/mph) */) {
    return get_speed_limit(utl::parse<unsigned>(max_speed));
  } else {
    return get_speed_limit(tags["highway"]);
  }
}

std::pair<bool /* car */, bool /* bike */> is_one_way(osm::Way const& w) {
  auto const one_way = w.tags()["oneway"];
  auto const one_way_bike = w.tags()["oneway:bicycle"];
  auto const is_one_way = one_way != nullptr && one_way == "yes"sv;
  return {is_one_way,
          is_one_way && (one_way_bike == nullptr || one_way_bike != "no"sv)};
}

way_properties get_way_properties(osm::Way const& w) {
  auto const highway = w.tags()["highway"];
  auto const [one_way_car, one_way_bike] = is_one_way(w);
  auto const bicycle = w.tags()["bicycle"];
  auto const foot = w.tags()["foot"];
  return {.is_car_accessible_ = is_highway_car_accessible(highway),
          .is_bike_accessible_ = is_highway_bike_accessible(highway) ||
                                 bicycle == "yes"sv ||
                                 bicycle == "designated"sv,
          .is_walk_accessible_ =
              foot != "no"sv && foot != "private"sv &&
              foot != "destination"sv /* TODO */ &&
              (is_highway_walk_accessible(highway) || foot == "yes"sv ||
               foot == "designated"sv || foot == "permissive"sv),
          .is_oneway_car_ = one_way_car,
          .is_oneway_bike_ = one_way_bike,
          .speed_limit_ = get_speed_limit(w)};
}

bool is_access_foot_bike_accessible(char const* access) {
  if (access == nullptr) {
    return true;
  }
  switch (cista::hash(std::string_view{access})) {
    case cista::hash("no"):
    case cista::hash("private"):
    case cista::hash("agricultural"):
    case cista::hash("delivery"): return false;
    default: return true;
  }
}

bool is_access_car_accessible(char const* access) {
  if (access == nullptr) {
    return true;
  }
  switch (cista::hash(std::string_view{access})) {
    case cista::hash("no"):
    case cista::hash("agricultural"):
    case cista::hash("forestry"):
    case cista::hash("emergency"):
    case cista::hash("psv"):
    case cista::hash("customers"):
    case cista::hash("private"):
    case cista::hash("delivery"): return false;
    // case cista::hash("destination"): TODO
    default: return true;
  }
}

bool is_barrier_foot_bike_accessible(char const* barrier) {
  if (barrier == nullptr) {
    return true;
  }

  switch (cista::hash(std::string_view{barrier})) {
    case cista::hash("yes"):
    case cista::hash("wall"):
    case cista::hash("fence"): return false;
    default: return true;
  }
}

bool is_barrier_car_accessible(char const* barrier) {
  if (barrier == nullptr) {
    return true;
  }

  switch (cista::hash(std::string_view{barrier})) {
    case cista::hash("cattle_grid"):
    case cista::hash("border_control"):
    case cista::hash("toll_booth"):
    case cista::hash("sally_port"):
    case cista::hash("gate"):
    case cista::hash("lift_gate"):
    case cista::hash("no"):
    case cista::hash("entrance"):
    case cista::hash("height_restrictor"):  // TODO
    case cista::hash("arch"): return true;
    default: return false;
  }
}

node_properties get_node_properties(osm::Node const& n) {
  auto const access = n.tags()["access"];
  auto const barrier = n.tags()["barrier"];
  auto const is_foot_bike_accessible = is_access_foot_bike_accessible(access) &&
                                       is_barrier_foot_bike_accessible(barrier);
  auto const is_car_accessible =
      is_access_car_accessible(access) && is_barrier_car_accessible(barrier);
  return {
      .is_bike_accessible_ = is_foot_bike_accessible,
      .is_walk_accessible_ = is_foot_bike_accessible,
      .is_car_accessible_ = is_car_accessible,
  };
}

struct way_handler : public osm::handler::Handler {
  way_handler(ways& w) : w_{w} {}

  void way(osm::Way const& w) {
    if (!w.tags().has_key("highway")) {
      return;
    }

    auto l = std::scoped_lock<std::mutex>{mutex_};

    auto const get_point = [](osmium::NodeRef const& n) {
      return point::from_location(n.location());
    };

    auto const get_node_id = [&](osmium::NodeRef const& n) {
      w_.node_way_counter_.increment(n.positive_ref());
      return osm_node_idx_t{n.positive_ref()};
    };

    w_.way_osm_idx_.push_back(osm_way_idx_t{w.id()});
    w_.way_polylines_.emplace_back(w.nodes() |
                                   std::views::transform(get_point));
    w_.way_osm_nodes_.emplace_back(w.nodes() |
                                   std::views::transform(get_node_id));
    w_.way_properties_.emplace_back(get_way_properties(w));
  }

  std::mutex mutex_;
  ways& w_;
};

struct node_handler : public osm::handler::Handler {
  node_handler(ways& w) : w_{w} { w_.node_properties_.resize(w_.n_nodes()); }

  void node(osm::Node const& n) {
    if (auto const node_idx = w_.find_node_idx(osm_node_idx_t{n.id()});
        node_idx.has_value()) {
      w_.node_properties_[*node_idx] = get_node_properties(n);
    }
  }

  ways& w_;
};

struct mark_inaccessible_handler : public osm::handler::Handler {
  mark_inaccessible_handler(ways& w) : w_{w} {}

  void node(osm::Node const& n) {
    auto const access = n.tags()["access"];
    auto const barrier = n.tags()["barrier"];
    auto const accessible = is_access_foot_bike_accessible(access) &&
                            is_barrier_foot_bike_accessible(barrier) &&
                            is_access_car_accessible(access) &&
                            is_barrier_car_accessible(barrier);
    if (!accessible) {
      w_.node_way_counter_.increment(n.id());
    }
  }

  ways& w_;
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
    input_file = osm_io::File{in};
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

  auto w = ways{std::move(out), cista::mmap::protection::WRITE};
  w.node_way_counter_.reserve(12000000000);
  {  // Collect node coordinates.
    pt->status("Load OSM / Coordinates").in_high(file_size).out_bounds(0, 20);

    auto node_idx_builder = tiles::hybrid_node_idx_builder{node_idx};

    auto inaccessilbe_handler = mark_inaccessible_handler{w};
    auto reader =
        osm_io::Reader{input_file, osm_eb::node, osmium::io::read_meta::no};
    while (auto buffer = reader.read()) {
      pt->update(reader.offset());
      osm::apply(buffer, node_idx_builder, inaccessilbe_handler);
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
    auto h = way_handler{w};
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

  w.connect_ways();

  {
    pt->status("Load OSM / Node Properties")
        .in_high(file_size)
        .out_bounds(90, 100);
    auto const thread_count = std::max(2U, std::thread::hardware_concurrency());

    // pool must be destructed before handlers!
    auto pool =
        osmium::thread::Pool{static_cast<int>(thread_count), thread_count * 8U};

    auto reader = osm_io::Reader{input_file, pool, osm_eb::node,
                                 osmium::io::read_meta::no};
    auto seq_reader = tiles::sequential_until_finish<osm_mem::Buffer>{[&] {
      pt->update(reader.offset());
      return reader.read();
    }};
    auto h = node_handler{w};
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
}

}  // namespace osr
