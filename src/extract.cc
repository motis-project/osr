#include "osr/extract.h"

#include "fmt/core.h"
#include "fmt/std.h"

#include "osmium/area/assembler.hpp"
#include "osmium/area/multipolygon_manager.hpp"
#include "osmium/handler/node_locations_for_ways.hpp"
#include "osmium/index/map/flex_mem.hpp"
#include "osmium/io/pbf_input.hpp"
#include "osmium/io/xml_input.hpp"

#include "utl/progress_tracker.h"
#include "utl/to_vec.h"

#include "tiles/osm/hybrid_node_idx.h"
#include "tiles/osm/tmp_file.h"
#include "tiles/util_parallel.h"

#include "osr/ways.h"

namespace osm = osmium;
namespace osm_io = osmium::io;
namespace osm_rel = osmium::relations;
namespace osm_eb = osmium::osm_entity_bits;
namespace osm_area = osmium::area;
namespace osm_mem = osmium::memory;
namespace fs = std::filesystem;

namespace osr {

struct handler : public osmium::handler::Handler {
  handler(fs::path out) : w_{std::move(out), cista::mmap::protection::WRITE} {
    w_.node_way_counter_.reserve(12000000000);
  }

  void way(osmium::Way const& w) {
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
  }

  std::mutex mutex_;
  ways w_;
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
  pt->status("Load OSM").in_high(file_size * 2U).out_bounds(0, 30);

  auto const node_idx_file =
      tiles::tmp_file{(out / "idx.bin").generic_string()};
  auto const node_dat_file =
      tiles::tmp_file{(out / "dat.bin").generic_string()};
  auto node_idx =
      tiles::hybrid_node_idx{node_idx_file.fileno(), node_dat_file.fileno()};

  {  // Collect node coordinates.
    pt->status("Load OSM / Pass 1");
    auto node_idx_builder = tiles::hybrid_node_idx_builder{node_idx};

    auto reader =
        osm_io::Reader{input_file, osm_eb::node, osmium::io::read_meta::no};
    while (auto buffer = reader.read()) {
      pt->update(reader.offset());
      osm::apply(buffer, node_idx_builder);
    }
    reader.close();
    node_idx_builder.finish();
  }

  auto h = handler{out};
  {  // Extract streets, places, and areas.
    pt->status("Load OSM / Pass 2").out_bounds(30, 60);
    auto const thread_count = std::max(2U, std::thread::hardware_concurrency());

    // pool must be destructed before handlers!
    auto pool =
        osmium::thread::Pool{static_cast<int>(thread_count), thread_count * 8U};

    auto reader = osm_io::Reader{input_file, pool, osm_eb::way,
                                 osmium::io::read_meta::no};
    auto seq_reader = tiles::sequential_until_finish<osm_mem::Buffer>{[&] {
      pt->update(file_size + reader.offset());
      return reader.read();
    }};
    std::atomic_bool has_exception{false};
    std::vector<std::future<void>> workers;
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

  h.w_.connect_ways();
}

}  // namespace osr
