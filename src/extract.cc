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

#include "osr/db.h"
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
  void way(osmium::Way const& w) {
    if (!w.tags().has_key("highway")) {
      return;
    }

    auto l = std::scoped_lock<std::mutex>{mutex_};
    w_.way(w);
  }

  std::mutex mutex_;
  ways w_;
};

void extract(config const& conf) {
  auto ec = std::error_code{};
  fs::remove_all(conf.out_, ec);
  if (!fs::is_directory(conf.out_)) {
    fs::create_directories(conf.out_);
  }

  auto input_file = osm_io::File{};
  auto file_size = std::size_t{0U};
  try {
    input_file = osm_io::File{conf.in_};
    file_size =
        osm_io::Reader{input_file, osmium::io::read_meta::no}.file_size();
  } catch (...) {
    fmt::println("load_osm failed [file={}]", conf.in_);
    throw;
  }

  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Load OSM").in_high(file_size * 2U);

  auto const node_idx_file =
      tiles::tmp_file{(conf.out_ / "idx.bin").generic_string()};
  auto const node_dat_file =
      tiles::tmp_file{(conf.out_ / "dat.bin").generic_string()};
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

  auto h = handler{};
  {  // Extract streets, places, and areas.
    pt->status("Load OSM / Pass 2");
    auto const thread_count = std::max(2U, std::thread::hardware_concurrency());

    // pool must be destructed before handlers!
    auto pool =
        osmium::thread::Pool{static_cast<int>(thread_count), thread_count * 8U};

    auto reader = osm_io::Reader{input_file, pool, osm_eb::way,
                                 osmium::io::read_meta::no};
    while (auto buf = reader.read()) {
      pt->update(file_size + reader.offset());
      tiles::update_locations(node_idx, buf);
      osm::apply(buf, h);
    }

    reader.close();
    pt->update(pt->in_high_);
  }

  std::cout << "Writing graph\n";
  h.w_.write_graph(conf.out_);
}

}  // namespace osr
