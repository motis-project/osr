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
#include "osr/node_info.h"
#include "osr/way_info.h"

namespace osm = osmium;
namespace osm_io = osmium::io;
namespace osm_rel = osmium::relations;
namespace osm_eb = osmium::osm_entity_bits;
namespace osm_area = osmium::area;
namespace osm_mem = osmium::memory;
namespace fs = std::filesystem;

namespace osr {

constexpr auto const kFlushSize = 100'000;

struct feature_handler : public osmium::handler::Handler {
  feature_handler(db& db) : db_{db} {}

  ~feature_handler() {
    flush_nodes();
    flush_rels();
  }

  void way(osmium::Way const& w) {
    auto const i = osm_way_idx_t{w.id()};
    rel_cache_[i].nodes_ =
        utl::to_vec(w.nodes(), [&](osmium::NodeRef const& x) {
          node_cache_[osm_node_idx_t{x.ref()}].ways_.emplace_back(i);
          return osm_node_idx_t{x.ref()};
        });

    if (rel_cache_.size() > kFlushSize) {
      flush_rels();
    }
  }

  void node(osmium::Node const& n) {
    node_cache_[osm_node_idx_t{n.id()}].p_ = point::from_location(n.location());

    if (node_cache_.size() > kFlushSize) {
      flush_nodes();
    }
  }

  void flush_nodes() {
    db_.write(node_cache_);
    node_cache_.clear();
  }

  void flush_rels() {
    db_.write(rel_cache_);
    rel_cache_.clear();
  }

  db& db_;
  hash_map<osm_node_idx_t, node_info> node_cache_;
  hash_map<osm_way_idx_t, way_info> rel_cache_;
};

void extract(config const& conf) {
  auto const& in = conf.in_;
  auto const& tmp = conf.tmp_;
  auto const& db_out = conf.db_;
  auto const& db_max_size = conf.db_max_size_;

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

  auto pt = utl::get_active_progress_tracker_or_activate("import");
  pt->status("Load OSM").out_mod(3.F).in_high(2 * file_size);

  auto const node_idx_file =
      tiles::tmp_file{(tmp / "idx.bin").generic_string()};
  auto const node_dat_file =
      tiles::tmp_file{(tmp / "dat.bin").generic_string()};
  auto node_idx =
      tiles::hybrid_node_idx{node_idx_file.fileno(), node_dat_file.fileno()};

  {  // Collect node coordinates.
    pt->status("Load OSM / Pass 1");
    auto node_idx_builder = tiles::hybrid_node_idx_builder{node_idx};

    auto reader = osm_io::Reader{input_file, osm_eb::node | osm_eb::relation,
                                 osmium::io::read_meta::no};
    while (auto buffer = reader.read()) {
      pt->update(reader.offset());
      osm::apply(buffer, node_idx_builder);
    }
    reader.close();

    node_idx_builder.finish();
    std::clog << "Hybrid Node Index Statistics:\n";
    node_idx_builder.dump_stats();
  }

  auto mp_queue = tiles::in_order_queue<osm_mem::Buffer>{};
  auto db = osr::db{db_out, db_max_size};
  {  // Extract.
    pt->status("Load OSM / Pass 2");
    auto const thread_count =
        std::max(2, static_cast<int>(std::thread::hardware_concurrency()));

    // poor mans thread local (we don't know the threads themselves)
    auto next_handlers_slot = std::atomic_size_t{0U};
    auto handlers = std::vector<std::pair<std::thread::id, feature_handler>>{};
    handlers.reserve(thread_count);
    for (auto i = 0; i < thread_count; ++i) {
      handlers.emplace_back(std::thread::id{}, feature_handler{db});
    }
    auto const get_handler = [&]() -> feature_handler& {
      auto const thread_id = std::this_thread::get_id();
      if (auto it = std::find_if(
              begin(handlers), end(handlers),
              [&](auto const& pair) { return pair.first == thread_id; });
          it != end(handlers)) {
        return it->second;
      }
      auto slot = next_handlers_slot.fetch_add(1);
      utl::verify(slot < handlers.size(), "more threads than expected");
      handlers[slot].first = thread_id;
      return handlers[slot].second;
    };

    // pool must be destructed before handlers!
    auto pool = osmium::thread::Pool{thread_count,
                                     static_cast<size_t>(thread_count * 8)};

    auto reader = osm_io::Reader{input_file, pool, osmium::io::read_meta::no};
    auto seq_reader = tiles::sequential_until_finish<osm_mem::Buffer>{[&] {
      pt->update(reader.file_size() + reader.offset());
      return reader.read();
    }};

    std::atomic_bool has_exception{false};
    std::vector<std::future<void>> workers;
    auto handler = feature_handler{db};
    workers.reserve(thread_count / 2);
    for (auto i = 0; i < thread_count / 2; ++i) {
      workers.emplace_back(pool.submit([&] {
        try {
          while (true) {
            auto opt = seq_reader.process();
            if (!opt.has_value()) {
              break;
            }

            auto& [idx, buf] = *opt;
            tiles::update_locations(node_idx, buf);
            osm::apply(buf, handler);

            mp_queue.process_in_order(idx, std::move(buf), [&](auto buf2) {
              auto p = std::make_shared<osm_mem::Buffer>(
                  std::forward<decltype(buf2)>(buf2));
              pool.submit([&, p] { osm::apply(*p, get_handler()); });
            });
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
    utl::verify(mp_queue.queue_.empty(), "mp_queue not empty!");

    reader.close();
    pt->update(pt->in_high_);
  }

  db.write_graph(conf.graph_, conf.node_map_, conf.edge_map_);
}

}  // namespace osr
