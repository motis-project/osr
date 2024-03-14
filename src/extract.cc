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

constexpr auto const kFlushSizeStart = 100'000;
constexpr auto const kFlushSize = 500'000;

template <typename T>
struct poor_mans_thread_local {
  template <typename... Args>
  poor_mans_thread_local(std::size_t const thread_count, Args&&... args) {
    handlers_.reserve(thread_count);
    for (auto i = 0U; i != thread_count; ++i) {
      handlers_.emplace_back(
          std::pair<std::thread::id, T>{std::thread::id{}, T{args...}});
    }
  }

  T& get_handler() {
    auto const thread_id = std::this_thread::get_id();
    if (auto it = std::find_if(
            begin(handlers_), end(handlers_),
            [&](auto const& pair) { return pair.first == thread_id; });
        it != end(handlers_)) {
      return it->second;
    }
    auto slot = next_handlers_slot_.fetch_add(1);
    utl::verify(slot < handlers_.size(), "more threads than expected");
    handlers_[slot].first = thread_id;
    return handlers_[slot].second;
  }

  std::vector<std::pair<std::thread::id, T>> handlers_;
  std::atomic_size_t next_handlers_slot_{0};
};

struct feature_handler : public osmium::handler::Handler {
  feature_handler() = default;
  feature_handler(db& db, std::mutex& flush_mutex)
      : flush_mutex_{&flush_mutex}, db_{&db} {}
  feature_handler(feature_handler const&) = default;
  feature_handler& operator=(feature_handler const&) = default;
  feature_handler(feature_handler&&) = default;
  feature_handler& operator=(feature_handler&&) = default;
  ~feature_handler() { x_flush(); }

  void way(osmium::Way const& w) {
    if (!w.tags().has_key("highway")) {
      return;
    }

    auto const i = osm_way_idx_t{w.id()};
    rel_cache_[i].nodes_ =
        utl::to_vec(w.nodes(), [&](osmium::NodeRef const& x) {
          node_cache_[osm_node_idx_t{x.ref()}].ways_.emplace_back(i);
          return osm_node_idx_t{x.ref()};
        });

    if (rel_cache_.size() > kFlushSize || node_cache_.size() > kFlushSize) {
      x_flush();
      return;
    }

    if (rel_cache_.size() > kFlushSizeStart ||
        node_cache_.size() > kFlushSizeStart) {
      try_flush();
    }
  }

  void x_flush() {
    auto const lock = std::scoped_lock<std::mutex>{*flush_mutex_};

    db_->write(node_cache_, true);
    node_cache_.clear();

    db_->write(rel_cache_);
    rel_cache_.clear();
  }

  void try_flush() {
    auto const lock =
        std::unique_lock<std::mutex>{*flush_mutex_, std::try_to_lock};
    if (lock.owns_lock()) {
      db_->write(node_cache_, true);
      node_cache_.clear();

      db_->write(rel_cache_);
      rel_cache_.clear();
    }
  }

  std::mutex* flush_mutex_{nullptr};
  db* db_{nullptr};
  hash_map<osm_node_idx_t, node_info> node_cache_;
  hash_map<osm_way_idx_t, way_info> rel_cache_;
};

struct node_handler : public osmium::handler::Handler {
  node_handler(db& db, std::mutex& flush_mutex)
      : flush_mutex_{&flush_mutex}, db_{&db} {}

  ~node_handler() { x_flush(); }

  void node(osmium::Node const& n) {
    node_cache_[osm_node_idx_t{n.id()}].p_ = point::from_location(n.location());

    if (node_cache_.size() > kFlushSize) {
      x_flush();
    }
  }

  void x_flush() {
    auto const lock = std::scoped_lock<std::mutex>{*flush_mutex_};
    db_->write(node_cache_, false);
    node_cache_.clear();
  }

  hash_map<osm_node_idx_t, node_info> node_cache_;
  std::mutex* flush_mutex_{nullptr};
  db* db_;
};

void extract(config const& conf) {
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

  auto pt = utl::get_active_progress_tracker_or_activate("import");
  pt->status("Load OSM").out_mod(3.F).in_high(file_size * 2U);

  auto flush_mutex = std::mutex{};
  auto db = osr::db{conf.out_};

  auto handlers = poor_mans_thread_local<feature_handler>{
      static_cast<std::size_t>(
          osmium::thread::Pool::default_instance().num_threads()),
      db, flush_mutex};
  {  // Read ways.
    auto& thread_pool = osmium::thread::Pool::default_instance();
    auto reader = osmium::io::Reader{input_file, osmium::osm_entity_bits::way,
                                     osmium::io::read_meta::no};
    while (auto buffer = reader.read()) {
      pt->update(reader.offset());
      auto p = std::make_shared<osm_mem::Buffer>(
          std::forward<decltype(buffer)>(buffer));
      thread_pool.submit([&, p] { osm::apply(*p, handlers.get_handler()); });
    }
  }

  {  // Read nodes.
    auto reader = osmium::io::Reader{input_file, osmium::osm_entity_bits::node,
                                     osmium::io::read_meta::no};
    auto handler = node_handler{db, flush_mutex};
    while (auto buffer = reader.read()) {
      pt->update(reader.file_size() + reader.offset());
      osmium::apply(buffer, handler);
    }
  }

  std::cout << "Last flush\n";
  handlers.handlers_.clear();  // trigger flush

  std::cout << "Writing graph\n";
  db.write_graph();
}

}  // namespace osr
