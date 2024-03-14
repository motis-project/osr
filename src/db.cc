#include "osr/db.h"

#include <span>

#include "osmium/osm/way.hpp"

#include "lmdb/lmdb.hpp"

#include "utl/get_or_create.h"
#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/buf_io.h"
#include "osr/graph.h"
#include "osr/mmap_vec.h"
#include "osr/node_info.h"
#include "osr/point.h"
#include "osr/way_info.h"

namespace fs = std::filesystem;

// Binary layout:
// - point [4 bytes lat, 4 bytes lng]
// - [8 bytes wayation index, ...]   <-- invariant: sorted, unique
constexpr auto const kNodeDb = "NODE_DB";

// Binary layout
// - 8 byte edge flags
// - [8 bytes osm node index, ...]   <-- exact order
constexpr auto const kWayDb = "REL_NODES_DB";

namespace osr {

struct db::impl {
  impl(fs::path const& path, std::uint64_t const max_size) {
    env_.set_maxdbs(2);
    env_.set_mapsize(max_size);
    env_.open(path.c_str(),
              lmdb::env_open_flags::NOSUBDIR | lmdb::env_open_flags::NOSYNC);

    auto t = lmdb::txn{env_};
    t.dbi_open(kNodeDb, lmdb::dbi_flags::CREATE | lmdb::dbi_flags::INTEGERKEY);
    t.dbi_open(kWayDb, lmdb::dbi_flags::CREATE | lmdb::dbi_flags::INTEGERKEY);
    t.commit();
  }

  void write_graph(fs::path const& graph_path,
                   fs::path const& node_map_path,
                   fs::path const& edge_map_path) {
    auto t = lmdb::txn{env_, lmdb::txn_flags::NOSYNC};
    auto nodes_db = t.dbi_open(kNodeDb);

    {  // Assign graph node ids to every node with >1 wayation.
      auto node_map = node_map_t{cista::mmap{node_map_path.c_str()}};

      auto node_idx = node_idx_t{0U};
      auto node = node_info{};
      auto buf = std::vector<std::uint8_t>{};
      auto c = lmdb::cursor{t, nodes_db};
      for (auto el = c.get(lmdb::cursor_op::FIRST); el;
           el = c.get(lmdb::cursor_op::NEXT)) {
        if (!node_info::has_more_than_one_way(el->second)) {
          continue;
        }

        auto const osm_node_idx = lmdb::as_int<osm_node_idx_t>(el->first);

        node.read(el->second);
        node.n_ = node_idx++;
        node.write(buf);
        c.put(el->first, view(buf));

        node_map.push_back(osm_node_idx);
      }
    }

    auto g = graph{};
    {  // Create graph edges.
      auto edge_map = edge_map_t{cista::mmap{edge_map_path.c_str()}};

      auto way = way_info{};
      auto nodes = std::vector<std::tuple<node_idx_t, point, distance_t>>{};
      auto out = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
      auto in = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
      auto ways_db = t.dbi_open(kWayDb);
      auto c = lmdb::cursor{t, ways_db};
      for (auto el = c.get(lmdb::cursor_op::FIRST); el;
           el = c.get(lmdb::cursor_op::NEXT)) {
        nodes.clear();
        way.read(el->second);

        auto const dbg =
            false;  // lmdb::as_int<osm_way_idx_t>(el->first) == 913798295;
        auto const print_dbg = [&](auto& a, auto&&... x) {
          if (dbg) {
            fmt::println(fmt::runtime(a), std::forward<decltype(x)>(x)...);
          }
        };

        // collect graph nodes
        auto pred_pos = std::make_optional<point>();
        auto distance = 0.0;
        for (auto const& n : way.nodes_) {
          auto node = t.get(nodes_db, to_idx(n));

          if (!node.has_value()) {
            fmt::println("node {} not found", n);
            continue;
          }

          auto const pos = node_info::read_point(*node);
          print_dbg("node: n={}, node_idx={} at pos={}", n,
                    node_info::read_node_idx(*node), pos);
          if (pred_pos.has_value()) {
            distance += geo::distance(pos, *pred_pos);
          }

          if (auto const node_idx = node_info::read_node_idx(*node);
              node_idx != node_idx_t::invalid()) {
            print_dbg("  routing graph node!");
            nodes.emplace_back(node_idx, pos, distance);
            distance = 0.0;
          }

          pred_pos = pos;
        }

        // build edges
        for (auto const [a, b] : utl::pairwise(nodes)) {
          auto const [from, from_pos, _] = a;
          auto const [to, to_pos, dist] = b;
          auto edge_idx = g.add_edge(from, to, way.edge_flags_, dist);
          edge_map.push_back(lmdb::as_int<osm_way_idx_t>(el->first));
          out[from].push_back(edge_idx);
          in[to].push_back(edge_idx);
          print_dbg("edge [{}]: {} -> {}", edge_idx, from, to);
        }
      }

      for (auto const edges : out) {
        g.out_edges_.emplace_back(edges);
      }

      for (auto const edges : in) {
        g.in_edges_.emplace_back(edges);
      }
    }

    g.write(graph_path);
  }

  void write(hash_map<osm_way_idx_t, way_info>& ways) {
    auto l = std::lock_guard<std::mutex>{flush_mutex_};

    auto buf = std::vector<std::uint8_t>{};
    auto t = lmdb::txn{env_, lmdb::txn_flags::NOSYNC};
    auto db = t.dbi_open(kWayDb);

    for (auto const& [r, way] : ways) {
      way.write(buf);
      t.put(db, to_idx(r), view(buf));
    }

    t.commit();
  }

  void write(hash_map<osm_node_idx_t, node_info>& nodes, bool const create) {
    auto l = std::lock_guard<std::mutex>{flush_mutex_};

    auto buf = std::vector<std::uint8_t>{};
    auto t = lmdb::txn{env_, lmdb::txn_flags::NOSYNC};
    auto db = t.dbi_open(kNodeDb);
    auto existing_node = node_info{};

    for (auto& [n, node] : nodes) {
      utl::sort(node.ways_);

      auto const existing = t.get(db, to_idx(n));
      if (existing.has_value()) {
        existing_node.read(*existing);
        existing_node.merge(node);
        existing_node.write(buf);
        t.put(db, to_idx(n), view(buf));
      } else if (create) {
        node.write(buf);
        t.put(db, to_idx(n), view(buf));
      }
    }

    t.commit();
  }

  point get_node_pos(osm_node_idx_t const i) {
    auto t = lmdb::txn{env_, lmdb::txn_flags::RDONLY};
    auto db = t.dbi_open(kNodeDb);
    auto const val = t.get(db, to_idx(i));
    utl::verify(val.has_value(), "osm node {} not found", i);
    return node_info::read_point(*val);
  }

  void get_node(osm_node_idx_t const i, node_info& nfo) {
    auto t = lmdb::txn{env_, lmdb::txn_flags::RDONLY};
    auto db = t.dbi_open(kNodeDb);
    auto const val = t.get(db, to_idx(i));
    utl::verify(val.has_value(), "osm node {} not found", i);
    nfo.read(*val);
  }

  void get_way(osm_way_idx_t const i, way_info& nfo) {
    auto t = lmdb::txn{env_, lmdb::txn_flags::RDONLY};
    auto db = t.dbi_open(kWayDb);
    auto const val = t.get(db, to_idx(i));
    utl::verify(val.has_value(), "osm node {} not found", i);
    nfo.read(*val);
  }

  lmdb::env env_;
  std::mutex flush_mutex_;
};

db::db(fs::path const& path, std::uint64_t const max_size)
    : impl_(std::make_unique<impl>(path, max_size)) {}

db::~db() = default;

void db::write_graph(fs::path const& graph_path,
                     fs::path const& node_map_path,
                     fs::path const& edge_map_path) {
  impl_->write_graph(graph_path, node_map_path, edge_map_path);
}

point db::get_node_pos(osm_node_idx_t i) { return impl_->get_node_pos(i); }
void db::get_node(osm_node_idx_t i, node_info& nfo) { impl_->get_node(i, nfo); }
void db::get_way(osm_way_idx_t i, way_info& nfo) { impl_->get_way(i, nfo); }

void db::write(hash_map<osm_way_idx_t, way_info>& x) { impl_->write(x); }
void db::write(hash_map<osm_node_idx_t, node_info>& x, bool const create) {
  impl_->write(x, create);
}

}  // namespace osr