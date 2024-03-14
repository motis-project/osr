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
#include "osr/paged.h"
#include "osr/point.h"
#include "osr/way_info.h"

namespace fs = std::filesystem;

namespace osr {

struct db::impl {
  impl(fs::path const& p) : path_{p} {}

  void write_graph() {
    {  // Assign graph node ids to every node with >1 wayation.
      auto node_map = node_map_t{cista::mmap{(path_ / "node_map.bin").c_str()}};

      auto node_idx = node_idx_t{0U};
      auto node = node_info{};
      auto buf = std::vector<std::uint8_t>{};
      for (auto const& [osm_node_idx, _] : *nodes_.map_) {
        auto const val = *nodes_.get(osm_node_idx);
        if (!node_info::has_more_than_one_way(val)) {
          continue;
        }
        node_info::write_node_idx(val, node_idx++);
        node_map.push_back(osm_node_idx);
      }
    }

    auto g = graph{};
    {  // Create graph edges.
      auto edge_map = edge_map_t{cista::mmap{(path_ / "edge_map.bin").c_str()}};

      auto way = way_info{};
      auto nodes = std::vector<std::tuple<node_idx_t, point, distance_t>>{};
      auto out = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
      auto in = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
      for (auto const& [osm_way_idx, _] : *ways_.map_) {
        nodes.clear();

        auto const val = *ways_.get(osm_way_idx);
        way.read(val);

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
          auto node = nodes_.get(n);

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
          auto const [from, from_pos, _d] = a;
          auto const [to, to_pos, dist] = b;
          auto edge_idx = g.add_edge(from, to, way.edge_flags_, dist);
          edge_map.push_back(osm_way_idx);
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

    g.write(path_ / "graph.bin");
    nodes_.write_index(path_ / "nodes.index.bin");
    ways_.write_index(path_ / "ways.index.bin");
  }

  void write(hash_map<osm_way_idx_t, way_info>& ways) {
    auto buf = std::vector<std::uint8_t>{};
    for (auto const& [r, way] : ways) {
      way.write(buf);
      ways_.put(r, view(buf));
    }
  }

  void write(hash_map<osm_node_idx_t, node_info>& nodes, bool const create) {
    auto buf = std::vector<std::uint8_t>{};
    auto existing_node = node_info{};

    for (auto& [n, node] : nodes) {
      utl::sort(node.ways_);

      auto const existing = nodes_.get(n);
      if (existing.has_value()) {
        existing_node.read(*existing);
        existing_node.merge(node);
        existing_node.write(buf);
        nodes_.put(n, view(buf));
      } else if (create) {
        node.write(buf);
        nodes_.put(n, view(buf));
      }
    }
  }

  point get_node_pos(osm_node_idx_t const i) {
    auto const val = nodes_.get(i);
    utl::verify(val.has_value(), "osm node {} not found", i);
    return node_info::read_point(*val);
  }

  void get_node(osm_node_idx_t const i, node_info& nfo) {
    auto const val = nodes_.get(i);
    utl::verify(val.has_value(), "osm node {} not found", i);
    nfo.read(*val);
  }

  void get_way(osm_way_idx_t const i, way_info& nfo) {
    auto const val = ways_.get(i);
    utl::verify(val.has_value(), "osm node {} not found", i);
    nfo.read(*val);
  }

  fs::path path_;
  mmm<osm_node_idx_t> nodes_{path_ / "nodes.data.bin"};
  mmm<osm_way_idx_t> ways_{path_ / "ways.data.bin"};
};

db::db(fs::path const& p) : impl_{std::make_unique<impl>(p)} {}

db::~db() = default;

void db::write_graph() { impl_->write_graph(); }

point db::get_node_pos(osm_node_idx_t i) { return impl_->get_node_pos(i); }
void db::get_node(osm_node_idx_t i, node_info& nfo) { impl_->get_node(i, nfo); }
void db::get_way(osm_way_idx_t i, way_info& nfo) { impl_->get_way(i, nfo); }

void db::write(hash_map<osm_way_idx_t, way_info>& x) { impl_->write(x); }
void db::write(hash_map<osm_node_idx_t, node_info>& x, bool const create) {
  impl_->write(x, create);
}

}  // namespace osr