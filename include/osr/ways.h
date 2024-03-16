#pragma once

#include <filesystem>
#include <ranges>

#include "osmium/osm/way.hpp"

#include "osr/graph.h"
#include "osr/multi_counter.h"
#include "osr/point.h"
#include "osr/types.h"
#include "utl/zip.h"

namespace osr {

constexpr auto const kMode =
    cista::mode::WITH_INTEGRITY | cista::mode::WITH_STATIC_VERSION;

struct ways {
  void way(osmium::Way const& w) {
    if (!w.tags().has_key("highway")) {
      return;
    }

    auto const get_point = [](osmium::NodeRef const& n) {
      return point::from_location(n.location());
    };

    auto const get_node_id = [&](osmium::NodeRef const& n) {
      node_way_counter_.increment(n.positive_ref());
      return osm_node_idx_t{n.positive_ref()};
    };

    way_osm_idx_.push_back(osm_way_idx_t{w.id()});
    way_polylines_.emplace_back(w.nodes() | std::views::transform(get_point));
    way_nodes_.emplace_back(w.nodes() | std::views::transform(get_node_id));
  }

  void write_graph(std::filesystem::path const& p) {
    auto node_map = node_map_t{cista::mmap{(p / "node_map.bin").c_str()}};
    {  // Assign graph node ids to every node with >1 wayation.
      auto node_idx = node_idx_t{0U};
      for (auto i = osm_node_idx_t{0U}; i != node_way_counter_.size(); ++i) {
        if (!node_way_counter_.is_multi(to_idx(i))) {
          continue;
        }
        node_map.push_back(i);
        osm_node_to_graph_.push_back({i, node_idx});
        ++node_idx;
      }
    }

    // Build edges.
    auto g = graph{};
    auto out = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
    auto in = cista::raw::mutable_fws_multimap<node_idx_t, edge_idx_t>{};
    auto edge_map = edge_map_t{cista::mmap{(p / "edge_map.bin").c_str()}};
    for (auto const [osm_way_idx, nodes, polyline] :
         utl::zip(way_osm_idx_, way_nodes_, way_polylines_)) {
      auto pred_pos = std::make_optional<point>();
      auto from = node_idx_t::invalid();
      auto distance = 0.0;

      auto way_edges = way_edges_.add_back_sized(0U);
      for (auto const [osm_node_idx, pos] : utl::zip(nodes, polyline)) {
        if (pred_pos.has_value()) {
          distance += geo::distance(pos, *pred_pos);
        }

        if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
          auto const to = get_node_idx(osm_node_idx);

          if (from != node_idx_t::invalid()) {
            auto edge_idx = g.add_edge(from, to, edge_flags_t{}, distance);
            edge_map.push_back(osm_way_idx);
            out[from].push_back(edge_idx);
            in[to].push_back(edge_idx);
            way_edges.push_back(edge_idx);
          }

          distance = 0.0;
          from = to;
        }

        pred_pos = pos;
      }
    }

    for (auto const edges : out) {
      g.out_edges_.emplace_back(edges);
    }
    for (auto const edges : in) {
      g.in_edges_.emplace_back(edges);
    }

    g.write(p / "graph.bin");

    auto mmap = cista::mmap{(p / "ways.bin").generic_string().c_str(),
                            cista::mmap::protection::WRITE};
    auto writer = cista::buf<cista::mmap>(std::move(mmap));
    cista::serialize<kMode>(writer, *this);
  }

  node_idx_t get_node_idx(osm_node_idx_t const i) {
    auto const it =
        std::lower_bound(begin(osm_node_to_graph_), end(osm_node_to_graph_), i,
                         [](auto&& a, auto&& b) { return a.first < b; });
    utl::verify(it != end(osm_node_to_graph_), "osm node {} not found", i);
    return it->second;
  }

  vector<std::pair<osm_node_idx_t, node_idx_t>> osm_node_to_graph_;
  vector_map<way_idx_t, osm_way_idx_t> way_osm_idx_;
  vecvec<way_idx_t, point, std::uint64_t> way_polylines_;
  vecvec<way_idx_t, osm_node_idx_t, std::uint64_t> way_nodes_;
  vecvec<way_idx_t, edge_idx_t, std::uint64_t> way_edges_;
  multi_counter node_way_counter_;
};

}  // namespace osr