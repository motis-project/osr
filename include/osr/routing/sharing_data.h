#pragma once

#include <vector>

#include "geo/latlng.h"

#include "osr/routing/additional_edge.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data {
  geo::latlng get_additional_node_coordinates(node_idx_t const n) const {
    return additional_node_coordinates_.at(to_idx(n) - additional_node_offset_);
  }

  bool is_additional_node(node_idx_t const n) const {
    return n != node_idx_t::invalid() && to_idx(n) >= additional_node_offset_;
  }

  way_pos_t get_way_pos(ways::routing const& w,
                        node_idx_t const node,
                        way_idx_t const way) const {
    if (is_additional_node(node)) {
      if (auto it = additional_edges_.find(node);
          it != end(additional_edges_)) {
        auto const& edges = it->second;
        for (auto const [i, ae] : utl::enumerate(edges)) {
          if (ae.underlying_way_ == way) {
            return static_cast<way_pos_t>(i);
          }
        }
      }
      return 0;
    }
    return w.get_way_pos(node, way);
  }

  bitvec<node_idx_t> const* start_allowed_{};
  bitvec<node_idx_t> const* end_allowed_{};
  bitvec<node_idx_t> const* through_allowed_{};

  node_idx_t::value_t additional_node_offset_{};
  std::vector<geo::latlng> const& additional_node_coordinates_;
  hash_map<node_idx_t, std::vector<additional_edge>> const& additional_edges_;
};

inline bool is_allowed(bitvec<node_idx_t> const* b, node_idx_t const n) {
  return b == nullptr || b->test(n);
}

}  // namespace osr
