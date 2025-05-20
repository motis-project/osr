#pragma once

#include "geo/latlng.h"

#include "osr/routing/additional_edge.h"
#include "osr/types.h"

namespace osr {

struct sharing_data {
  bitvec<node_idx_t> const& start_allowed_;
  bitvec<node_idx_t> const& end_allowed_;
  bitvec<node_idx_t> const& through_allowed_;

  geo::latlng get_additional_node_coordinates(node_idx_t const n) const {
    return additional_node_coordinates_.at(to_idx(n) - additional_node_offset_);
  }

  node_idx_t::value_t additional_node_offset_{};
  std::vector<geo::latlng> const& additional_node_coordinates_;
  hash_map<node_idx_t, std::vector<additional_edge>> const& additional_edges_;
};

}  // namespace osr
