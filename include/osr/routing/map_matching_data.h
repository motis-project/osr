#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "geo/latlng.h"

#include "osr/lookup.h"
#include "osr/routing/astar.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"

namespace osr {

template <Profile P>
struct matched_way {
  typename P::node get_node(direction const dir) const {
    return dir == direction::kForward ? fwd_node_ : bwd_node_;
  }

  cost_t get_cost(direction const dir) const {
    return dir == direction::kForward ? fwd_cost_ : bwd_cost_;
  }

  cost_t get_cost(std::optional<direction> const dir) const {
    return dir ? get_cost(*dir) : std::min(fwd_cost_, bwd_cost_);
  }

  way_idx_t way_{way_idx_t::invalid()};
  double dist_to_way_{};
  geo::latlng projected_point_{};
  unsigned segment_idx_{};
  bool oneway_{};
  node_idx_t additional_node_idx_{node_idx_t::invalid()};
  typename P::node fwd_node_{P::node::invalid()};
  typename P::node bwd_node_{P::node::invalid()};
  node_candidate fwd_out_{};  // outgoing from point, way dir = fwd
  node_candidate fwd_in_{};  // incoming to point, way dir = fwd
  node_candidate bwd_out_{};  // outgoing from point, way dir = bwd
  node_candidate bwd_in_{};  // incoming to point, way dir = bwd
  cost_t fwd_cost_{kInfeasible};
  cost_t bwd_cost_{kInfeasible};
  std::size_t beeline_from_{0U};
  distance_t beeline_dist_{};
  cost_t match_penalty_{};
};

template <Profile P>
struct point_data {
  location loc_{};
  std::vector<matched_way<P>> matched_ways_{};
};

template <Profile P>
struct segment_data {
  astar<P, true> d_{};
  std::unique_ptr<sharing_data> sharing_{};
  cost_t min_cost_{std::numeric_limits<cost_t>::max()};
  cost_t max_cost_{std::numeric_limits<cost_t>::min()};
  bool all_beelined_{false};
  std::size_t beeline_from_{};
  distance_t beeline_dist_{};

  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_;
  std::size_t path_segments_{};

  std::chrono::microseconds d_dijkstra_{};
};

struct node_ref {
  std::size_t matched_way_idx_{};
  direction dir_{};
};

}  // namespace osr
