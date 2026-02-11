#pragma once

#include <cassert>
#include <utility>

#include "osr/routing/profile.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const& params,
                              ways::routing const& w,
                              typename P::node const n,
                              sharing_data const* additional,
                              Fn&& fn) {
  assert(additional != nullptr);
  if (auto const it = additional->additional_edges_.find(n.n_);
      it != end(additional->additional_edges_)) {
    for (auto const& ae : it->second) {
      auto const edge_dir =
          ae.reverse_ ? direction::kBackward : direction::kForward;
      assert(ae.underlying_way_ != way_idx_t::invalid());
      auto const way_props = w.way_properties_[ae.underlying_way_];

      auto const edge_cost =
          P::way_cost(params, way_props, edge_dir, ae.distance_);
      if (edge_cost == kInfeasible) {
        continue;
      }

      if (!additional->is_additional_node(ae.to_)) {
        auto const target_node_prop = w.node_properties_[ae.to_];
        if (P::node_cost(params, target_node_prop) == kInfeasible) {
          continue;
        }
      }

      fn(ae, edge_cost, edge_dir);
    }
  }
}

// the following function requires a profile where node has the structure
// node{node_idx_t n_, way_pos_t way_, direction dir_} and get_way function

template <Profile P>
std::pair<typename P::node, cost_t> get_adjacent_additional_node_with_way(
    typename P::parameters const& params,
    ways::routing const& w,
    typename P::node const n,
    sharing_data const* additional,
    additional_edge const& ae,
    direction const edge_dir,
    cost_t const edge_cost,
    cost_t const uturn_penalty) {
  auto const prev_way = n.get_way(w, additional);

  auto const is_u_turn = prev_way == ae.underlying_way_ && n.dir_ != edge_dir;

  auto const target = typename P::node{
      ae.to_, additional->get_way_pos(w, ae.to_, ae.underlying_way_), edge_dir};
  auto cost = edge_cost;

  if (is_u_turn) {
    cost = clamp_cost(static_cast<std::uint64_t>(cost) + uturn_penalty);
  }

  if (!additional->is_additional_node(ae.to_)) {
    cost = clamp_cost(static_cast<std::uint64_t>(cost) +
                      P::node_cost(params, w.node_properties_[ae.to_]));
  }

  return {target, cost};
}

}  // namespace osr
