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

template <WayAwareProfile P>
std::pair<typename P::node, cost_t> get_adjacent_additional_node(
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

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const& params,
                            ways::routing const& w,
                            typename P::node const n,
                            bitvec<node_idx_t> const* blocked,
                            cost_t const uturn_penalty,
                            Fn&& fn) {
  auto way_pos = way_pos_t{0U};
  for (auto const [way, i] :
       utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
    auto const expand = [&](direction const way_dir, std::uint16_t const from,
                            std::uint16_t const to) {
      // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
      auto const target_node = w.way_nodes_[way][to];
      if constexpr (WithBlocked) {
        if (blocked->test(target_node)) {
          return;
        }
      }

      auto const target_node_prop = w.node_properties_[target_node];
      auto const nc = P::node_cost(params, target_node_prop);
      if (nc == kInfeasible) {
        return;
      }

      auto const target_way_prop = w.way_properties_[way];
      if (P::way_cost(params, target_way_prop, way_dir, 0U) == kInfeasible) {
        return;
      }

      if constexpr (WithRestrictions) {
        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }
      }

      auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
      auto const dist = w.get_way_node_distance(way, std::min(from, to));
      auto const target = typename P::node{
          target_node, w.get_way_pos(target_node, way, to), way_dir};
      auto const wc = P::way_cost(params, target_way_prop, way_dir, dist);
      auto const cost = wc == kInfeasible
                            ? kInfeasible
                            : clamp_cost(static_cast<std::uint64_t>(wc) + nc +
                                         (is_u_turn ? uturn_penalty : 0U));
      fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
         false);
    };

    if (i != 0U) {
      expand(flip<SearchDir>(direction::kBackward), i, i - 1);
    }
    if (i != w.way_nodes_[way].size() - 1U) {
      expand(flip<SearchDir>(direction::kForward), i, i + 1);
    }

    ++way_pos;
  }
}

}  // namespace osr
