#pragma once

#include <cassert>
#include <optional>
#include <utility>

#include "osr/routing/profile.h"
#include "osr/ways.h"

namespace osr {

struct additional_edge;
struct sharing_data;

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const&,
                              ways::routing const&,
                              typename P::node,
                              sharing_data const*,
                              std::optional<routing_time_t>,
                              duration_t,
                              direction,
                              Fn&&);

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          bool IsBus,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const&,
                            ways::routing const&,
                            typename P::node,
                            bitvec<node_idx_t> const*,
                            cost_t,
                            std::optional<routing_time_t>,
                            duration_t,
                            direction,
                            Fn&&);

template <WayAwareProfile P>
constexpr cost_t get_profile_turn_cost(typename P::parameters const& params,
                                       ways::routing const& w,
                                       typename P::node const n,
                                       way_pos_t const way_pos,
                                       direction const way_dir,
                                       bool const is_u_turn) {
  if (is_u_turn) {
    return 0U;
  }

  return P::turn_cost(params,
                      w.get_turn_angle(n.n_, n.way_, n.dir_, way_pos, way_dir));
}

template <WayAwareProfile P>
constexpr cost_t get_profile_additional_turn_cost(
    typename P::parameters const& params,
    ways::routing const& w,
    typename P::node const n,
    sharing_data const* additional,
    additional_edge const& ae,
    direction const edge_dir,
    bool const is_u_turn) {
  if (additional == nullptr || additional->is_additional_node(n.n_)) {
    return 0U;
  }

  return get_profile_turn_cost<P>(
      params, w, n, additional->get_way_pos(w, n.n_, ae.underlying_way_),
      edge_dir, is_u_turn);
}

template <WayAwareProfile P>
constexpr duration_t get_profile_edge_duration(
    typename P::parameters const& params,
    ways::routing const& w,
    typename P::node const from,
    typename P::node const to,
    way_idx_t const way,
    way_properties const& way_props,
    direction const way_dir,
    distance_t const dist,
    cost_t const way_cost,
    cost_t const node_cost,
    cost_t const turn_cost,
    bool const is_u_turn,
    cost_t const uturn_penalty) {
  if constexpr (requires {
                  P::edge_duration(params, w, from, to, way, way_props, way_dir,
                                   dist, way_cost, node_cost, turn_cost,
                                   is_u_turn, uturn_penalty);
                }) {
    return P::edge_duration(params, w, from, to, way, way_props, way_dir, dist,
                            way_cost, node_cost, turn_cost, is_u_turn,
                            uturn_penalty);
  } else {
    return duration_from_cost(clamp_cost(
        static_cast<std::uint64_t>(way_cost) + node_cost + turn_cost +
        (is_u_turn ? static_cast<std::uint64_t>(uturn_penalty) : 0U)));
  }
}

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const& params,
                              ways::routing const& w,
                              typename P::node const n,
                              sharing_data const* additional,
                              Fn&& fn) {
  for_each_additional_edge<P>(params, w, n, additional, std::nullopt,
                              duration_t{0}, direction::kForward,
                              std::forward<Fn>(fn));
}

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const& params,
                              ways::routing const& w,
                              typename P::node const n,
                              sharing_data const* additional,
                              std::optional<routing_time_t> const start_time,
                              duration_t const current_duration,
                              direction const search_dir,
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
          P::way_cost(params, w, ae.underlying_way_, way_props, edge_dir,
                      ae.distance_, start_time, current_duration, search_dir);
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
std::tuple<typename P::node, cost_t, duration_t> get_adjacent_additional_node(
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
  auto const turn_cost = get_profile_additional_turn_cost<P>(
      params, w, n, additional, ae, edge_dir, is_u_turn);
  if (turn_cost == kInfeasible) {
    return {P::node::invalid(), kInfeasible, duration_t{0}};
  }

  auto const target = typename P::node{
      ae.to_, additional->get_way_pos(w, ae.to_, ae.underlying_way_), edge_dir};
  auto cost = clamp_cost(static_cast<std::uint64_t>(edge_cost) + turn_cost);

  if (is_u_turn) {
    cost = clamp_cost(static_cast<std::uint64_t>(cost) + uturn_penalty);
  }

  if (!additional->is_additional_node(ae.to_)) {
    cost = clamp_cost(static_cast<std::uint64_t>(cost) +
                      P::node_cost(params, w.node_properties_[ae.to_]));
  }

  auto const node_cost = additional->is_additional_node(ae.to_)
                             ? cost_t{0U}
                             : P::node_cost(params, w.node_properties_[ae.to_]);
  return {target, cost,
          get_profile_edge_duration<P>(
              params, w, n, target, ae.underlying_way_,
              w.way_properties_[ae.underlying_way_], edge_dir, ae.distance_,
              edge_cost, node_cost, turn_cost, is_u_turn, uturn_penalty)};
}

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          bool IsBus = false,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const& params,
                            ways::routing const& w,
                            typename P::node const n,
                            bitvec<node_idx_t> const* blocked,
                            cost_t const uturn_penalty,
                            Fn&& fn) {
  for_each_adjacent_node<P, SearchDir, WithBlocked, WithRestrictions, IsBus>(
      params, w, n, blocked, uturn_penalty, std::nullopt, duration_t{0},
      SearchDir, std::forward<Fn>(fn));
}

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          bool IsBus = false,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const& params,
                            ways::routing const& w,
                            typename P::node const n,
                            bitvec<node_idx_t> const* blocked,
                            cost_t const uturn_penalty,
                            std::optional<routing_time_t> const start_time,
                            duration_t const current_duration,
                            direction const search_dir,
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
      if (P::way_cost(params, w, way, target_way_prop, way_dir, 0U, start_time,
                      current_duration, search_dir) == kInfeasible) {
        return;
      }

      if constexpr (WithRestrictions) {
        if (is_profile_turn_restricted<P, SearchDir, IsBus>(
                params, w, n.n_, n.way_, way_pos, start_time, current_duration,
                search_dir)) {
          return;
        }
      }

      auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
      auto const turn_cost =
          get_profile_turn_cost<P>(params, w, n, way_pos, way_dir, is_u_turn);
      if (turn_cost == kInfeasible) {
        return;
      }

      auto const dist = w.get_way_node_distance(way, std::min(from, to));
      auto const target = typename P::node{
          target_node, w.get_way_pos(target_node, way, to), way_dir};
      auto const wc =
          P::way_cost(params, w, way, target_way_prop, way_dir, dist,
                      start_time, current_duration, search_dir);
      auto const cost =
          wc == kInfeasible
              ? kInfeasible
              : clamp_cost(static_cast<std::uint64_t>(wc) + nc +
                           (is_u_turn ? uturn_penalty : 0U) + turn_cost);
      auto const duration = get_profile_edge_duration<P>(
          params, w, n, target, way, target_way_prop, way_dir, dist, wc, nc,
          turn_cost, is_u_turn, uturn_penalty);
      fn(target, cost, duration, dist, way, from, to,
         elevation_storage::elevation{}, false);
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
