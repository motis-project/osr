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
                              timezone_cache_t const&,
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
                            timezone_cache_t const&,
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

template <WayAwareProfile P, direction SearchDir, bool IsBus>
bool is_profile_turn_restricted(typename P::parameters const& params,
                                ways::routing const& w,
                                timezone_cache_t const& timezones,
                                node_idx_t const n,
                                std::uint8_t const from,
                                std::uint8_t const to,
                                std::optional<routing_time_t> const start_time,
                                duration_t const current_duration,
                                direction const search_dir) {
  if constexpr (requires {
                  P::template is_restricted<SearchDir>(
                      params, w, timezones, n, from, to, start_time,
                      current_duration, search_dir);
                }) {
    return P::template is_restricted<SearchDir>(params, w, timezones, n, from,
                                                to, start_time,
                                                current_duration, search_dir);
  } else {
    return w.template is_restricted<SearchDir, IsBus>(n, from, to);
  }
}

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const& params,
                              ways::routing const& w,
                              timezone_cache_t const& timezones,
                              typename P::node const n,
                              sharing_data const* additional,
                              Fn&& fn) {
  for_each_additional_edge<P>(params, w, timezones, n, additional, std::nullopt,
                              duration_t{0}, direction::kForward,
                              std::forward<Fn>(fn));
}

template <Profile P, typename Fn>
void for_each_additional_edge(typename P::parameters const& params,
                              ways::routing const& w,
                              timezone_cache_t const& timezones,
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

      auto const edge_cost = P::way_cost(
          params, w, timezones, ae.underlying_way_, way_props, edge_dir,
          ae.distance_, start_time, current_duration, search_dir);
      if (edge_cost.cost_ == kInfeasible) {
        continue;
      }

      if (!additional->is_additional_node(ae.to_)) {
        auto const target_node_prop = w.node_properties_[ae.to_];
        if (P::node_cost(params, target_node_prop).cost_ == kInfeasible) {
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
    cost_and_duration const edge_cost,
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
  auto total = clamp_add(edge_cost, turn_cost);

  if (is_u_turn) {
    total = clamp_add(total, uturn_penalty);
  }

  if (!additional->is_additional_node(ae.to_)) {
    total = clamp_add(total, P::node_cost(params, w.node_properties_[ae.to_]));
  }

  return {target, total.cost_, total.duration_};
}

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          bool IsBus = false,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const& params,
                            ways::routing const& w,
                            timezone_cache_t const& timezones,
                            typename P::node const n,
                            bitvec<node_idx_t> const* blocked,
                            cost_t const uturn_penalty,
                            Fn&& fn) {
  for_each_adjacent_node<P, SearchDir, WithBlocked, WithRestrictions, IsBus>(
      params, w, timezones, n, blocked, uturn_penalty, std::nullopt,
      duration_t{0}, SearchDir, std::forward<Fn>(fn));
}

template <WayAwareProfile P,
          direction SearchDir,
          bool WithBlocked,
          bool WithRestrictions,
          bool IsBus = false,
          typename Fn>
void for_each_adjacent_node(typename P::parameters const& params,
                            ways::routing const& w,
                            timezone_cache_t const& timezones,
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
      if (nc.cost_ == kInfeasible) {
        return;
      }

      auto const target_way_prop = w.way_properties_[way];
      if (P::way_cost(params, w, timezones, way, target_way_prop, way_dir, 0U,
                      start_time, current_duration, search_dir)
              .cost_ == kInfeasible) {
        return;
      }

      if constexpr (WithRestrictions) {
        if (is_profile_turn_restricted<P, SearchDir, IsBus>(
                params, w, timezones, n.n_, n.way_, way_pos, start_time,
                current_duration, search_dir)) {
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
          P::way_cost(params, w, timezones, way, target_way_prop, way_dir, dist,
                      start_time, current_duration, search_dir);
      if (wc.cost_ == kInfeasible) {
        return;
      }

      auto total = clamp_add(clamp_add(wc, nc), turn_cost);
      if (is_u_turn) {
        total = clamp_add(total, uturn_penalty);
      }
      fn(target, total.cost_, total.duration_, dist, way, from, to,
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
