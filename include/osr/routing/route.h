#pragma once

#include <vector>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/parameters.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/with_profile.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <Profile, bool EarlyTermination>
struct dijkstra;

template <Profile, bool EarlyTermination>
struct astar;

template <Profile>
struct bidirectional;

struct sharing_data;

struct route_endpoint_options {
  bool exact_return_at_from_{};
  std::vector<bool> exact_return_at_to_{};

  bool exact_return_at_to(std::size_t const i) const {
    return i < exact_return_at_to_.size() && exact_return_at_to_[i];
  }
};

template <Profile P>
bidirectional<P>& get_bidirectional();

template <Profile P>
dijkstra<P, false>& get_dijkstra();

template <Profile P>
astar<P, false>& get_astar();

template <search_profile profile>
struct search_state {
  using profile_t = profile_selector<profile>::type;

  search_state(profile_parameters const& params,
               ways const& w,
               lookup const& l,
               location const& from,
               std::vector<location> const& to,
               match_view_t const& from_match,
               std::vector<match_t> const& to_match,
               direction dir,
               cost_t const max_distance,
               bitvec<node_idx_t> const* blocked,
               sharing_data const* sharing,
               elevation_storage const* elevations)
      : d_(get_dijkstra<profile_t>()),
        params_(std::get<profile_t::parameters>(params)),
        w_(w),
        l_(l),
        from_(from),
        to_(to),
        from_match_(from_match),
        to_match_(to_match),
        dir_(dir),
        max_distance_(max_distance),
        blocked_(blocked),
        sharing_(sharing),
        elevations_(elevations) {}

  void init() {
    d_.reset(max_distance_);
    for (auto const [i, start] : utl::enumerate(from_match_)) {
      if (can_continue_ && component_seen(w_, from_match_, i)) {
        continue;
      }

      if (utl::none_of(to_match_, [&](auto const dest) {
            return w_.r_->way_component_[start.way_] ==
                   w_.r_->way_component_[dest.way_];
          })) {
        continue;
      }

      auto const start_way = start.way_;
      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max_distance_) {
          profile_t::resolve_start_node(
              *w_.r_, start.way_, nc->node_, from_.lvl_, dir_,
              [&](auto const node) {
                auto label = typename profile_t::label{node, nc->cost_};
                label.track(label, *w_.r_, start_way, node.get_node(), false);
                d_.add_start(w_, label);
              });
        }
      }

      can_continue_ = !d_.run(params_, w_, *w_.r_, distance_, blocked_,
                              sharing_, elevations_, dir_) ||
                      can_continue_;
    }
  }

  bool run(cost_t const distance) {
    distance_ = std::min(distance_ + distance, max_distance_);

    if (d_.pq_.empty() && can_continue_) {
      can_continue_ = false;
      init();
      return can_continue_;
    }

    can_continue_ = !d_.run(params_, w_, *w_.r_, distance_, blocked_, sharing_,
                            elevations_, dir_);
    return can_continue_;
  }

  std::vector<std::optional<path>> results(
      std::function<bool(path const&)> const& do_reconstruct) {
    auto const distance_lng_degrees =
        geo::approx_distance_lng_degrees(from_.pos_);

    auto results = std::vector<std::optional<path>>{};

    for (auto const [m, t] : utl::zip(to_match_, to_)) {
      if (auto const direct = try_direct(from_, t); direct.has_value()) {
        results.push_back(direct);
      } else {
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from_.pos_, t.pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;

        for (auto const& start : from_match_) {
          auto const c =
              best_candidate(params_, w_, d_, t.lvl_, m, distance_, dir_, true,
                             start, limit_squared_max_matching_distance);

          if (c.has_value()) {
            auto [nc, wc, n, p] = *c;
            d_.cost_.at(n.get_key()).write(n, p);
            if (do_reconstruct(p)) {
              p = reconstruct<profile_t>(params_, w_, l_, blocked_, sharing_,
                                         elevations_, d_, from_, t, start, *wc,
                                         *nc, n, p.cost_, dir_);
              p.uses_elevator_ = true;  // TODO: why?
            }
            results.push_back(p);
          }
        }
      }
    }
  }

  dijkstra<profile_t, false> d_;
  profile_t::parameters const& params_;
  ways const& w_;
  lookup const& l_;
  location const& from_;
  std::vector<location> const& to_;
  match_view_t const& from_match_;
  std::vector<match_t> const& to_match_;
  direction const dir_;
  cost_t const max_distance_;
  bitvec<node_idx_t> const* blocked_;
  sharing_data const* sharing_;
  elevation_storage const* elevations_;
  cost_t distance_;
  bool can_continue_ = true;
};

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt,
    route_endpoint_options const& = {});

std::optional<path> route(profile_parameters const&,
                          ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt,
                          route_endpoint_options const& = {});

std::optional<path> route_bidirectional(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    location const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr);

std::optional<path> route_dijkstra(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    location const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::optional<routing_time_t> = std::nullopt);

std::optional<path> route_astar(profile_parameters const&,
                                ways const&,
                                lookup const&,
                                search_profile,
                                location const& from,
                                location const& to,
                                cost_t max,
                                direction,
                                double max_match_distance,
                                bitvec<node_idx_t> const* blocked = nullptr,
                                sharing_data const* sharing = nullptr,
                                elevation_storage const* = nullptr,
                                std::optional<routing_time_t> = std::nullopt);

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile const,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    cost_t const max,
    direction const,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt);

std::optional<path> route(profile_parameters const&,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t const& from_match,
                          match_view_t const& to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt);

}  // namespace osr
