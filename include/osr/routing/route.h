#pragma once

#include <iterator>
#include <vector>

#include "utl/enumerate.h"

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

constexpr auto const kMaxMatchingDistanceSquaredRatio = 9.0;

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

bool component_seen(ways const&,
                    match_view_t const&,
                    size_t,
                    unsigned times = 1);

template <Profile P, typename Search>
std::optional<
    std::tuple<candidate_node, way_idx_t, bool, typename P::node, path>>
best_candidate(typename P::parameters const&,
               ways const&,
               Search& search,
               level_t const,
               match_view_t const&,
               cost_t const,
               direction const,
               std::optional<routing_time_t> const,
               bool,
               std::optional<way_idx_t> const,
               double const);

template <Profile P, typename Search>
path reconstruct(typename P::parameters const&,
                 ways const&,
                 lookup const&,
                 bitvec<node_idx_t> const*,
                 sharing_data const*,
                 elevation_storage const*,
                 Search const&,
                 location const&,
                 location const&,
                 way_idx_t const,
                 candidate_node const&,
                 candidate_node const&,
                 bool const,
                 way_idx_t const,
                 candidate_node const&,
                 bool const,
                 typename P::node const,
                 cost_t const,
                 direction const,
                 std::optional<routing_time_t> const);

template <search_profile profile>
struct search_state {
  using profile_t = profile_selector<profile>::type;

  search_state(profile_parameters const& params,
               ways const& w,
               lookup const& l,
               location const& from,
               std::vector<location> const& to,
               match_view_t const& from_match,
               match_result const& to_match,
               direction dir,
               cost_t const init_distance,
               cost_t const step_size,
               cost_t const max_distance,
               bitvec<node_idx_t> const* blocked,
               sharing_data const* sharing,
               elevation_storage const* elevations,
               std::optional<routing_time_t> start_time)
      : d_(get_dijkstra<profile_t>()),
        params_(std::get<profile_t::parameters>(params)),
        w_(w),
        l_(l),
        from_(from),
        from_match_(from_match),
        to_match_(to_match),
        dir_(dir),
        distance_(init_distance),
        step_size_(step_size),
        max_distance_(max_distance),
        blocked_(blocked),
        sharing_(sharing),
        elevations_(elevations),
        start_time_(start_time) {
    d_.reset(max_distance_);
  }

  bool run() {
    while (iteration_ != from_match_.size()) {
      if (!can_continue_ || !component_seen(w_, from_match_, iteration_)) {
        break;
      }

      if (utl::any_of(to_match_, [&](auto const dest) {
            return w_.r_->way_component_[from_match_.way_[iteration_]] ==
                   w_.r_->way_component_[dest.way_];
          })) {
        break;
      }

      iteration_++;
    }

    if (iteration_ != from_match_.size()) {
      auto const start_way = from_match_.way_[iteration_];
      auto const start_left = from_match_.left(iteration_);
      auto const start_right = from_match_.right(iteration_);
      for (auto const* nc : {&start_left, &start_right}) {
        if (nc->valid() && nc->cost_ < max_distance_) {
          // TODO: why is this needed?
          // What is the difference between start_cost and nc->cost_?
          auto const start_cost = get_endpoint_cost<profile_t>(
              params_, w_, start_way, *nc, false, flip(dir_, nc->way_dir_),
              start_time_, duration_t{0}, dir_);
          if (start_cost.cost_ == kInfeasible ||
              start_cost.cost_ >= max_distance_) {
            continue;
          }

          resolve_endpoint_node<profile_t>(
              *w_.r_, start_way, false, nc->node_, from_.lvl_, dir_,
              [&](auto const node) {
                auto label = typename profile_t::label{node, start_cost.cost_};
                label.track(label, *w_.r_, start_way, node.get_node(), false);
                d_.add_start(w_, label, start_cost.duration_);
              });
        }
      }
    } else {
      distance_ = std::min(distance_ + step_size_, max_distance_);
    }

    can_continue_ = !d_.run(params_, w_, *w_.r_, distance_, blocked_, sharing_,
                            elevations_, dir_) ||
                    can_continue_;

    return can_continue_ && (distance_ != max_distance_);
  }

  dijkstra<profile_t, false> d_;
  std::size_t iteration_ = {0};
  profile_t::parameters const& params_;
  ways const& w_;
  lookup const& l_;
  location const& from_;
  match_view_t const& from_match_;
  match_result const& to_match_;
  direction const dir_;
  cost_t const step_size_;
  cost_t const max_distance_;
  bitvec<node_idx_t> const* blocked_;
  sharing_data const* sharing_;
  elevation_storage const* elevations_;
  cost_t distance_;
  bool can_continue_ = true;
  std::optional<routing_time_t> const start_time_;
};

template <search_profile profile>
std::vector<std::optional<path>> get_results(
    search_state<profile> const& state,
    location const& from,
    std::vector<location> to,
    match_view_t const& from_match,
    match_result const& to_match,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

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
