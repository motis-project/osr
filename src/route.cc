#include "osr/routing/route.h"

#include <cstdint>

#include <cmath>
#include <algorithm>
#include <memory>
#include <optional>
#include <ranges>
#include <span>
#include <type_traits>
#include <utility>

#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/lookup.h"
#include "osr/routing/astar.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/cost_search_limit.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/path_reconstruction.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/hgv.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/with_profile.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

constexpr auto const kMinCostSettled = cost_t{900};

void verify_matching_penalty_factor(double const factor) {
  utl::verify(std::isfinite(factor) && factor >= 0.0,
              "matching penalty factor must be finite and nonnegative");
}

duration_t to_duration_limit(std::chrono::seconds const duration) {
  utl::verify(duration.count() >= 0 && duration.count() < kMaxDuration.count(),
              "duration limit must be between 0 and {} seconds",
              kMaxDuration.count() - 1U);
  return duration_t{static_cast<duration_t::rep>(duration.count())};
}

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
}

bool requires_dijkstra(search_profile const p) {
  switch (p) {
    case search_profile::kBikeSharing:
    case search_profile::kCarSharing:
    case search_profile::kCarParking:
    case search_profile::kCarParkingWheelchair:
    case search_profile::kCarDropOff:
    case search_profile::kCarDropOffWheelchair:
    case search_profile::kHgv: return true;
    default: return false;
  }
}

struct endpoint_candidate {
  way_idx_t way_;
  candidate_node node_;
  double graph_distance_;  // projected point -> node
  cost_t matching_penalty_;
};

template <Profile P>
struct endpoint_root {
  typename P::node node_;
  endpoint_candidate endpoint_;
  cost_and_duration connection_;
};

cost_t matching_penalty(double const distance,
                        double const closest_distance,
                        double const factor) {
  auto const additional_distance = std::max(0.0, distance - closest_distance);
  auto const penalty = std::round(distance + additional_distance * factor);
  return !(penalty < static_cast<double>(kInfeasible))
             ? kInfeasible
             : static_cast<cost_t>(penalty);
}

template <typename Fn>
void for_each_endpoint_candidate(match_view_t const& matches,
                                 double const penalty_factor,
                                 Fn&& fn) {
  for (auto i = std::size_t{0U}; i != matches.size(); ++i) {
    for (auto const& node : {matches.left(i), matches.right(i)}) {
      if (node.valid()) {
        auto const distance_to_way =
            static_cast<double>(matches.dist_to_way_[i]);
        fn(endpoint_candidate{.way_ = matches.way_[i],
                              .node_ = node,
                              .graph_distance_ = std::max(
                                  0.0, node.dist_to_node_ - distance_to_way),
                              .matching_penalty_ = matching_penalty(
                                  distance_to_way, matches.dist_to_way_.front(),
                                  penalty_factor)});
      }
    }
  }
}

template <Profile P>
cost_and_duration get_endpoint_connection(
    typename P::parameters const& params,
    ways const& w,
    endpoint_candidate const& endpoint,
    typename P::node const node,
    route_end const end,
    endpoint_role const role,
    std::optional<routing_time_t> const start_time,
    duration_t const current_duration,
    direction const search_dir) {
  auto const way_dir = flip(travel_dir_of(end), endpoint.node_.way_dir_);
  if (role == endpoint_role::kRoot &&
      !P::endpoint_root_allowed(params, node, way_dir)) {
    return infeasible_cost_and_duration();
  }
  auto const connection =
      P::endpoint_way_cost(params, *w.r_, w.timezones_, node, endpoint.way_,
                           w.r_->way_properties_[endpoint.way_], way_dir,
                           static_cast<distance_t>(endpoint.graph_distance_),
                           start_time, current_duration, search_dir);
  auto total =
      clamp_add(connection, endpoint.matching_penalty_, duration_t{0U});
  if (role == endpoint_role::kGoal) {
    total = clamp_add(
        total, P::endpoint_transition_cost(params, *w.r_, w.timezones_, node,
                                           endpoint.way_, way_dir, search_dir,
                                           start_time, current_duration));
  }
  return total;
}

path::segment make_endpoint_segment(lookup const& l,
                                    location const& location,
                                    endpoint_candidate const& endpoint,
                                    cost_and_duration const connection,
                                    node_idx_t const graph_node,
                                    mode const mode,
                                    endpoint_role const role,
                                    bool const reverse,
                                    direction const dir) {
  auto const graph_node_at_from = role == endpoint_role::kRoot
                                      ? dir == direction::kBackward
                                      : dir == direction::kForward;
  return {.polyline_ = l.get_node_candidate_path(
              endpoint.way_, endpoint.node_.node_, endpoint.node_.way_dir_,
              reverse, location),
          .from_level_ = endpoint.node_.lvl_,
          .to_level_ = endpoint.node_.lvl_,
          .from_ = graph_node_at_from ? graph_node : node_idx_t::invalid(),
          .to_ = graph_node_at_from ? node_idx_t::invalid() : graph_node,
          .way_ = way_idx_t::invalid(),
          .cost_ = connection.cost_,
          .duration_ = connection.duration_,
          .dist_ = static_cast<distance_t>(endpoint.graph_distance_),
          .mode_ = mode};
}

template <Profile P, typename CostMap>
endpoint_root<P> const& get_endpoint_root(
    CostMap const& costs,
    std::span<endpoint_root<P> const> const roots,
    typename P::node const node) {
  auto const& entry = costs.at(node.get_key());
  auto const cd = cost_and_duration{.cost_ = entry.cost(node),
                                    .duration_ = entry.duration(node)};
  auto const root = std::find_if(begin(roots), end(roots), [&](auto const& r) {
    return r.node_ == node && r.connection_ == cd;
  });
  utl::verify(root != end(roots), "no exact endpoint root for settled state");
  return *root;
}

template <Profile P>
path reconstruct_bi(typename P::parameters const& params,
                    ways const& w,
                    lookup const& l,
                    bitvec<node_idx_t> const* blocked,
                    sharing_data const* sharing,
                    elevation_storage const* elevations,
                    bidirectional<P> const& b,
                    location const& from,
                    location const& to,
                    std::span<endpoint_root<P> const> const starts,
                    std::span<endpoint_root<P> const> const destinations,
                    cost_t const cost,
                    duration_t const duration,
                    direction const dir) {
  auto forward_n = b.meet_point_1_;

  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;

  while (true) {
    auto const& e = b.cost1_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const pred_duration = b.cost1_.at(pred->get_key()).duration(*pred);
      auto const expected_cost = static_cast<cost_t>(
          e.cost(forward_n) - b.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      forward_n, pred_duration, {}, expected_cost,
                      clamp_sub_duration(e.duration(forward_n), pred_duration),
                      forward_segments, dir);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start = get_endpoint_root<P>(b.cost1_, starts, forward_n);
  forward_segments.push_back(make_endpoint_segment(
      l, from, start.endpoint_, start.connection_, forward_n.get_node(),
      forward_n.get_mode(), endpoint_role::kRoot, dir == direction::kBackward,
      dir));

  auto backward_segments = std::vector<path::segment>{};
  auto backward_n = b.meet_point_2_;
  auto backward_dist = 0.0;

  while (true) {
    auto const& e = b.cost2_.at(backward_n.get_key());
    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) -
                              b.template get_cost<direction::kBackward>(*pred));
      auto const curr_duration = e.duration(backward_n);
      auto const pred_duration = b.cost2_.at(pred->get_key()).duration(*pred);
      auto const expected_duration =
          clamp_sub_duration(curr_duration, pred_duration);
      backward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      backward_n, pred_duration, {}, expected_cost,
                      expected_duration, backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const& destination =
      get_endpoint_root<P>(b.cost2_, destinations, backward_n);
  backward_segments.push_back(make_endpoint_segment(
      l, to, destination.endpoint_, destination.connection_,
      backward_n.get_node(), backward_n.get_mode(), endpoint_role::kGoal,
      dir == direction::kForward, dir));

  // Neither search half includes the turn joining the two meeting states,
  // so we add it to the segment leaving the meeting node in travel direction.
  auto& outgoing = dir == direction::kForward ? backward_segments.front()
                                              : forward_segments.front();
  outgoing.cost_ = clamp_cost(static_cast<std::uint64_t>(outgoing.cost_) +
                              b.best_transition_.cost_);
  outgoing.duration_ =
      clamp_add_duration(outgoing.duration_, b.best_transition_.duration_);

  if (dir == direction::kForward) {
    std::reverse(forward_segments.begin(), forward_segments.end());
  } else {
    std::reverse(backward_segments.begin(), backward_segments.end());
    forward_segments.swap(backward_segments);
  }
  forward_segments.insert(forward_segments.end(), backward_segments.begin(),
                          backward_segments.end());

  auto total_dist = start.endpoint_.graph_distance_ + forward_dist +
                    backward_dist + destination.endpoint_.graph_distance_;

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : forward_segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .duration_ = duration,
                .dist_ = total_dist,
                .elevation_ = path_elevation,
                .segments_ = forward_segments};

  b.cost1_.at(b.meet_point_1_.get_key()).write(b.meet_point_1_, p);
  auto const uses_elevator = p.uses_elevator_;
  b.cost2_.at(b.meet_point_2_.get_key()).write(b.meet_point_2_, p);
  p.uses_elevator_ = p.uses_elevator_ || uses_elevator;
  return p;
}

template <Profile P, typename Search>
path reconstruct(typename P::parameters const& params,
                 ways const& w,
                 lookup const& l,
                 bitvec<node_idx_t> const* blocked,
                 sharing_data const* sharing,
                 elevation_storage const* elevations,
                 Search const& search,
                 location const& from,
                 location const& to,
                 std::span<endpoint_root<P> const> const starts,
                 endpoint_candidate const& destination,
                 cost_and_duration const destination_connection,
                 typename P::node const dest_node,
                 cost_and_duration const total,
                 direction const dir,
                 std::optional<routing_time_t> const start_time) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{make_endpoint_segment(
      l, to, destination, destination_connection, n.get_node(),
      dest_node.get_mode(), endpoint_role::kGoal, dir == direction::kForward,
      dir)};
  auto dist = 0.0;
  while (true) {
    auto const& e = search.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const pred_duration =
          search.cost_.at(pred->get_key()).duration(*pred);
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - search.get_cost(*pred));
      dist += add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                          n, pred_duration, start_time, expected_cost,
                          clamp_sub_duration(e.duration(n), pred_duration),
                          segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start = get_endpoint_root<P>(search.cost_, starts, n);
  segments.push_back(make_endpoint_segment(
      l, from, start.endpoint_, start.connection_, n.get_node(), n.get_mode(),
      endpoint_role::kRoot, dir == direction::kBackward, dir));
  if (dir == direction::kForward) {
    std::reverse(begin(segments), end(segments));
  }
  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = total.cost_,
                .duration_ = total.duration_,
                .dist_ = start.endpoint_.graph_distance_ + dist +
                         destination.graph_distance_,
                .elevation_ = path_elevation,
                .segments_ = segments};
  search.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
}

// Turns every match candidate into the search roots it can be entered from,
// with an initial cost covering the virtual edge from the query position to
// the graph node.
template <Profile P, typename AddFn>
std::vector<endpoint_root<P>> add_endpoint_roots(
    typename P::parameters const& params,
    ways const& w,
    match_view_t const& matches,
    route_end const end,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    cost_t const max,
    double const penalty_factor,
    bool const exact_return_allowed,
    AddFn&& add) {
  auto roots = std::vector<endpoint_root<P>>{};
  for_each_endpoint_candidate(
      matches, penalty_factor, [&](endpoint_candidate const& endpoint) {
        P::resolve_endpoint(
            *w.r_, endpoint.way_, endpoint.node_.node_, matches.lvl_, end,
            endpoint_role::kRoot, exact_return_allowed, [&](auto const node) {
              auto const connection = get_endpoint_connection<P>(
                  params, w, endpoint, node, end, endpoint_role::kRoot,
                  start_time, duration_t{0U}, dir);
              if (!connection.feasible() || connection.cost_ >= max) {
                return;
              }
              auto label = typename P::label{node, connection.cost_};
              label.track(label, *w.r_, endpoint.way_, node.get_node(), false);
              roots.emplace_back(endpoint_root<P>{node, endpoint, connection});
              add(std::move(label), connection.duration_);
            });
      });
  return roots;
}

template <Profile P>
struct destination_candidate {
  endpoint_candidate endpoint_;
  typename P::node node_;
  cost_and_duration connection_;
  cost_and_duration total_;
};

template <Profile P, typename Search>
std::optional<destination_candidate<P>> best_candidate(
    typename P::parameters const& params,
    ways const& w,
    Search const& search,
    match_view_t const& matches,
    cost_t const max,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    double const penalty_factor,
    bool const exact_return_allowed) {
  auto best = std::optional<destination_candidate<P>>{};
  auto const end = route_end_of(opposite(dir));
  for_each_endpoint_candidate(
      matches, penalty_factor, [&](endpoint_candidate const& endpoint) {
        auto const& candidate_node = endpoint.node_;
        auto const way_dir = flip(travel_dir_of(end), candidate_node.way_dir_);
        auto const consider = [&](auto const node) {
          auto const target_cost = search.get_cost(node);
          if (target_cost == kInfeasible) {
            return;
          }
          auto const target_duration =
              search.cost_.at(node.get_key()).duration(node);
          auto const reachable = P::is_dest_reachable(
              params, *w.r_, w.timezones_, node, endpoint.way_, way_dir, dir,
              start_time, target_duration);
          if (!reachable) {
            return;
          }
          auto const connection = get_endpoint_connection<P>(
              params, w, endpoint, node, end, endpoint_role::kGoal, start_time,
              target_duration, dir);
          if (!connection.feasible()) {
            return;
          }
          auto const total =
              clamp_add(cost_and_duration{.cost_ = target_cost,
                                          .duration_ = target_duration},
                        connection);
          if (!total.feasible()) {
            return;
          }
          if (total.cost_ >= max) {
            return;
          }
          if (!best.has_value() || total < best->total_) {
            best = destination_candidate<P>{endpoint, node, connection, total};
          }
        };
        P::resolve_endpoint(*w.r_, endpoint.way_, candidate_node.node_,
                            matches.lvl_, end, endpoint_role::kGoal,
                            exact_return_allowed, consider);
      });
  return best;
}

std::optional<path> try_direct(osr::location from,
                               osr::location to,
                               direction const dir) {
  auto const dist = geo::distance(from.pos_, to.pos_);
  if (dist >= 8.0) {
    return std::nullopt;
  }
  if (dir == direction::kBackward) {
    std::swap(from, to);
  }
  return std::optional{
      path{.cost_ = 60U,
           .duration_ = duration_from_cost(60U),
           .dist_ = dist,
           .segments_ = {path::segment{.polyline_ = {from.pos_, to.pos_},
                                       .from_level_ = from.lvl_,
                                       .to_level_ = to.lvl_,
                                       .from_ = node_idx_t::invalid(),
                                       .to_ = node_idx_t::invalid(),
                                       .way_ = way_idx_t::invalid(),
                                       .cost_ = 60U,
                                       .duration_ = duration_from_cost(60U),
                                       .dist_ = static_cast<distance_t>(dist)}},
           .uses_elevator_ = false}};
}

cost_t max_matching_penalty(match_view_t const& matches, double const factor) {
  auto max = cost_t{0U};
  for_each_endpoint_candidate(matches, factor,
                              [&](endpoint_candidate const& e) {
                                if (e.matching_penalty_ != kInfeasible) {
                                  max = std::max(max, e.matching_penalty_);
                                }
                              });
  return max;
}

cost_t endpoint_cost_limit(cost_t const profile_limit,
                           cost_t const from_penalty,
                           cost_t const to_penalty,
                           cost_t const max_penalty_slack) {
  auto const limit = static_cast<std::uint64_t>(profile_limit) +
                     std::min(from_penalty, max_penalty_slack) +
                     std::min(to_penalty, max_penalty_slack) + 1U;
  return static_cast<cost_t>(
      std::min<std::uint64_t>(limit, kMaxDurationSearchCost));
}

// Budget contribution of matching penalties per endpoint. Candidates with a
// higher penalty remain eligible; routes through them may exceed the budget.
template <typename P>
constexpr cost_t matching_penalty_budget_slack() {
  return std::is_same_v<P, car> || std::is_same_v<P, bus> ||
                 std::is_same_v<P, hgv>
             ? 300U
             : 600U;
}

// Applied only to the final choice: a route rejected here must not make the
// caller fall back to another search that could return a different route.
std::optional<path> within_duration_limit(std::optional<path> p,
                                          duration_t const max_duration) {
  return p.has_value() && p->duration_ <= max_duration ? std::move(p)
                                                       : std::nullopt;
}

template <Profile P>
std::optional<path> route_bidirectional(typename P::parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        bidirectional<P>& b,
                                        location const& from,
                                        location const& to,
                                        match_view_t const& from_match,
                                        match_view_t const& to_match,
                                        cost_t const max,
                                        direction const dir,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations,
                                        double const penalty_factor) {
  auto const search_max = std::max(kMinCostSettled, max);
  b.reset({.profile_ = params,
           .w_ = &w,
           .max_ = search_max,
           .dir_ = dir,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from,
           .end_loc_ = to});
  if (!b.search_bounds_valid_) {
    return std::nullopt;
  }

  auto const starts = add_endpoint_roots<P>(
      params, w, from_match, route_end_of(dir), dir, std::nullopt, max,
      penalty_factor, false, [&](auto&& label, duration_t const duration) {
        b.add_start(std::forward<decltype(label)>(label), duration);
      });
  auto const destinations = add_endpoint_roots<P>(
      params, w, to_match, route_end_of(opposite(dir)), dir, std::nullopt, max,
      penalty_factor, false, [&](auto&& label, duration_t const duration) {
        b.add_end(std::forward<decltype(label)>(label), duration);
      });
  if (starts.empty() || destinations.empty() || b.pq1_.empty() ||
      b.pq2_.empty()) {
    return std::nullopt;
  }

  b.run();
  if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
    return std::nullopt;
  }
  auto const cost = b.best_cost_;
  if (cost >= max) {
    return std::nullopt;
  }
  return reconstruct_bi<P>(params, w, l, blocked, sharing, elevations, b, from,
                           to, starts, destinations, cost, b.best_duration(),
                           dir);
}

template <Profile P>
std::optional<path> route_dijkstra(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P, false>& d,
    location const& from,
    location const& to,
    match_view_t const& from_match,
    match_view_t const& to_match,
    cost_t const max,
    duration_t const max_duration,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    route_options const& options) {
  auto const search_max = std::max(kMinCostSettled, max);
  d.reset({.profile_ = params,
           .w_ = &w,
           .max_ = search_max,
           .max_duration_ = max_duration,
           .dir_ = dir,
           .start_time_ = start_time,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from,
           .end_loc_ = to});
  auto const starts = add_endpoint_roots<P>(
      params, w, from_match, route_end_of(dir), dir, start_time, max,
      options.matching_penalty_factor_, options.exact_return_at_from_,
      [&](auto&& label, duration_t const duration) {
        d.add_start(std::forward<decltype(label)>(label), duration);
      });
  if (d.pq_.empty()) {
    return std::nullopt;
  }
  d.run();
  auto const find_candidate = [&]() {
    return best_candidate<P>(params, w, d, to_match, max, dir, start_time,
                             options.matching_penalty_factor_,
                             options.exact_return_at_to(0U));
  };
  auto candidate = find_candidate();
  // Settle through the complete cost, including the destination connector:
  // a cheaper route through an over-duration state may still beat this one.
  if (candidate.has_value() && candidate->total_.duration_ <= max_duration &&
      d.settle_up_to(candidate->total_.cost_)) {
    candidate = find_candidate();
  }
  if (!candidate.has_value() || candidate->total_.duration_ > max_duration) {
    return std::nullopt;
  }
  return reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from, to,
                        starts, candidate->endpoint_, candidate->connection_,
                        candidate->node_, candidate->total_, dir, start_time);
}

template <Profile P>
std::optional<path> route_astar(typename P::parameters const& params,
                                ways const& w,
                                lookup const& l,
                                astar<P, false>& a,
                                location const& from,
                                location const& to,
                                match_view_t const& from_match,
                                match_view_t const& to_match,
                                cost_t const max,
                                direction const dir,
                                std::optional<routing_time_t> const start_time,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations,
                                double const penalty_factor) {
  auto const search_max = std::max(kMinCostSettled, max);
  a.reset({.profile_ = params,
           .w_ = &w,
           .max_ = search_max,
           .dir_ = dir,
           .start_time_ = start_time,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from,
           .end_loc_ = to});
  for_each_endpoint_candidate(
      to_match, penalty_factor, [&](endpoint_candidate const& endpoint) {
        auto const& candidate_node = endpoint.node_;
        auto const add = [&](auto const node) { a.add_destination(node); };
        P::resolve_endpoint(*w.r_, endpoint.way_, candidate_node.node_,
                            to_match.lvl_, route_end_of(opposite(dir)),
                            endpoint_role::kGoal, false, add);
      });
  if (a.destinations_.empty()) {
    return std::nullopt;
  }
  auto const starts = add_endpoint_roots<P>(
      params, w, from_match, route_end_of(dir), dir, start_time, max,
      penalty_factor, false, [&](auto&& label, duration_t const duration) {
        a.add_start(std::forward<decltype(label)>(label), duration);
      });
  if (a.pq_.empty()) {
    return std::nullopt;
  }
  a.run();
  auto const candidate = best_candidate<P>(params, w, a, to_match, max, dir,
                                           start_time, penalty_factor, false);
  if (!candidate.has_value()) {
    return std::nullopt;
  }
  return reconstruct<P>(params, w, l, blocked, sharing, elevations, a, from, to,
                        starts, candidate->endpoint_, candidate->connection_,
                        candidate->node_, candidate->total_, dir, start_time);
}

// Keeps a finished one-to-many search alive so that paths to individual
// destinations can be reconstructed later on demand.
template <Profile P>
struct one_to_many_state_impl final : public one_to_many_state {
  explicit one_to_many_state_impl(std::vector<location> to)
      : to_{std::move(to)}, candidates_(to_.size()) {}

  std::vector<std::optional<path>> const& results() const override {
    return results_;
  }

  std::optional<path> reconstruct(ways const& w,
                                  lookup const& l,
                                  std::size_t const k,
                                  sharing_data const* sharing) override {
    if (k >= results_.size() || !results_[k].has_value()) {
      return std::nullopt;
    }
    if (k >= candidates_.size() || !candidates_[k].has_value()) {
      return results_[k];  // direct path (from ~ to), nothing to reconstruct
    }
    auto const& c = *candidates_[k];
    auto const& sp = d_.params_;
    return osr::reconstruct<P>(sp.profile_, w, l, sp.blocked_, sharing,
                               sp.elevations_, d_, sp.start_loc_, to_[k],
                               starts_, c.endpoint_, c.connection_, c.node_,
                               c.total_, sp.dir_, sp.start_time_);
  }

  // The search owns everything it ran with (profile parameters, blocked,
  // direction, start time, the start location, ...) - see `osr::search_params`.
  // `sharing_data` is the exception: it is only valid while the search runs,
  // so `reconstruct()` takes it from the caller.
  dijkstra<P> d_;
  std::vector<location> to_;
  std::vector<endpoint_root<P>> starts_;
  std::vector<std::optional<destination_candidate<P>>> candidates_;
  std::vector<std::optional<path>> results_;
};

template <Profile P>
std::vector<std::optional<path>> route(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P, false>& d,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    duration_t const max_duration,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    route_options const& options,
    one_to_many_state_impl<P>* const state = nullptr) {
  auto result = std::vector<std::optional<path>>(to_match.size());

  if (from_match.empty() || to_match.empty()) {
    return result;
  }

  auto const profile_limit = cost_search_limit(params, max_duration);
  auto const from_penalty =
      max_matching_penalty(from_match, options.matching_penalty_factor_);
  auto const destination_limits = utl::to_vec(
      std::views::iota(match_idx_t::value_t{0U},
                       static_cast<match_idx_t::value_t>(to_match.size())),
      [&](match_idx_t::value_t const i) {
        return endpoint_cost_limit(
            profile_limit, from_penalty,
            max_matching_penalty(to_match[match_idx_t{i}],
                                 options.matching_penalty_factor_),
            matching_penalty_budget_slack<P>());
      });
  auto const max = std::ranges::max(destination_limits);
  auto const search_max = std::max(kMinCostSettled, max);
  d.reset({.profile_ = params,
           .w_ = &w,
           .max_ = search_max,
           .max_duration_ = max_duration,
           .dir_ = dir,
           .start_time_ = start_time,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from});
  auto local_starts = std::vector<endpoint_root<P>>{};
  auto& starts = state == nullptr ? local_starts : state->starts_;
  starts = add_endpoint_roots<P>(
      params, w, from_match, route_end_of(dir), dir, start_time, max,
      options.matching_penalty_factor_, options.exact_return_at_from_,
      [&](auto&& label, duration_t const duration) {
        d.add_start(std::forward<decltype(label)>(label), duration);
      });
  d.run();
  auto const find_candidate = [&](std::size_t const i) {
    return best_candidate<P>(
        params, w, d,
        to_match[match_idx_t{static_cast<match_idx_t::value_t>(i)}],
        destination_limits[i], dir, start_time,
        options.matching_penalty_factor_, options.exact_return_at_to(i));
  };
  auto local_candidates =
      std::vector<std::optional<destination_candidate<P>>>{};
  auto& candidates = state == nullptr ? local_candidates : state->candidates_;
  candidates.resize(result.size());
  auto settle_cost = cost_t{0U};
  for (auto i = std::size_t{0U}; i != result.size(); ++i) {
    candidates[i] = find_candidate(i);
    if (candidates[i].has_value() &&
        candidates[i]->total_.duration_ <= max_duration) {
      settle_cost = std::max(settle_cost, candidates[i]->total_.cost_);
    }
  }
  // Finalize all candidates before reconstructing paths from the shared search.
  if (d.settle_up_to(settle_cost)) {
    for (auto i = std::size_t{0U}; i != result.size(); ++i) {
      candidates[i] = find_candidate(i);
    }
  }
  for (auto i = std::size_t{0U}; i != result.size(); ++i) {
    if (auto direct = try_direct(from, to[i], dir); direct.has_value()) {
      result[i] = within_duration_limit(std::move(direct), max_duration);
      candidates[i].reset();
      continue;
    }
    auto const& candidate = candidates[i];
    if (!candidate.has_value() || candidate->total_.duration_ > max_duration) {
      continue;
    }
    auto p = path{.cost_ = candidate->total_.cost_,
                  .duration_ = candidate->total_.duration_};
    d.cost_.at(candidate->node_.get_key()).write(candidate->node_, p);
    if (do_reconstruct(p)) {
      p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                         to[i], starts, candidate->endpoint_,
                         candidate->connection_, candidate->node_,
                         candidate->total_, dir, start_time);
    }
    result[i] = std::move(p);
  }
  return result;
}

std::optional<path> route_bidirectional(profile_parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        search_profile const profile,
                                        location const& from,
                                        location const& to,
                                        std::chrono::seconds const max_duration,
                                        direction const dir,
                                        double const max_match_distance,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations,
                                        route_options const& options) {
  utl::verify(!options.wants_exact_return(),
              "route_bidirectional does not support exact returns");
  return route(params, w, l, profile, from, to, max_duration, dir,
               max_match_distance, blocked, sharing, elevations,
               routing_algorithm::kAStarBi, std::nullopt, options);
}

std::vector<std::optional<path>> route(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    std::chrono::seconds const max_duration,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    std::optional<routing_time_t> const start_time,
    route_options const& options) {
  verify_matching_penalty_factor(options.matching_penalty_factor_);
  auto const duration_limit = to_duration_limit(max_duration);
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::vector<std::optional<path>> {
        auto const& pp = std::get<typename P::parameters>(params);
        auto from_m = match_result{};
        l.match<P>(pp, from, false, dir, max_match_distance, blocked,
                   options.exact_return_at_from_, from_m, start_time);
        auto const from_match = from_m[match_idx_t{0U}];
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto to_match = match_result{};
        for (auto i = std::size_t{0U}; i != to.size(); ++i) {
          l.match<P>(pp, to[i], true, dir, max_match_distance, blocked,
                     options.exact_return_at_to(i), to_match, start_time);
        }
        auto d = dijkstra<P>{};
        return route(pp, w, l, d, from, to, from_match, to_match,
                     duration_limit, dir, start_time, blocked, sharing,
                     elevations, do_reconstruct, options);
      });
}

std::optional<path> route_dijkstra(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    location const& to,
    std::chrono::seconds const max_duration,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::optional<routing_time_t> const start_time,
    route_options const& options) {
  return route(params, w, l, profile, from, to, max_duration, dir,
               max_match_distance, blocked, sharing, elevations,
               routing_algorithm::kDijkstra, start_time, options);
}

std::optional<path> route_astar(profile_parameters const& params,
                                ways const& w,
                                lookup const& l,
                                search_profile const profile,
                                location const& from,
                                location const& to,
                                std::chrono::seconds const max_duration,
                                direction const dir,
                                double const max_match_distance,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations,
                                std::optional<routing_time_t> const start_time,
                                route_options const& options) {
  verify_matching_penalty_factor(options.matching_penalty_factor_);
  auto const duration_limit = to_duration_limit(max_duration);
  utl::verify(!options.wants_exact_return(),
              "route_astar does not support exact returns");
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto from_m = match_result{};
    l.match<P>(pp, from, false, dir, max_match_distance, blocked, false, from_m,
               start_time);
    auto to_m = match_result{};
    l.match<P>(pp, to, true, dir, max_match_distance, blocked, false, to_m,
               start_time);
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    if (auto direct = try_direct(from, to, dir); direct.has_value()) {
      return within_duration_limit(std::move(direct), duration_limit);
    }
    auto const max = endpoint_cost_limit(
        cost_search_limit(params, duration_limit),
        max_matching_penalty(from_match, options.matching_penalty_factor_),
        max_matching_penalty(to_match, options.matching_penalty_factor_),
        matching_penalty_budget_slack<P>());
    auto a = astar<P>{};
    return within_duration_limit(
        route_astar(pp, w, l, a, from, to, from_match, to_match, max, dir,
                    start_time, blocked, sharing, elevations,
                    options.matching_penalty_factor_),
        duration_limit);
  });
}

std::unique_ptr<one_to_many_state> route_one_to_many(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    std::chrono::seconds const max_duration,
    direction const dir,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    std::optional<routing_time_t> const start_time,
    route_options const& options) {
  verify_matching_penalty_factor(options.matching_penalty_factor_);
  auto const duration_limit = to_duration_limit(max_duration);
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::unique_ptr<one_to_many_state> {
        auto s = std::make_unique<one_to_many_state_impl<P>>(to);
        if (from_match.empty()) {
          s->results_.resize(to.size());
          return s;
        }
        s->results_ = route(std::get<typename P::parameters>(params), w, l,
                            s->d_, from, s->to_, from_match, to_match,
                            duration_limit, dir, start_time, blocked, sharing,
                            elevations, do_reconstruct, options, s.get());
        return s;
      });
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t const& from_match,
                          match_view_t const& to_match,
                          std::chrono::seconds const max_duration,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo,
                          std::optional<routing_time_t> const start_time,
                          route_options const& options) {
  verify_matching_penalty_factor(options.matching_penalty_factor_);
  auto const duration_limit = to_duration_limit(max_duration);
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (requires_dijkstra(profile)) {
    algo = routing_algorithm::kDijkstra;
  }

  utl::verify(
      algo != routing_algorithm::kAStarBi || !options.wants_exact_return(),
      "bidirectional routing does not support exact returns");

  auto p = with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    if (auto direct = try_direct(from, to, dir); direct.has_value()) {
      return direct;
    }
    auto const max = endpoint_cost_limit(
        cost_search_limit(params, duration_limit),
        max_matching_penalty(from_match, options.matching_penalty_factor_),
        max_matching_penalty(to_match, options.matching_penalty_factor_),
        matching_penalty_budget_slack<P>());
    switch (algo) {
      case routing_algorithm::kAStarBi: {
        auto b = bidirectional<P>{};
        auto result = route_bidirectional(
            pp, w, l, b, from, to, from_match, to_match, max, dir, blocked,
            sharing, elevations, options.matching_penalty_factor_);
        if (result.has_value()) {
          return result;
        }
        [[fallthrough]];
      }
      case routing_algorithm::kDijkstra: {
        auto d = dijkstra<P>{};
        return route_dijkstra(pp, w, l, d, from, to, from_match, to_match, max,
                              duration_limit, dir, start_time, blocked, sharing,
                              elevations, options);
      }
    }
    throw utl::fail("not implemented");
  });
  return within_duration_limit(std::move(p), duration_limit);
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          std::chrono::seconds const max_duration,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo,
                          std::optional<routing_time_t> const start_time,
                          route_options const& options) {
  verify_matching_penalty_factor(options.matching_penalty_factor_);
  return with_profile(profile, [&]<Profile P>(P&&) {
    auto const& pp = std::get<typename P::parameters>(params);
    auto from_matches = match_result{};
    auto to_matches = match_result{};
    l.match<P>(pp, from, false, dir, max_match_distance, blocked,
               options.exact_return_at_from_, from_matches, start_time);
    l.match<P>(pp, to, true, dir, max_match_distance, blocked,
               options.exact_return_at_to(0U), to_matches, start_time);
    return route(params, w, l, profile, from, to, from_matches[match_idx_t{0U}],
                 to_matches[match_idx_t{0U}], max_duration, dir, blocked,
                 sharing, elevations, algo, start_time, options);
  });
}

}  // namespace osr
