#include "osr/routing/route.h"

#include <cstdint>

#include <algorithm>
#include <optional>
#include <utility>

#include "boost/thread/tss.hpp"

#include "utl/concat.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/lookup.h"
#include "osr/routing/astar.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/path_reconstruction.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/with_profile.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

constexpr auto const kMaxMatchingDistanceSquaredRatio = 9.0;
constexpr auto const kBottomKDefinitelyConsidered = 5;

template <Profile P>
bidirectional<P>& get_bidirectional() {
  static auto s = boost::thread_specific_ptr<bidirectional<P>>{};
  if (s.get() == nullptr) {
    s.reset(new bidirectional<P>{});
  }
  return *s.get();
}

template <Profile P>
dijkstra<P>& get_dijkstra() {
  static auto s = boost::thread_specific_ptr<dijkstra<P>>{};
  if (s.get() == nullptr) {
    s.reset(new dijkstra<P>{});
  }
  return *s.get();
}

template <Profile P>
astar<P>& get_astar() {
  static auto s = boost::thread_specific_ptr<astar<P>>{};
  if (s.get() == nullptr) {
    s.reset(new astar<P>{});
  }
  return *s.get();
}

duration_t sum_segment_durations(std::vector<path::segment> const& segments,
                                 duration_t total = duration_t{0}) {
  for (auto const& segment : segments) {
    total = clamp_add_duration(total, segment.duration_);
  }
  return total;
}

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
}

std::vector<geo::latlng> get_endpoint_path(lookup const& l,
                                           way_candidate const& candidate,
                                           node_candidate const& node,
                                           bool const reverse,
                                           location const& query) {
  auto path = l.get_node_candidate_path(candidate, node, reverse, query);
  if (candidate.exact_return_ && (path.empty() || path.back() != query.pos_)) {
    path.push_back(query.pos_);
  }
  return path;
}

template <Profile P>
cost_and_duration get_endpoint_cost(
    typename P::parameters const& params,
    ways const& w,
    way_candidate const& candidate,
    node_candidate const& node,
    direction const way_dir,
    std::optional<routing_time_t> const start_time,
    duration_t const current_duration,
    direction const search_dir) {
  if constexpr (SharingProfile<P>) {
    if (candidate.exact_return_) {
      return P::exact_return_way_cost(
          params, *w.r_, w.timezones_, candidate.way_,
          w.r_->way_properties_[candidate.way_], way_dir,
          static_cast<distance_t>(node.dist_to_node_), start_time,
          current_duration, search_dir);
    }
  }
  return P::way_cost(params, *w.r_, w.timezones_, candidate.way_,
                     w.r_->way_properties_[candidate.way_], way_dir,
                     static_cast<distance_t>(node.dist_to_node_), start_time,
                     current_duration, search_dir);
}

template <Profile P, typename Fn>
void resolve_endpoint_node(ways::routing const& w,
                           way_candidate const& candidate,
                           node_idx_t const node,
                           level_t const level,
                           direction const search_dir,
                           Fn&& fn) {
  if constexpr (SharingProfile<P>) {
    if (candidate.exact_return_) {
      P::resolve_exact_return_node(w, candidate.way_, node, level, search_dir,
                                   std::forward<Fn>(fn));
      return;
    }
  }
  P::resolve_start_node(w, candidate.way_, node, level, search_dir,
                        std::forward<Fn>(fn));
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
                    way_candidate const& start,
                    way_candidate const& dest,
                    cost_t const cost,
                    direction const dir) {
  auto forward_n = b.meet_point_1_;

  // TODO subtract meetpoint node cost?

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

  auto const& start_node_candidate =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;

  forward_segments.push_back(
      {.polyline_ =
           l.get_node_candidate_path(start, start_node_candidate, false, from),
       .from_level_ = start_node_candidate.lvl_,
       .to_level_ = start_node_candidate.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),

       .way_ = way_idx_t::invalid(),
       .cost_ = start_node_candidate.cost_,
       .duration_ = duration_from_cost(start_node_candidate.cost_),
       .dist_ = static_cast<distance_t>(start_node_candidate.dist_to_node_),
       .mode_ = forward_n.get_mode()});

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

  auto const& dest_node_candidate =
      backward_n.get_node() == dest.left_.node_ ? dest.left_ : dest.right_;

  backward_segments.push_back(
      {.polyline_ =
           l.get_node_candidate_path(dest, dest_node_candidate, true, to),
       .from_level_ = dest_node_candidate.lvl_,
       .to_level_ = dest_node_candidate.lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_node_candidate.cost_,
       .duration_ = duration_from_cost(dest_node_candidate.cost_),
       .dist_ = static_cast<distance_t>(dest_node_candidate.dist_to_node_),
       .mode_ = backward_n.get_mode()});

  if (dir == direction::kForward) {
    std::reverse(forward_segments.begin(), forward_segments.end());
  } else {
    std::reverse(backward_segments.begin(), backward_segments.end());
  }
  forward_segments.insert(forward_segments.end(), backward_segments.begin(),
                          backward_segments.end());

  auto total_dist = start_node_candidate.dist_to_node_ + forward_dist +
                    backward_dist + dest_node_candidate.dist_to_node_;

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : forward_segments) {
    path_elevation += segment.elevation_;
  }
  auto p =
      path{.cost_ = cost,
           .duration_ = sum_segment_durations(forward_segments, duration_t{0}),
           .dist_ = total_dist,
           .elevation_ = path_elevation,
           .segments_ = forward_segments};

  b.cost2_.at(backward_n.get_key()).write(backward_n, p);
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
                 way_candidate const& start,
                 way_candidate const& dest,
                 node_candidate const& dest_nc,
                 typename P::node const dest_node,
                 cost_t const cost,
                 direction const dir,
                 std::optional<routing_time_t> const start_time) {

  auto n = dest_node;
  auto const dest_node_duration = search.cost_.at(n.get_key()).duration(n);
  auto const dest_endpoint_cost = get_endpoint_cost<P>(
      params, w, dest, dest_nc, flip(opposite(dir), dest_nc.way_dir_),
      start_time, dest_node_duration, dir);
  auto segments = std::vector<path::segment>{
      {.polyline_ =
           get_endpoint_path(l, dest, dest_nc, dir == direction::kForward, to),
       .from_level_ = dest_nc.lvl_,
       .to_level_ = dest_nc.lvl_,
       .from_ =
           dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .to_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_endpoint_cost.cost_,
       .duration_ = dest_endpoint_cost.duration_,
       .dist_ = static_cast<distance_t>(dest_nc.dist_to_node_),
       .mode_ = dest_node.get_mode()}};
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

  auto const& start_nc =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  auto const start_endpoint_cost = get_endpoint_cost<P>(
      params, w, start, start_nc, flip(dir, start_nc.way_dir_), start_time,
      duration_t{0}, dir);
  segments.push_back(
      {.polyline_ = get_endpoint_path(l, start, start_nc,
                                      dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_endpoint_cost.cost_,
       .duration_ = start_endpoint_cost.duration_,
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = n.get_mode()});
  if (dir == direction::kForward) {
    std::reverse(begin(segments), end(segments));
  }
  if constexpr (SharingProfile<P>) {
    if (!segments.empty()) {
      auto const& last = segments.back();
      if (last.mode_ == mode::kFoot && last.way_ == way_idx_t::invalid() &&
          last.cost_ == cost_t{0U} && last.dist_ == distance_t{0U} &&
          last.to_ == node_idx_t::invalid()) {
        segments.pop_back();
      }
    }
  }
  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .duration_ = sum_segment_durations(segments),
                .dist_ = start_nc.dist_to_node_ + dist + dest_nc.dist_to_node_,
                .elevation_ = path_elevation,
                .segments_ = segments};
  search.cost_.at(dest_node.get_key()).write(dest_node, p);
  if (dest.exact_return_ || start.exact_return_) {
    p.track_node_ = node_idx_t::invalid();
  }
  return p;
}

bool component_seen(ways const& w,
                    match_view_t matches,
                    size_t match_idx,
                    unsigned times = 1) {
  auto this_component = w.r_->way_component_[matches[match_idx].way_];
  for (auto j = 0U; j < match_idx; ++j) {
    if (w.r_->way_component_[matches[j].way_] == this_component) {
      if (--times == 0) {
        return true;
      }
    }
  }
  return false;
}

template <SharingProfile P>
struct source_seed {
  typename P::node node_;
  way_candidate const* match_;
  cost_t cost_;
  duration_t duration_;
};

template <SharingProfile P, typename IsRelevant>
std::vector<source_seed<P>> get_source_seeds(
    typename P::parameters const& params,
    ways const& w,
    level_t const lvl,
    match_view_t const matches,
    cost_t const max,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    IsRelevant&& is_relevant) {
  auto seeds = std::vector<source_seed<P>>{};
  for (auto const& match : matches) {
    if (!is_relevant(match)) {
      continue;
    }
    auto const component = w.r_->way_component_[match.way_];
    auto closest_in_component = std::numeric_limits<double>::max();
    for (auto const& candidate : matches) {
      if (is_relevant(candidate) &&
          w.r_->way_component_[candidate.way_] == component) {
        closest_in_component =
            std::min(closest_in_component, candidate.dist_to_way_);
      }
    }
    if (match.dist_to_way_ > closest_in_component + P::kMatchTolerance) {
      continue;
    }

    for (auto const* nc : {&match.left_, &match.right_}) {
      if (!nc->valid() || nc->cost_ >= max) {
        continue;
      }
      auto const start_cost =
          get_endpoint_cost<P>(params, w, match, *nc, flip(dir, nc->way_dir_),
                               start_time, duration_t{0}, dir);
      if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
        continue;
      }
      resolve_endpoint_node<P>(
          *w.r_, match, nc->node_, lvl, dir, [&](auto const node) {
            auto const existing = utl::find_if(
                seeds,
                [&](source_seed<P> const& seed) { return seed.node_ == node; });
            auto const candidate = source_seed<P>{
                node, &match, start_cost.cost_, start_cost.duration_};
            if (existing == end(seeds)) {
              seeds.emplace_back(candidate);
            } else if (candidate.cost_ < existing->cost_ ||
                       (candidate.cost_ == existing->cost_ &&
                        candidate.match_->dist_to_way_ <
                            existing->match_->dist_to_way_)) {
              *existing = candidate;
            }
          });
    }
  }
  return seeds;
}

template <SharingProfile P>
void add_source_seeds(ways const& w,
                      dijkstra<P>& d,
                      std::span<source_seed<P> const> const seeds) {
  for (auto const& seed : seeds) {
    auto label = typename P::label{seed.node_, seed.cost_};
    label.track(label, *w.r_, seed.match_->way_, seed.node_.get_node(), false);
    d.add_start(w, label, seed.duration_);
  }
}

template <SharingProfile P, typename Search>
source_seed<P> const& get_source_seed(
    Search const& search,
    std::span<source_seed<P> const> const seeds,
    typename P::node node) {
  while (true) {
    auto const pred = search.cost_.at(node.get_key()).pred(node);
    if (!pred.has_value()) {
      break;
    }
    node = *pred;
  }
  auto const cost = search.get_cost(node);
  auto const it = utl::find_if(seeds, [&](source_seed<P> const& seed) {
    return seed.node_ == node && seed.cost_ == cost;
  });
  utl::verify(it != end(seeds), "no source match for routing root");
  return *it;
}

template <Profile P, typename Search>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename P::node,
                         path>>
best_candidate(typename P::parameters const& params,
               ways const& w,
               Search& search,
               level_t const lvl,
               match_view_t m,
               cost_t const max,
               direction const dir,
               std::optional<routing_time_t> const start_time,
               bool should_continue,
               way_candidate const* start,
               double const limit_squared_max_matching_distance) {
  using result_t = std::tuple<node_candidate const*, way_candidate const*,
                              typename P::node, path>;

  auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max(),
                        .duration_ = kMaxDuration};
  auto best_node = P::node::invalid();
  auto best = static_cast<node_candidate const*>(nullptr);

  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto const consider = [&](auto&& node) {
      auto const target_cost = search.get_cost(node);
      if (target_cost == kInfeasible || target_cost > best_cost.cost_) {
        return;
      }

      auto const target_duration =
          search.cost_.at(node.get_key()).duration(node);
      auto const reachable = [&]() {
        if constexpr (SharingProfile<P>) {
          if (dest.exact_return_) {
            return P::is_exact_return_reachable(
                params, *w.r_, w.timezones_, node, dest.way_,
                flip(opposite(dir), x->way_dir_), dir, start_time,
                target_duration);
          }
        }
        return P::is_dest_reachable(params, *w.r_, w.timezones_, node,
                                    dest.way_, flip(opposite(dir), x->way_dir_),
                                    dir, start_time, target_duration);
      }();
      if (!reachable) {
        return;
      }

      auto const dest_way_cost = get_endpoint_cost<P>(
          params, w, dest, *x, flip(opposite(dir), x->way_dir_), start_time,
          target_duration, dir);
      if (dest_way_cost.cost_ == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + dest_way_cost.cost_;
      auto const total_duration =
          clamp_add_duration(target_duration, dest_way_cost.duration_);
      if (total_cost < best_cost.cost_ ||
          (total_cost == best_cost.cost_ &&
           total_duration < best_cost.duration_)) {
        best_node = node;
        best = x;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
        best_cost.duration_ = total_duration;
      }
    };
    if constexpr (SharingProfile<P>) {
      if (dest.exact_return_) {
        P::resolve_exact_return_node(*w.r_, dest.way_, x->node_, lvl,
                                     opposite(dir), consider);
      } else {
        P::resolve_all(*w.r_, x->node_, lvl, consider);
      }
    } else {
      P::resolve_all(*w.r_, x->node_, lvl, consider);
    }
  };

  auto selected = std::optional<result_t>{};
  auto closest_reachable_match = std::optional<double>{};
  auto const start_component =
      start != nullptr
          ? std::make_optional(w.r_->way_component_[start->way_])
          : std::nullopt;
  auto component_seen_ctr = 0;
  for (auto const [j, dest] : utl::enumerate(m)) {
    if constexpr (SharingProfile<P>) {
      if (closest_reachable_match.has_value() &&
          dest.dist_to_way_ > *closest_reachable_match + P::kMatchTolerance) {
        break;
      }
    }
    if (start_component.has_value() &&
        *start_component != w.r_->way_component_[dest.way_]) {
      continue;
    }
    if (!should_continue && ++component_seen_ctr > 10) {
      break;
    }
    if (std::pow(dest.dist_to_way_, 2) > limit_squared_max_matching_distance &&
        j > kBottomKDefinitelyConsidered) {
      break;
    }
    best_node = P::node::invalid();
    best_cost = path{.cost_ = std::numeric_limits<cost_t>::max(),
                     .duration_ = kMaxDuration};
    best = nullptr;

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid()) {
        get_best(dest, x);
      }
    }

    if (best != nullptr) {
      if constexpr (SharingProfile<P>) {
        if (!closest_reachable_match.has_value()) {
          closest_reachable_match = dest.dist_to_way_;
        }
        auto const is_better =
            !selected.has_value() ||
            best_cost.cost_ < std::get<3>(*selected).cost_ ||
            (best_cost.cost_ == std::get<3>(*selected).cost_ &&
             (best_cost.duration_ < std::get<3>(*selected).duration_ ||
              (best_cost.duration_ == std::get<3>(*selected).duration_ &&
               dest.dist_to_way_ < std::get<1>(*selected)->dist_to_way_)));
        if (is_better) {
          selected = result_t{best, &dest, best_node, best_cost};
        }
      } else {
        return best_cost.cost_ < max
                   ? std::optional{result_t{best, &dest, best_node, best_cost}}
                   : std::nullopt;
      }
    }
  }
  return selected.has_value() && std::get<3>(*selected).cost_ < max
             ? selected
             : std::nullopt;
}

std::optional<path> try_direct(osr::location const& from,
                               osr::location const& to) {
  auto const dist = geo::distance(from.pos_, to.pos_);
  if (dist < 8.0) {
    return std::optional{path{
        .cost_ = 60U,
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
  } else {
    return std::nullopt;
  }
}

template <Profile P>
std::optional<path> route_bidirectional(typename P::parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        bidirectional<P>& b,
                                        location const& from,
                                        location const& to,
                                        match_view_t from_match,
                                        match_view_t to_match,
                                        cost_t const max,
                                        direction const dir,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  b.reset(params, max, from, to);
  if (b.radius_ == max) {
    return std::nullopt;
  }

  auto const limit_squared_max_matching_distance =
      geo::approx_squared_distance(from.pos_, to.pos_,
                                   b.distance_lon_degrees_) /
      kMaxMatchingDistanceSquaredRatio;

  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (b.max_reached_1_ && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = start.way_;
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        auto const start_cost =
            get_endpoint_cost<P>(params, w, start, *nc, flip(dir, nc->way_dir_),
                                 {}, duration_t{0}, dir);
        if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
          continue;
        }
        resolve_endpoint_node<P>(
            *w.r_, start, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, start_cost.cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              b.add_start(params, w, label, sharing, start_cost.duration_);
            });
      }
    }
    if (b.pq1_.empty()) {
      continue;
    }
    for (auto const [j, end] : utl::enumerate(to_match)) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }
      if (b.max_reached_2_ && component_seen(w, to_match, j)) {
        continue;
      }
      if (std::pow(end.dist_to_way_, 2) > limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }
      auto const end_way = end.way_;
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          auto const end_cost = get_endpoint_cost<P>(
              params, w, end, *nc, flip(opposite(dir), nc->way_dir_), {},
              duration_t{0}, dir);
          resolve_endpoint_node<P>(
              *w.r_, end, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) {
                auto label = typename P::label{node, end_cost.cost_};
                label.track(label, *w.r_, end_way, node.get_node(), false);
                b.add_end(params, w, label, sharing);
              });
        }
      }
      if (b.pq2_.empty()) {
        continue;
      }
      auto const should_continue =
          b.run(params, w, *w.r_, max, blocked, sharing, elevations, dir);

      if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
        if (should_continue) {
          continue;
        }
        return std::nullopt;
      }

      auto const cost = b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);

      return reconstruct_bi(params, w, l, blocked, sharing, elevations, b, from,
                            to, start, end, cost, dir);
    }
    b.pq1_.clear();
    b.pq2_.clear();
    b.cost2_.clear();
    b.max_reached_2_ = false;
  }
  return std::nullopt;
}

template <Profile P>
std::optional<path> route_dijkstra(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P>& d,
    location const& from,
    location const& to,
    match_view_t from_match,
    match_view_t to_match,
    cost_t const max,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  auto const limit_squared_max_matching_distance =
      std::pow(geo::distance(from.pos_, to.pos_), 2) /
      kMaxMatchingDistanceSquaredRatio;

  if constexpr (SharingProfile<P>) {
    auto const is_relevant = [&](way_candidate const& start) {
      return utl::any_of(to_match, [&](way_candidate const& end) {
        return w.r_->way_component_[start.way_] ==
               w.r_->way_component_[end.way_];
      });
    };
    auto const seeds = get_source_seeds<P>(params, w, from.lvl_, from_match,
                                           max, dir, start_time, is_relevant);
    if (seeds.empty()) {
      return std::nullopt;
    }
    d.reset(max);
    add_source_seeds<P>(w, d, seeds);
    auto const search_complete = d.run(params, w, *w.r_, max, start_time,
                                       blocked, sharing, elevations, dir);
    auto const c = best_candidate<P>(params, w, d, to.lvl_, to_match, max, dir,
                                     start_time, search_complete, nullptr,
                                     limit_squared_max_matching_distance);
    if (!c.has_value()) {
      return std::nullopt;
    }
    auto const [nc, wc, node, p] = *c;
    auto const& seed = get_source_seed<P>(d, seeds, node);
    return reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                          to, *seed.match_, *wc, *nc, node, p.cost_, dir,
                          start_time);
  } else {
    d.reset(max);
    auto should_continue = true;
    for (auto const [i, start] : utl::enumerate(from_match)) {
      if (!should_continue && component_seen(w, from_match, i)) {
        continue;
      }
      if (utl::none_of(to_match, [&](way_candidate const& end) {
            return w.r_->way_component_[start.way_] ==
                   w.r_->way_component_[end.way_];
          })) {
        continue;
      }

      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          auto const start_cost = get_endpoint_cost<P>(
              params, w, start, *nc, flip(dir, nc->way_dir_), start_time,
              duration_t{0}, dir);
          if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
            continue;
          }
          resolve_endpoint_node<P>(
              *w.r_, start, nc->node_, from.lvl_, dir, [&](auto const node) {
                d.add_start(w, {node, start_cost.cost_}, start_cost.duration_);
              });
        }
      }

      if (d.pq_.empty()) {
        continue;
      }

      should_continue = d.run(params, w, *w.r_, max, start_time, blocked,
                              sharing, elevations, dir) &&
                        should_continue;

      auto const c = best_candidate<P>(params, w, d, to.lvl_, to_match, max,
                                       dir, start_time, should_continue, &start,
                                       limit_squared_max_matching_distance);
      if (c.has_value()) {
        auto const [nc, wc, node, p] = *c;
        return reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                              from, to, start, *wc, *nc, node, p.cost_, dir,
                              start_time);
      }
    }
    return std::nullopt;
  }
}

template <Profile P>
std::optional<path> route_astar(typename P::parameters const& params,
                                ways const& w,
                                lookup const& l,
                                astar<P>& a,
                                location const& from,
                                location const& to,
                                match_view_t from_match,
                                match_view_t to_match,
                                cost_t const max,
                                direction const dir,
                                std::optional<routing_time_t> const start_time,
                                bitvec<node_idx_t> const* blocked,
                                sharing_data const* sharing,
                                elevation_storage const* elevations) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  auto const limit_squared_max_matching_distance =
      std::pow(geo::distance(from.pos_, to.pos_), 2) /
      kMaxMatchingDistanceSquaredRatio;

  a.reset(max, from, to);
  auto should_continue = true;
  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    if (utl::none_of(to_match, [&](way_candidate const& end) {
          return w.r_->way_component_[start.way_] ==
                 w.r_->way_component_[end.way_];
        })) {
      continue;
    }

    a.reset(max, from, to);
    for (auto const [j, end] : utl::enumerate(to_match)) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }
      if (!should_continue && component_seen(w, to_match, j, 10)) {
        continue;
      }
      if (std::pow(end.dist_to_way_, 2) > limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }

      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_all(*w.r_, nc->node_, to.lvl_, [&](auto const node) {
            if (!P::is_dest_reachable(params, *w.r_, w.timezones_, node,
                                      end.way_,
                                      flip(opposite(dir), nc->way_dir_), dir,
                                      start_time, duration_t{0})) {
              return;
            }
            a.add_destination(params, w, sharing, node);
          });
        }
      }
    }

    if (a.destinations_.empty()) {
      continue;
    }

    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        auto const start_cost =
            get_endpoint_cost<P>(params, w, start, *nc, flip(dir, nc->way_dir_),
                                 start_time, duration_t{0}, dir);
        if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
          continue;
        }
        resolve_endpoint_node<P>(
            *w.r_, start, nc->node_, from.lvl_, dir, [&](auto const node) {
              a.add_start(params, w, sharing,
                          typename P::label{node, start_cost.cost_},
                          start_cost.duration_);
            });
      }
    }

    if (a.pq_.empty()) {
      continue;
    }

    should_continue = a.run(params, w, *w.r_, max, start_time, blocked, sharing,
                            elevations, dir) &&
                      should_continue;

    auto const c = best_candidate<P>(params, w, a, to.lvl_, to_match, max, dir,
                                     start_time, should_continue, &start,
                                     limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, a, from,
                            to, start, *wc, *nc, node, p.cost_, dir,
                            start_time);
    }
  }

  return std::nullopt;
}

// 1:n
template <Profile P>
std::vector<std::optional<path>> route(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P>& d,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const dir,
    std::optional<routing_time_t> const start_time,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct) {
  auto result = std::vector<std::optional<path>>{};
  result.resize(to_match.size());

  if (from_match.empty()) {
    return result;
  }

  auto const distance_lng_degrees = geo::approx_distance_lng_degrees(from.pos_);

  if constexpr (SharingProfile<P>) {
    auto direct = std::vector<bool>(to_match.size(), false);
    for (auto i = 0U; i != to.size(); ++i) {
      if (auto const p = try_direct(from, to[i]); p.has_value()) {
        result[i] = p;
        direct[i] = true;
      }
    }
    auto const is_relevant = [&](way_candidate const& start) {
      for (auto j = 0U; j != to_match.size(); ++j) {
        if (!direct[j] &&
            utl::any_of(to_match[j], [&](way_candidate const& end) {
              return w.r_->way_component_[start.way_] ==
                     w.r_->way_component_[end.way_];
            })) {
          return true;
        }
      }
      return false;
    };
    auto const seeds = get_source_seeds<P>(params, w, from.lvl_, from_match,
                                           max, dir, start_time, is_relevant);
    if (seeds.empty()) {
      return result;
    }
    d.reset(max);
    add_source_seeds<P>(w, d, seeds);
    auto const search_complete = d.run(params, w, *w.r_, max, start_time,
                                       blocked, sharing, elevations, dir);

    for (auto j = 0U; j != result.size(); ++j) {
      if (direct[j]) {
        continue;
      }
      auto const limit_squared_max_matching_distance =
          geo::approx_squared_distance(from.pos_, to[j].pos_,
                                       distance_lng_degrees) /
          kMaxMatchingDistanceSquaredRatio;
      auto const c = best_candidate<P>(
          params, w, d, to[j].lvl_, to_match[j], max, dir, start_time,
          search_complete, nullptr, limit_squared_max_matching_distance);
      if (!c.has_value()) {
        continue;
      }
      auto [nc, wc, n, p] = *c;
      d.cost_.at(n.get_key()).write(n, p);
      if (do_reconstruct(p)) {
        auto const& seed = get_source_seed<P>(d, seeds, n);
        p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                           to[j], *seed.match_, *wc, *nc, n, p.cost_, dir,
                           start_time);
        p.uses_elevator_ = true;
      }
      result[j] = std::move(p);
    }
    return result;
  } else {
    d.reset(max);
    auto should_continue = true;
    for (auto const [i, start] : utl::enumerate(from_match)) {
      if (!should_continue && component_seen(w, from_match, i)) {
        continue;
      }
      auto const start_way = start.way_;
      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          auto const start_cost = get_endpoint_cost<P>(
              params, w, start, *nc, flip(dir, nc->way_dir_), start_time,
              duration_t{0}, dir);
          if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
            continue;
          }
          resolve_endpoint_node<P>(
              *w.r_, start, nc->node_, from.lvl_, dir, [&](auto const node) {
                auto label = typename P::label{node, start_cost.cost_};
                label.track(label, *w.r_, start_way, node.get_node(), false);
                d.add_start(w, label, start_cost.duration_);
              });
        }
      }

      should_continue = d.run(params, w, *w.r_, max, start_time, blocked,
                              sharing, elevations, dir) &&
                        should_continue;

      auto found = 0U;
      for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
        if (r.has_value()) {
          ++found;
        } else if (auto const direct = try_direct(from, t);
                   direct.has_value()) {
          r = direct;
          ++found;
        } else {
          auto const limit_squared_max_matching_distance =
              geo::approx_squared_distance(from.pos_, t.pos_,
                                           distance_lng_degrees) /
              kMaxMatchingDistanceSquaredRatio;

          auto const c = best_candidate<P>(
              params, w, d, t.lvl_, m, max, dir, start_time, should_continue,
              &start, limit_squared_max_matching_distance);
          if (c.has_value()) {
            auto [nc, wc, n, p] = *c;
            d.cost_.at(n.get_key()).write(n, p);
            if (do_reconstruct(p)) {
              p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                                 from, t, start, *wc, *nc, n, p.cost_, dir,
                                 start_time);
              p.uses_elevator_ = true;
            }
            r = std::move(p);
            ++found;
          }
        }
      }

      if (found == result.size()) {
        return result;
      }
    }
    return result;
  }
}

std::optional<path> route_bidirectional(profile_parameters const& params,
                                        ways const& w,
                                        lookup const& l,
                                        search_profile const profile,
                                        location const& from,
                                        location const& to,
                                        cost_t const max,
                                        direction const dir,
                                        double const max_match_distance,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_bidirectional(pp, w, l, get_bidirectional<P>(), from, to,
                               from_match, to_match, max, dir, blocked, sharing,
                               elevations);
  });
}

std::vector<std::optional<path>> route(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    std::optional<routing_time_t> const start_time,
    route_endpoint_options const& endpoint_options) {
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::vector<std::optional<path>> {
        auto const& pp = std::get<typename P::parameters>(params);
        auto const from_match = l.match_endpoint<P>(
            pp, from, false, dir, max_match_distance, blocked,
            endpoint_options.exact_return_at_from_, start_time);
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto to_match = std::vector<match_t>{};
        to_match.reserve(to.size());
        for (auto const [i, x] : utl::enumerate(to)) {
          to_match.emplace_back(l.match_endpoint<P>(
              pp, x, true, dir, max_match_distance, blocked,
              endpoint_options.exact_return_at_to(i), start_time));
        }
        return route(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                     to_match, max, dir, start_time, blocked, sharing,
                     elevations, do_reconstruct);
      });
}

std::optional<path> route_dijkstra(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    location const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::optional<routing_time_t> const start_time) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const from_match = l.match<P>(pp, from, false, dir, max_match_distance,
                                       blocked, start_time);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked, start_time);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_dijkstra(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                          to_match, max, dir, start_time, blocked, sharing,
                          elevations);
  });
}

std::optional<path> route_astar(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    location const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::optional<routing_time_t> const start_time) {
  return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
    auto const& pp = std::get<typename P::parameters>(params);
    auto const from_match = l.match<P>(pp, from, false, dir, max_match_distance,
                                       blocked, start_time);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked, start_time);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_astar(pp, w, l, get_astar<P>(), from, to, from_match, to_match,
                       max, dir, start_time, blocked, sharing, elevations);
  });
}

std::vector<std::optional<path>> route(
    profile_parameters const& params,
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const dir,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    std::optional<routing_time_t> const start_time) {
  if (from_match.empty()) {
    return std::vector<std::optional<path>>(to.size());
  }
  return with_profile(profile, [&]<Profile P>(P&&) {
    return route(std::get<typename P::parameters>(params), w, l,
                 get_dijkstra<P>(), from, to, from_match, to_match, max, dir,
                 start_time, blocked, sharing, elevations, do_reconstruct);
  });
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo,
                          std::optional<routing_time_t> const start_time) {
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing ||
      profile == search_profile::kHgv) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }

  switch (algo) {
    case routing_algorithm::kDijkstra:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_dijkstra(std::get<typename P::parameters>(params), w, l,
                              get_dijkstra<P>(), from, to, from_match, to_match,
                              max, dir, start_time, blocked, sharing,
                              elevations);
      });
    case routing_algorithm::kAStarBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        auto const& pp = std::get<typename P::parameters>(params);
        if constexpr (requires { P::kExactBidirectional; }) {
          if constexpr (!P::kExactBidirectional) {
            return route_astar(pp, w, l, get_astar<P>(), from, to, from_match,
                               to_match, max, dir, start_time, blocked, sharing,
                               elevations);
          }
        }
        auto result = route_bidirectional(pp, w, l, get_bidirectional<P>(),
                                          from, to, from_match, to_match, max,
                                          dir, blocked, sharing, elevations);
        if constexpr (requires(typename P::node const n) {
                        P::bidirectional_meet_cost(pp, *w.r_, n, n);
                      }) {
          if (!result.has_value()) {
            return route_dijkstra(pp, w, l, get_dijkstra<P>(), from, to,
                                  from_match, to_match, max, dir, start_time,
                                  blocked, sharing, elevations);
          }
        }
        return result;
      });
  }
  throw utl::fail("not implemented");
}

std::optional<path> route(profile_parameters const& params,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations,
                          routing_algorithm algo,
                          std::optional<routing_time_t> const start_time,
                          route_endpoint_options const& endpoint_options) {
  if ((endpoint_options.exact_return_at_from_ ||
       endpoint_options.exact_return_at_to(0U)) &&
      (profile == search_profile::kBikeSharing ||
       profile == search_profile::kCarSharing)) {
    return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
      auto const& pp = std::get<typename P::parameters>(params);
      auto const from_match = l.match_endpoint<P>(
          pp, from, false, dir, max_match_distance, blocked,
          endpoint_options.exact_return_at_from_, start_time);
      auto const to_match = l.match_endpoint<P>(
          pp, to, true, dir, max_match_distance, blocked,
          endpoint_options.exact_return_at_to(0U), start_time);
      if (from_match.empty() || to_match.empty()) {
        return std::nullopt;
      }
      return route_dijkstra(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                            to_match, max, dir, start_time, blocked, sharing,
                            elevations);
    });
  }
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing ||
      profile == search_profile::kCarParkingWheelchair ||
      profile == search_profile::kCarParking ||
      profile == search_profile::kHgv) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(params, w, l, profile, from, to, max, dir,
                            max_match_distance, blocked, sharing, elevations,
                            start_time);
    case routing_algorithm::kAStarBi:
      return route_bidirectional(params, w, l, profile, from, to, max, dir,
                                 max_match_distance, blocked, sharing,
                                 elevations);
  }
  throw utl::fail("not implemented");
}

}  // namespace osr
