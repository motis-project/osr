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
                                           way_idx_t const way,
                                           candidate_node const& node,
                                           bool const exact_return,
                                           bool const reverse,
                                           location const& query) {
  auto path =
      l.get_node_candidate_path(way, node.node_, node.way_dir_, reverse, query);
  if (exact_return && (path.empty() || path.back() != query.pos_)) {
    path.push_back(query.pos_);
  }
  return path;
}

template <Profile P>
cost_and_duration get_endpoint_cost(
    typename P::parameters const& params,
    ways const& w,
    way_idx_t const way,
    candidate_node const& node,
    bool const exact_return,
    direction const way_dir,
    std::optional<routing_time_t> const start_time,
    duration_t const current_duration,
    direction const search_dir) {
  if constexpr (SharingProfile<P>) {
    if (exact_return) {
      return P::exact_return_way_cost(
          params, *w.r_, w.timezones_, way, w.r_->way_properties_[way], way_dir,
          static_cast<distance_t>(node.dist_to_node_), start_time,
          current_duration, search_dir);
    }
  }
  return P::way_cost(params, *w.r_, w.timezones_, way,
                     w.r_->way_properties_[way], way_dir,
                     static_cast<distance_t>(node.dist_to_node_), start_time,
                     current_duration, search_dir);
}

template <Profile P, typename Fn>
void resolve_endpoint_node(ways::routing const& w,
                           way_idx_t const way,
                           bool const exact_return,
                           node_idx_t const node,
                           level_t const level,
                           direction const search_dir,
                           Fn&& fn) {
  if constexpr (SharingProfile<P>) {
    if (exact_return) {
      P::resolve_exact_return_node(w, way, node, level, search_dir,
                                   std::forward<Fn>(fn));
      return;
    }
  }
  P::resolve_start_node(w, way, node, level, search_dir, std::forward<Fn>(fn));
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
                    way_idx_t const start_way,
                    candidate_node const& start_left,
                    candidate_node const& start_right,
                    way_idx_t const dest_way,
                    candidate_node const& dest_left,
                    candidate_node const& dest_right,
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
      forward_n.get_node() == start_left.node_ ? start_left : start_right;

  forward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start_way, start_node_candidate.node_, start_node_candidate.way_dir_,
           false, from),
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
      backward_n.get_node() == dest_left.node_ ? dest_left : dest_right;

  backward_segments.push_back(
      {.polyline_ =
           l.get_node_candidate_path(dest_way, dest_node_candidate.node_,
                                     dest_node_candidate.way_dir_, true, to),
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
                 way_idx_t const start_way,
                 candidate_node const& start_left,
                 candidate_node const& start_right,
                 bool const start_exact_return,
                 way_idx_t const dest_way,
                 candidate_node const& dest_nc,
                 bool const dest_exact_return,
                 typename P::node const dest_node,
                 cost_t const cost,
                 direction const dir,
                 std::optional<routing_time_t> const start_time) {

  auto n = dest_node;
  auto const dest_node_duration = search.cost_.at(n.get_key()).duration(n);
  auto const dest_endpoint_cost =
      get_endpoint_cost<P>(params, w, dest_way, dest_nc, dest_exact_return,
                           flip(opposite(dir), dest_nc.way_dir_), start_time,
                           dest_node_duration, dir);
  auto segments = std::vector<path::segment>{
      {.polyline_ = get_endpoint_path(l, dest_way, dest_nc, dest_exact_return,
                                      dir == direction::kForward, to),
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
      n.get_node() == start_left.node_ ? start_left : start_right;
  auto const start_endpoint_cost = get_endpoint_cost<P>(
      params, w, start_way, start_nc, start_exact_return,
      flip(dir, start_nc.way_dir_), start_time, duration_t{0}, dir);
  segments.push_back(
      {.polyline_ =
           get_endpoint_path(l, start_way, start_nc, start_exact_return,
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
  if (dest_exact_return || start_exact_return) {
    p.track_node_ = node_idx_t::invalid();
  }
  return p;
}

bool component_seen(ways const& w,
                    match_view_t const& matches,
                    size_t match_idx,
                    unsigned times = 1) {
  auto this_component = w.r_->way_component_[matches.way_[match_idx]];
  for (auto j = 0U; j < match_idx; ++j) {
    if (w.r_->way_component_[matches.way_[j]] == this_component) {
      if (--times == 0) {
        return true;
      }
    }
  }
  return false;
}

struct ranked_match_cursor {
  explicit ranked_match_cursor(match_view_t const& matches,
                               bool const single_after_first = false)
      : matches_{matches}, single_after_first_{single_after_first} {}

  std::pair<std::optional<std::size_t>, std::optional<std::size_t>> next() {
    return {next(foot_idx_, false), next(vehicle_idx_, true)};
  }

  template <typename Fn>
  bool for_each_in_stage(unsigned const stage, Fn&& fn) {
    if (single_after_first_ && stage != 0U) {
      while (next_idx_ != matches_.size() &&
             (next_idx_ == first_foot_ || next_idx_ == first_vehicle_)) {
        ++next_idx_;
      }
      if (next_idx_ == matches_.size()) {
        return true;
      }
      fn(next_idx_++);
      return next_idx_ == matches_.size();
    }

    do {
      auto const [foot, vehicle] = next();
      if (!foot.has_value() && !vehicle.has_value()) {
        return true;
      }
      if (stage == 0U) {
        first_foot_ = foot;
        first_vehicle_ = vehicle;
      }
      if (foot.has_value()) {
        fn(*foot);
      }
      if (vehicle.has_value()) {
        fn(*vehicle);
      }
    } while (stage == kAllRemainingStage);
    return false;
  }

  unsigned stage_count() const {
    return single_after_first_ ? static_cast<unsigned>(matches_.size() + 1U)
                               : kStageCount;
  }

  static constexpr auto kStageCount = 3U;

private:
  static constexpr auto kAllRemainingStage = kStageCount - 1U;

  std::optional<std::size_t> next(std::size_t& idx, bool const vehicle) {
    while (idx != matches_.size() && matches_.vehicle_match(idx) != vehicle) {
      ++idx;
    }
    if (idx == matches_.size()) {
      return std::nullopt;
    }
    return idx++;
  }

  match_view_t matches_;
  bool single_after_first_{};
  std::size_t foot_idx_{};
  std::size_t vehicle_idx_{};
  std::size_t next_idx_{};
  std::optional<std::size_t> first_foot_{};
  std::optional<std::size_t> first_vehicle_{};
};

template <SharingProfile P>
struct source_seed {
  typename P::node node_;
  way_idx_t way_;
  candidate_node left_;
  candidate_node right_;
  bool exact_return_{};
  cost_t cost_;
  duration_t duration_;
};

template <SharingProfile P>
void add_source_match(typename P::parameters const& params,
                      ways const& w,
                      level_t const lvl,
                      match_view_t const& matches,
                      std::size_t const match_idx,
                      cost_t const max,
                      direction const dir,
                      std::optional<routing_time_t> const start_time,
                      std::vector<source_seed<P>>& seeds,
                      dijkstra<P>& d) {
  auto const way = matches.way_[match_idx];
  auto const left = matches.left(match_idx);
  auto const right = matches.right(match_idx);
  auto const exact_return = matches.exact_return(match_idx);
  for (auto const* nc : {&left, &right}) {
    if (!nc->valid() || nc->cost_ >= max) {
      continue;
    }
    auto const start_cost = get_endpoint_cost<P>(
        params, w, way, *nc, exact_return, flip(dir, nc->way_dir_), start_time,
        duration_t{0}, dir);
    if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
      continue;
    }
    resolve_endpoint_node<P>(
        *w.r_, way, exact_return, nc->node_, lvl, dir, [&](auto const node) {
          auto const existing = utl::find_if(
              seeds,
              [&](source_seed<P> const& seed) { return seed.node_ == node; });
          auto const candidate = source_seed<P>{node,
                                                way,
                                                left,
                                                right,
                                                exact_return,
                                                start_cost.cost_,
                                                start_cost.duration_};
          if (existing != end(seeds) && existing->cost_ <= candidate.cost_) {
            return;
          }
          if (existing == end(seeds)) {
            seeds.emplace_back(candidate);
          } else {
            *existing = candidate;
          }
          auto label = typename P::label{node, candidate.cost_};
          label.track(label, *w.r_, way, node.get_node(), false);
          d.add_start(w, label, candidate.duration_);
        });
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
std::optional<
    std::tuple<candidate_node, way_idx_t, bool, typename P::node, path>>
best_candidate(typename P::parameters const& params,
               ways const& w,
               Search& search,
               level_t const lvl,
               match_view_t const& m,
               cost_t const max,
               direction const dir,
               std::optional<routing_time_t> const start_time,
               bool should_continue,
               std::optional<way_idx_t> const start_way,
               double const limit_squared_max_matching_distance) {
  using result_t =
      std::tuple<candidate_node, way_idx_t, bool, typename P::node, path>;

  auto const evaluate =
      [&](std::size_t const dest_idx) -> std::optional<result_t> {
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max(),
                          .duration_ = kMaxDuration};
    auto best_node = P::node::invalid();
    auto best = candidate_node{};
    auto have_best = false;
    auto const dest_way = m.way_[dest_idx];
    auto const exact_return = m.exact_return(dest_idx);

    auto const get_best = [&](candidate_node const& x) {
      auto const consider = [&](auto&& node) {
        auto const target_cost = search.get_cost(node);
        if (target_cost == kInfeasible || target_cost > best_cost.cost_) {
          return;
        }

        auto const target_duration =
            search.cost_.at(node.get_key()).duration(node);
        auto const reachable = [&]() {
          if constexpr (SharingProfile<P>) {
            if (exact_return) {
              return P::is_exact_return_reachable(
                  params, *w.r_, w.timezones_, node, dest_way,
                  flip(opposite(dir), x.way_dir_), dir, start_time,
                  target_duration);
            }
          }
          return P::is_dest_reachable(params, *w.r_, w.timezones_, node,
                                      dest_way, flip(opposite(dir), x.way_dir_),
                                      dir, start_time, target_duration);
        }();
        if (!reachable) {
          return;
        }

        auto const dest_way_cost = get_endpoint_cost<P>(
            params, w, dest_way, x, exact_return,
            flip(opposite(dir), x.way_dir_), start_time, target_duration, dir);
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
          have_best = true;
          best_cost.cost_ = static_cast<cost_t>(total_cost);
          best_cost.duration_ = total_duration;
        }
      };
      if constexpr (SharingProfile<P>) {
        if (exact_return) {
          P::resolve_exact_return_node(*w.r_, dest_way, x.node_, lvl,
                                       opposite(dir), consider);
        } else {
          P::resolve_all(*w.r_, x.node_, lvl, consider);
        }
      } else {
        P::resolve_all(*w.r_, x.node_, lvl, consider);
      }
    };

    for (auto const& x : {m.left(dest_idx), m.right(dest_idx)}) {
      if (x.valid()) {
        get_best(x);
      }
    }
    return have_best && best_cost.cost_ < max
               ? std::optional{result_t{best, dest_way, exact_return, best_node,
                                        best_cost}}
               : std::nullopt;
  };

  if constexpr (SharingProfile<P>) {
    auto matches = ranked_match_cursor{m};
    auto component_seen_ctr = 0U;
    for (auto rank = 0U;; ++rank) {
      auto selected = std::optional<result_t>{};
      auto stop = false;
      auto const [foot, vehicle] = matches.next();
      if (!foot.has_value() && !vehicle.has_value()) {
        break;
      }
      for (auto const idx : {foot, vehicle}) {
        if (!idx.has_value()) {
          continue;
        }
        if (!should_continue && ++component_seen_ctr > 10U) {
          stop = true;
          break;
        }
        if (std::pow(m.dist_to_way_[*idx], 2) >
                limit_squared_max_matching_distance &&
            rank > kBottomKDefinitelyConsidered) {
          continue;
        }
        if (auto const candidate = evaluate(*idx);
            candidate.has_value() &&
            (!selected.has_value() ||
             std::get<4>(*candidate).cost_ < std::get<4>(*selected).cost_)) {
          selected = candidate;
        }
      }
      if (selected.has_value()) {
        return selected;
      }
      if (stop) {
        return std::nullopt;
      }
    }
  } else {
    auto const start_component = w.r_->way_component_[*start_way];
    auto component_seen_ctr = 0U;
    for (auto j = std::size_t{0U}; j != m.size(); ++j) {
      if (start_component != w.r_->way_component_[m.way_[j]]) {
        continue;
      }
      if (!should_continue && ++component_seen_ctr > 10U) {
        break;
      }
      if (std::pow(m.dist_to_way_[j], 2) >
              limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }
      if (auto const candidate = evaluate(j); candidate.has_value()) {
        return candidate;
      }
    }
  }
  return std::nullopt;
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
                                        match_view_t const& from_match,
                                        match_view_t const& to_match,
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

  for (auto i = std::size_t{0U}; i != from_match.size(); ++i) {
    if (b.max_reached_1_ && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = from_match.way_[i];
    auto const start_left = from_match.left(i);
    auto const start_right = from_match.right(i);
    for (auto const* nc : {&start_left, &start_right}) {
      if (nc->valid() && nc->cost_ < max) {
        auto const start_cost = P::way_cost(
            params, *w.r_, w.timezones_, start_way,
            w.r_->way_properties_[start_way], flip(dir, nc->way_dir_),
            static_cast<distance_t>(nc->dist_to_node_), {}, duration_t{0}, dir);
        if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
          continue;
        }
        P::resolve_start_node(
            *w.r_, start_way, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, start_cost.cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              b.add_start(params, w, label, sharing, start_cost.duration_);
            });
      }
    }
    if (b.pq1_.empty()) {
      continue;
    }
    for (auto j = std::size_t{0U}; j != to_match.size(); ++j) {
      auto const end_way = to_match.way_[j];
      if (w.r_->way_component_[start_way] != w.r_->way_component_[end_way]) {
        continue;
      }
      if (b.max_reached_2_ && component_seen(w, to_match, j)) {
        continue;
      }
      if (std::pow(to_match.dist_to_way_[j], 2) >
              limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }
      auto const end_left = to_match.left(j);
      auto const end_right = to_match.right(j);
      for (auto const* nc : {&end_left, &end_right}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) {
                auto label = typename P::label{node, nc->cost_};
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
                            to, start_way, start_left, start_right, end_way,
                            end_left, end_right, cost, dir);
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
    match_view_t const& from_match,
    match_view_t const& to_match,
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
    d.reset(max);
    auto seeds = std::vector<source_seed<P>>{};
    auto source_matches = ranked_match_cursor{from_match};
    auto should_continue = true;
    for (auto stage = 0U; stage != ranked_match_cursor::kStageCount; ++stage) {
      auto const exhausted = source_matches.for_each_in_stage(
          stage, [&](std::size_t const start_idx) {
            auto const start_way = from_match.way_[start_idx];
            auto const same_component = [&] {
              for (auto k = std::size_t{0U}; k != to_match.size(); ++k) {
                if (w.r_->way_component_[start_way] ==
                    w.r_->way_component_[to_match.way_[k]]) {
                  return true;
                }
              }
              return false;
            }();
            if (same_component) {
              add_source_match<P>(params, w, from.lvl_, from_match, start_idx,
                                  max, dir, start_time, seeds, d);
            }
          });

      if (d.pq_.empty()) {
        if (exhausted) {
          break;
        }
        continue;
      }
      should_continue = d.run(params, w, *w.r_, max, start_time, blocked,
                              sharing, elevations, dir) &&
                        should_continue;
      auto const c = best_candidate<P>(
          params, w, d, to.lvl_, to_match, max, dir, start_time,
          should_continue, std::nullopt, limit_squared_max_matching_distance);
      if (c.has_value()) {
        auto const [nc, dest_way, dest_exact, node, p] = *c;
        auto const& seed = get_source_seed<P>(d, seeds, node);
        return reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                              from, to, seed.way_, seed.left_, seed.right_,
                              seed.exact_return_, dest_way, nc, dest_exact,
                              node, p.cost_, dir, start_time);
      }
    }
    return std::nullopt;
  } else {
    d.reset(max);
    auto should_continue = true;
    for (auto i = std::size_t{0U}; i != from_match.size(); ++i) {
      if (!should_continue && component_seen(w, from_match, i)) {
        continue;
      }
      auto const start_way = from_match.way_[i];
      auto const start_left = from_match.left(i);
      auto const start_right = from_match.right(i);
      auto const same_component = [&] {
        for (auto k = std::size_t{0U}; k != to_match.size(); ++k) {
          if (w.r_->way_component_[start_way] ==
              w.r_->way_component_[to_match.way_[k]]) {
            return true;
          }
        }
        return false;
      }();
      if (!same_component) {
        continue;
      }

      for (auto const* nc : {&start_left, &start_right}) {
        if (nc->valid() && nc->cost_ < max) {
          auto const start_cost = get_endpoint_cost<P>(
              params, w, start_way, *nc, false, flip(dir, nc->way_dir_),
              start_time, duration_t{0}, dir);
          if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
            continue;
          }
          resolve_endpoint_node<P>(*w.r_, start_way, false, nc->node_,
                                   from.lvl_, dir, [&](auto const node) {
                                     d.add_start(w, {node, start_cost.cost_},
                                                 start_cost.duration_);
                                   });
        }
      }

      if (d.pq_.empty()) {
        continue;
      }

      should_continue = d.run(params, w, *w.r_, max, start_time, blocked,
                              sharing, elevations, dir) &&
                        should_continue;
      auto const c = best_candidate<P>(
          params, w, d, to.lvl_, to_match, max, dir, start_time,
          should_continue, start_way, limit_squared_max_matching_distance);
      if (c.has_value()) {
        auto const [nc, dest_way, dest_exact, node, p] = *c;
        return reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                              from, to, start_way, start_left, start_right,
                              false, dest_way, nc, dest_exact, node, p.cost_,
                              dir, start_time);
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
                                match_view_t const& from_match,
                                match_view_t const& to_match,
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
  for (auto i = std::size_t{0U}; i != from_match.size(); ++i) {
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = from_match.way_[i];
    auto const start_left = from_match.left(i);
    auto const start_right = from_match.right(i);
    auto const same_component = [&] {
      for (auto k = std::size_t{0U}; k != to_match.size(); ++k) {
        if (w.r_->way_component_[start_way] ==
            w.r_->way_component_[to_match.way_[k]]) {
          return true;
        }
      }
      return false;
    }();
    if (!same_component) {
      continue;
    }

    a.reset(max, from, to);
    for (auto j = std::size_t{0U}; j != to_match.size(); ++j) {
      auto const end_way = to_match.way_[j];
      if (w.r_->way_component_[start_way] != w.r_->way_component_[end_way]) {
        continue;
      }
      if (!should_continue && component_seen(w, to_match, j, 10)) {
        continue;
      }
      if (std::pow(to_match.dist_to_way_[j], 2) >
              limit_squared_max_matching_distance &&
          j > kBottomKDefinitelyConsidered) {
        break;
      }

      auto const end_left = to_match.left(j);
      auto const end_right = to_match.right(j);
      for (auto const* nc : {&end_left, &end_right}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_all(*w.r_, nc->node_, to.lvl_, [&](auto const node) {
            if (!P::is_dest_reachable(params, *w.r_, w.timezones_, node,
                                      end_way,
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

    for (auto const* nc : {&start_left, &start_right}) {
      if (nc->valid() && nc->cost_ < max) {
        auto const start_cost = P::way_cost(
            params, *w.r_, w.timezones_, start_way,
            w.r_->way_properties_[start_way], flip(dir, nc->way_dir_),
            static_cast<distance_t>(nc->dist_to_node_), start_time,
            duration_t{0}, dir);
        if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
          continue;
        }
        P::resolve_start_node(
            *w.r_, start_way, nc->node_, from.lvl_, dir, [&](auto const node) {
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
                                     start_time, should_continue, start_way,
                                     limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, dest_way, dest_exact, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, a, from,
                            to, start_way, start_left, start_right, false,
                            dest_way, nc, dest_exact, node, p.cost_, dir,
                            start_time);
    }
  }

  return std::nullopt;
}

template <Profile P>
std::vector<std::optional<path>> route(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    dijkstra<P>& d,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
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
    for (auto i = std::size_t{0U}; i != to.size(); ++i) {
      if (auto const p = try_direct(from, to[i]); p.has_value()) {
        result[i] = p;
      }
    }
    d.reset(max);
    auto seeds = std::vector<source_seed<P>>{};
    auto exact_return_backward = dir == direction::kBackward;
    if (exact_return_backward) {
      exact_return_backward = false;
      for (auto i = std::size_t{0U}; i != from_match.size(); ++i) {
        if (from_match.exact_return(i)) {
          exact_return_backward = true;
          break;
        }
      }
    }

    auto source_matches =
        ranked_match_cursor{from_match, exact_return_backward};
    auto should_continue = true;
    for (auto stage = 0U; stage != source_matches.stage_count(); ++stage) {
      auto const exhausted = source_matches.for_each_in_stage(
          stage, [&](std::size_t const start_idx) {
            if (exact_return_backward && stage != 0U && !should_continue &&
                component_seen(w, from_match, start_idx)) {
              return;
            }
            auto const start_way = from_match.way_[start_idx];
            auto relevant = false;
            for (auto j = std::size_t{0U}; j != to_match.size() && !relevant;
                 ++j) {
              if (result[j].has_value()) {
                continue;
              }
              auto const destinations =
                  to_match[match_idx_t{static_cast<match_idx_t::value_t>(j)}];
              for (auto k = std::size_t{0U}; k != destinations.size(); ++k) {
                if (w.r_->way_component_[start_way] ==
                    w.r_->way_component_[destinations.way_[k]]) {
                  relevant = true;
                  break;
                }
              }
            }
            if (relevant) {
              add_source_match<P>(params, w, from.lvl_, from_match, start_idx,
                                  max, dir, start_time, seeds, d);
            }
          });

      if (d.pq_.empty()) {
        if (exhausted) {
          break;
        }
        continue;
      }
      should_continue = d.run(params, w, *w.r_, max, start_time, blocked,
                              sharing, elevations, dir) &&
                        should_continue;

      auto found = 0U;
      for (auto j = std::size_t{0U}; j != result.size(); ++j) {
        if (result[j].has_value()) {
          ++found;
          continue;
        }
        auto const destinations =
            to_match[match_idx_t{static_cast<match_idx_t::value_t>(j)}];
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from.pos_, to[j].pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;
        auto const c = best_candidate<P>(
            params, w, d, to[j].lvl_, destinations, max, dir, start_time,
            should_continue, std::nullopt, limit_squared_max_matching_distance);
        if (!c.has_value()) {
          continue;
        }
        auto [nc, dest_way, dest_exact, n, p] = *c;
        d.cost_.at(n.get_key()).write(n, p);
        if (do_reconstruct(p)) {
          auto const& seed = get_source_seed<P>(d, seeds, n);
          p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                             from, to[j], seed.way_, seed.left_, seed.right_,
                             seed.exact_return_, dest_way, nc, dest_exact, n,
                             p.cost_, dir, start_time);
          p.uses_elevator_ = true;
        }
        result[j] = std::move(p);
        ++found;
      }
      if (found == result.size()) {
        return result;
      }
    }
    return result;
  } else {
    d.reset(max);
    auto should_continue = true;
    for (auto i = std::size_t{0U}; i != from_match.size(); ++i) {
      if (!should_continue && component_seen(w, from_match, i)) {
        continue;
      }
      auto const start_way = from_match.way_[i];
      auto const start_left = from_match.left(i);
      auto const start_right = from_match.right(i);
      for (auto const* nc : {&start_left, &start_right}) {
        if (nc->valid() && nc->cost_ < max) {
          auto const start_cost = get_endpoint_cost<P>(
              params, w, start_way, *nc, false, flip(dir, nc->way_dir_),
              start_time, duration_t{0}, dir);
          if (start_cost.cost_ == kInfeasible || start_cost.cost_ >= max) {
            continue;
          }
          resolve_endpoint_node<P>(
              *w.r_, start_way, false, nc->node_, from.lvl_, dir,
              [&](auto const node) {
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
      for (auto k = std::size_t{0U}; k != result.size(); ++k) {
        auto const m =
            to_match[match_idx_t{static_cast<match_idx_t::value_t>(k)}];
        auto const& t = to[k];
        auto& r = result[k];
        if (r.has_value()) {
          ++found;
        } else if (auto const direct = try_direct(from, t);
                   direct.has_value()) {
          r = direct;
        } else {
          auto const limit_squared_max_matching_distance =
              geo::approx_squared_distance(from.pos_, t.pos_,
                                           distance_lng_degrees) /
              kMaxMatchingDistanceSquaredRatio;
          auto const c = best_candidate<P>(
              params, w, d, t.lvl_, m, max, dir, start_time, should_continue,
              start_way, limit_squared_max_matching_distance);
          if (c.has_value()) {
            auto [nc, dest_way, dest_exact, n, p] = *c;
            d.cost_.at(n.get_key()).write(n, p);
            if (do_reconstruct(p)) {
              p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                                 from, t, start_way, start_left, start_right,
                                 false, dest_way, nc, dest_exact, n, p.cost_,
                                 dir, start_time);
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
    auto from_m = match_result{};
    l.complete_match<P>(pp, from, false, dir, max_match_distance, blocked,
                        std::nullopt, {}, from_m);
    auto to_m = match_result{};
    l.complete_match<P>(pp, to, true, dir, max_match_distance, blocked,
                        std::nullopt, {}, to_m);
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

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
        auto from_m = match_result{};
        l.match_endpoint<P>(pp, from, false, dir, max_match_distance, blocked,
                            endpoint_options.exact_return_at_from_, from_m,
                            start_time);
        auto const from_match = from_m[match_idx_t{0U}];
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto to_match = match_result{};
        for (auto const [i, x] : utl::enumerate(to)) {
          l.match_endpoint<P>(pp, x, true, dir, max_match_distance, blocked,
                              endpoint_options.exact_return_at_to(i), to_match,
                              start_time);
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
    auto from_m = match_result{};
    auto to_m = match_result{};
    if constexpr (SharingProfile<P>) {
      l.match_endpoint<P>(pp, from, false, dir, max_match_distance, blocked,
                          false, from_m, start_time);
      l.match_endpoint<P>(pp, to, true, dir, max_match_distance, blocked, false,
                          to_m, start_time);
    } else {
      l.complete_match<P>(pp, from, false, dir, max_match_distance, blocked,
                          start_time, {}, from_m);
      l.complete_match<P>(pp, to, true, dir, max_match_distance, blocked,
                          start_time, {}, to_m);
    }
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

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
    auto from_m = match_result{};
    l.complete_match<P>(pp, from, false, dir, max_match_distance, blocked,
                        start_time, {}, from_m);
    auto to_m = match_result{};
    l.complete_match<P>(pp, to, true, dir, max_match_distance, blocked,
                        start_time, {}, to_m);
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

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
    match_view_t const& from_match,
    match_result const& to_match,
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
                          match_view_t const& from_match,
                          match_view_t const& to_match,
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
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing) {
    return with_profile(profile, [&]<Profile P>(P&&) -> std::optional<path> {
      auto const& pp = std::get<typename P::parameters>(params);
      auto from_m = match_result{};
      l.match_endpoint<P>(pp, from, false, dir, max_match_distance, blocked,
                          endpoint_options.exact_return_at_from_, from_m,
                          start_time);
      auto to_m = match_result{};
      l.match_endpoint<P>(pp, to, true, dir, max_match_distance, blocked,
                          endpoint_options.exact_return_at_to(0U), to_m,
                          start_time);
      auto const from_match = from_m[match_idx_t{0U}];
      auto const to_match = to_m[match_idx_t{0U}];
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
