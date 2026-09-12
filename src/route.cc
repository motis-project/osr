#include "osr/routing/route.h"

#include <cassert>
#include <cstdint>

#include <algorithm>
#include <optional>

#include "utl/concat.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "boost/thread/tss.hpp"
#include "osr/elevation_storage.h"
#include "osr/lookup.h"
#include "osr/routing/astar.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/cch.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/dijkstra_bidir.h"
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
constexpr auto const kMinCostSettled = cost_t{900};

duration_t sum_segment_durations(std::vector<path::segment> const& segments,
                                 duration_t total = duration_t{0}) {
  for (auto const& segment : segments) {
    total = clamp_add_duration(total, segment.duration_);
  }
  return total;
}

template <Profile P>
dijkstra_bidir<P>& get_dijkstra_bidir() {
  static auto s = boost::thread_specific_ptr<dijkstra_bidir<P>>{};
  if (s.get() == nullptr) {
    s.reset(new dijkstra_bidir<P>{});
  }
  return *s.get();
}

template <Profile P>
cch<P>& get_cch() {
  static auto s = boost::thread_specific_ptr<cch<P>>{};
  if (s.get() == nullptr) {
    s.reset(new cch<P>{});
  }
  return *s.get();
}

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("cch"): return routing_algorithm::kCCH;
    case cista::hash("dijkstra_bidir"): return routing_algorithm::kDijkstraBi;
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
}

bool is_start_candidate(auto const& n,
                        node_idx_t const root,
                        cost_t const expected_cost) {
  // Matching node alone is not enough:
  // on a loop way, left and right are the same node but with different costs
  return n.node_ == root && n.cost_ == expected_cost;
}

candidate_node const& start_candidate(node_idx_t const root,
                                      cost_t const expected_cost,
                                      candidate_node const& left,
                                      candidate_node const& right) {
  if (is_start_candidate(left, root, expected_cost)) {
    return left;
  } else if (is_start_candidate(right, root, expected_cost)) {
    return right;
  } else {
    assert(false);  // should not happen
    return left.node_ == root ? left : right;
  }
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

  auto const& start_node_candidate = start_candidate(
      forward_n.get_node(), b.template get_cost<direction::kForward>(forward_n),
      start_left, start_right);

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
                 way_idx_t const dest_way,
                 candidate_node const& dest_nc,
                 typename P::node const dest_node,
                 cost_t const cost,
                 direction const dir,
                 std::optional<routing_time_t> const start_time) {

  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ =
           l.get_node_candidate_path(dest_way, dest_nc.node_, dest_nc.way_dir_,
                                     dir == direction::kForward, to),
       .from_level_ = dest_nc.lvl_,
       .to_level_ = dest_nc.lvl_,
       .from_ =
           dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .to_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc.cost_,
       .duration_ = duration_from_cost(dest_nc.cost_),
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

  auto const& start_nc = start_candidate(n.get_node(), search.get_cost(n),
                                         start_left, start_right);
  segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start_way, start_nc.node_, start_nc.way_dir_,
           dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
       .duration_ = duration_from_cost(start_nc.cost_),
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = n.get_mode()});
  if (dir == direction::kForward) {
    std::reverse(begin(segments), end(segments));
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

constexpr auto const kCchRouteDebugOutput = false;

struct cch_way_candidate {
  way_idx_t way_;
  candidate_node left_, right_;
};

cch_way_candidate cch_candidate(match_view_t const& match, std::size_t i) {
  return {match.way_[i], match.left(i), match.right(i)};
}

template <Profile P>
std::optional<path> reconstruct_dijkstra_bidir(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    dijkstra_bidir<P> const& d,
    location const& from,
    location const& to,
    cch_way_candidate const& start,
    cch_way_candidate const& dest,
    direction const dir) {
  if (d.meet_.get_node() == node_idx_t::invalid()) {
    return std::nullopt;
  }

  // Walk from the meeting node back to the selected start candidate through the
  // predecessor chain produced by the forward search.
  auto forward_n = d.meet_;
  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;
  while (true) {
    auto const& e = d.costForward_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const expected_cost = static_cast<cost_t>(
          e.cost(forward_n) - d.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      forward_n, duration_t{0}, std::nullopt, expected_cost,
                      duration_from_cost(expected_cost), forward_segments, dir);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_nc =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  // Add the off-graph segment from the requested start location to the first
  // graph node used by the forward search.
  forward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start.way_, start_nc.node_, start_nc.way_dir_,
           dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
       .duration_ = duration_from_cost(start_nc.cost_),
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = forward_n.get_mode()});

  // Walk from the meeting node back to the selected destination candidate
  // through the predecessor chain produced by the backward search.
  auto backward_n = d.meet_;
  auto backward_segments = std::vector<path::segment>{};
  auto backward_dist = 0.0;
  while (true) {
    auto const& e = d.costBackward_.at(backward_n.get_key());
    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) -
                              d.template get_cost<direction::kBackward>(*pred));
      backward_dist += add_path<P>(
          params, w, *w.r_, blocked, sharing, elevations, *pred, backward_n,
          duration_t{0}, std::nullopt, expected_cost,
          duration_from_cost(expected_cost), backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const* dest_nc = static_cast<candidate_node const*>(nullptr);
  auto const seed_cost = d.template get_cost<direction::kBackward>(backward_n);
  for (auto const* nc : {&dest.left_, &dest.right_}) {
    if (nc->valid() && nc->node_ == backward_n.get_node()) {
      dest_nc = nc;
      if (nc->cost_ == seed_cost) {
        break;
      }
    }
  }
  if (dest_nc == nullptr) {
    for (auto const* nc : {&dest.left_, &dest.right_}) {
      if (nc->valid()) {
        dest_nc = nc;
        break;
      }
    }
  }
  if (dest_nc == nullptr) {
    return std::nullopt;
  }

  // Add the off-graph segment from the final graph node to the requested
  // destination location.
  backward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path(dest.way_, dest_nc->node_,
                                              dest_nc->way_dir_,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc->lvl_,
       .to_level_ = dest_nc->lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc->cost_,
       .duration_ = duration_from_cost(dest_nc->cost_),
       .dist_ = static_cast<distance_t>(dest_nc->dist_to_node_),
       .mode_ = backward_n.get_mode()});

  // Both predecessor walks append segments in predecessor-chain order. Reverse
  // the half that points away from the requested output direction.
  if (dir == direction::kForward) {
    std::reverse(begin(forward_segments), end(forward_segments));
  } else {
    std::reverse(begin(backward_segments), end(backward_segments));
  }
  forward_segments.insert(end(forward_segments), begin(backward_segments),
                          end(backward_segments));

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : forward_segments) {
    path_elevation += segment.elevation_;
  }

  return path{.cost_ = d.mu_,
              .duration_ = sum_segment_durations(forward_segments),
              .dist_ = start_nc.dist_to_node_ + forward_dist + backward_dist +
                       dest_nc->dist_to_node_,
              .elevation_ = path_elevation,
              .segments_ = forward_segments};
}

template <Profile P>
double add_cch_path(typename P::parameters const& params,
                    ways const& w,
                    typename P::node from,
                    typename P::node to,
                    cost_t expected_cost,
                    std::vector<path::segment>& segments,
                    direction dir,
                    std::uint32_t depth = 0U,
                    bool turn_at_source = false,
                    bool turn_at_target = false);

template <Profile P>
std::optional<path> reconstruct_cch(typename P::parameters const& params,
                                    ways const& w,
                                    lookup const& l,
                                    bitvec<node_idx_t> const* blocked,
                                    sharing_data const* sharing,
                                    elevation_storage const* elevations,
                                    cch<P> const& c,
                                    location const& from,
                                    location const& to,
                                    cch_way_candidate const& start,
                                    cch_way_candidate const& dest,
                                    direction const dir) {
  (void)blocked;
  (void)sharing;
  (void)elevations;

  if (c.meet_forward_.get_node() == node_idx_t::invalid() ||
      c.meet_backward_.get_node() == node_idx_t::invalid()) {
    return std::nullopt;
  }

  // Walk from the meeting node back to the selected start candidate through the
  // predecessor chain produced by the forward search.
  auto forward_n = c.meet_forward_;
  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;
  while (true) {
    auto const& e = c.costForward_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const expected_cost = static_cast<cost_t>(
          e.cost(forward_n) - c.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_cch_path<P>(params, w, *pred, forward_n, expected_cost,
                          forward_segments, dir, 0U, true, false);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_nc =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  // Add the off-graph segment from the requested start location to the first
  // graph node used by the forward search.
  forward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start.way_, start_nc.node_, start_nc.way_dir_,
           dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
       .duration_ = duration_from_cost(start_nc.cost_),
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = forward_n.get_mode()});

  // Walk from the meeting node back to the selected destination candidate
  // through the predecessor chain produced by the backward search.
  auto backward_n = c.meet_backward_;
  auto backward_segments = std::vector<path::segment>{};
  auto backward_dist = 0.0;
  while (true) {
    auto const& e = c.costBackward_.at(backward_n.get_key());
    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) -
                              c.template get_cost<direction::kBackward>(*pred));
      backward_dist +=
          add_cch_path<P>(params, w, backward_n, *pred, expected_cost,
                          backward_segments, dir, 0U, false, true);
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const* dest_nc = static_cast<candidate_node const*>(nullptr);
  auto const seed_cost = c.template get_cost<direction::kBackward>(backward_n);
  for (auto const* nc : {&dest.left_, &dest.right_}) {
    if (nc->valid() && nc->node_ == backward_n.get_node()) {
      dest_nc = nc;
      if (nc->cost_ == seed_cost) {
        break;
      }
    }
  }
  if (dest_nc == nullptr) {
    for (auto const* nc : {&dest.left_, &dest.right_}) {
      if (nc->valid()) {
        dest_nc = nc;
        break;
      }
    }
  }
  if (dest_nc == nullptr) {
    return std::nullopt;
  }

  // Add the off-graph segment from the final graph node to the requested
  // destination location.
  backward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path(dest.way_, dest_nc->node_,
                                              dest_nc->way_dir_,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc->lvl_,
       .to_level_ = dest_nc->lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc->cost_,
       .duration_ = duration_from_cost(dest_nc->cost_),
       .dist_ = static_cast<distance_t>(dest_nc->dist_to_node_),
       .mode_ = backward_n.get_mode()});

  // Both predecessor walks append segments in predecessor-chain order. Reverse
  // the half that points away from the requested output direction.
  if (dir == direction::kForward) {
    std::reverse(begin(forward_segments), end(forward_segments));
  } else {
    std::reverse(begin(backward_segments), end(backward_segments));
  }
  forward_segments.insert(end(forward_segments), begin(backward_segments),
                          end(backward_segments));

  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : forward_segments) {
    path_elevation += segment.elevation_;
  }

  return path{.cost_ = c.mu_,
              .duration_ = sum_segment_durations(forward_segments),
              .dist_ = start_nc.dist_to_node_ + forward_dist + backward_dist +
                       dest_nc->dist_to_node_,
              .elevation_ = path_elevation,
              .segments_ = forward_segments};
}

template <Profile P>
shortcut const* find_cch_shortcut(ways::routing const& r,
                                  typename P::node const from,
                                  typename P::node const to) {
  for (auto const& s : r.shortcuts_[from.get_node()]) {
    if (s.to_ == to.get_node()) {
      return &s;
    }
  }
  return nullptr;
}

struct cch_edge_ref {
  cch_edge const* edge_{};
  cch_edge_weight const* weight_{};
  bool up_{};
};

template <Profile P>
cost_t get_cch_turn_cost(typename P::parameters const& params,
                         ways::routing const& r,
                         typename P::node const incoming,
                         way_pos_t const outgoing_way,
                         direction const outgoing_dir) {
  if (r.template is_restricted<direction::kForward, cch<P>::is_bus_profile()>(
          incoming.get_node(), incoming.way_, outgoing_way)) {
    return kInfeasible;
  }
  auto const is_u_turn =
      incoming.way_ == outgoing_way && outgoing_dir == opposite(incoming.dir_);
  return is_u_turn ? params.uturn_penalty_
                   : P::turn_cost(params,
                                  r.get_turn_angle(incoming.get_node(),
                                                   incoming.way_, incoming.dir_,
                                                   outgoing_way, outgoing_dir));
}

template <Profile P>
cch_edge_ref find_cch_edge_ref(ways::routing const& r,
                               typename P::node const from,
                               typename P::node const to) {
  auto const self = from.get_node() == to.get_node();
  auto const up = self || r.node_importance_[from.get_node()] <
                              r.node_importance_[to.get_node()];
  auto const low = up ? from.get_node() : to.get_node();
  auto const high = up ? to.get_node() : from.get_node();
  for (auto const& e : cch<P>::customized_edges(r)[low]) {
    if (e.to_ == high) {
      auto const* weight = static_cast<cch_edge_weight const*>(nullptr);
      for (auto const& candidate : e.weights_) {
        if ((!self && candidate.up_ != up) ||
            candidate.from_way_ != from.way_ ||
            candidate.from_dir_ != from.dir_ || candidate.to_way_ != to.way_ ||
            candidate.to_dir_ != to.dir_) {
          continue;
        }
        if (weight == nullptr || candidate.cost_ < weight->cost_ ||
            (candidate.cost_ == weight->cost_ &&
             candidate.distance_ < weight->distance_)) {
          weight = &candidate;
        }
      }
      return {.edge_ = &e,
              .weight_ = weight,
              .up_ = weight == nullptr ? up : weight->up_};
    }
  }
  return {};
}

template <Profile P>
cch_edge_ref find_cch_transition_ref(typename P::parameters const& params,
                                     ways::routing const& r,
                                     typename P::node const from,
                                     typename P::node const to,
                                     cost_t const expected_cost,
                                     bool const turn_at_source,
                                     bool const turn_at_target) {
  auto const self = from.get_node() == to.get_node();
  auto const up = self || r.node_importance_[from.get_node()] <
                              r.node_importance_[to.get_node()];
  auto const low = up ? from.get_node() : to.get_node();
  auto const high = up ? to.get_node() : from.get_node();
  for (auto const& e : cch<P>::customized_edges(r)[low]) {
    if (e.to_ != high) {
      continue;
    }
    for (auto const& candidate : e.weights_) {
      if ((!self && candidate.up_ != up) ||
          (self && turn_at_source && !candidate.up_) ||
          (self && turn_at_target && candidate.up_)) {
        continue;
      }

      auto total = candidate.cost_;
      if (turn_at_source) {
        if (candidate.to_way_ != to.way_ || candidate.to_dir_ != to.dir_) {
          continue;
        }
        auto const turn = get_cch_turn_cost<P>(
            params, r, from, candidate.from_way_, candidate.from_dir_);
        if (turn == kInfeasible) {
          continue;
        }
        total = clamp_cost(static_cast<std::uint64_t>(total) + turn);
      } else if (turn_at_target) {
        if (candidate.from_way_ != from.way_ ||
            candidate.from_dir_ != from.dir_) {
          continue;
        }
        auto const incoming = P::create_node(
            to.get_node(), kNoLevel, candidate.to_way_, candidate.to_dir_);
        auto const turn =
            get_cch_turn_cost<P>(params, r, incoming, to.way_, to.dir_);
        if (turn == kInfeasible) {
          continue;
        }
        total = clamp_cost(static_cast<std::uint64_t>(total) + turn);
      }

      if (total == expected_cost) {
        return {.edge_ = &e, .weight_ = &candidate, .up_ = candidate.up_};
      }
    }
    return {.edge_ = &e, .weight_ = nullptr, .up_ = up};
  }
  return {};
}

template <Profile P>
std::optional<distance_t> get_direct_cch_distance(ways::routing const& r,
                                                  typename P::node const from,
                                                  typename P::node const to) {
  // Shortcut unpacking bottoms out at original neighboring graph nodes.
  for (auto const [way, from_idx] :
       utl::zip(r.node_ways_[from.get_node()],
                r.node_in_way_idx_[from.get_node()])) {
    auto const nodes = r.way_nodes_[way];
    if (from_idx != 0U && nodes[from_idx - 1U] == to.get_node()) {
      return r.get_way_node_distance(way, from_idx - 1U);
    }
    if (from_idx + 1U < nodes.size() && nodes[from_idx + 1U] == to.get_node()) {
      return r.get_way_node_distance(way, from_idx);
    }
  }
  return std::nullopt;
}

template <Profile P>
cost_t get_cch_edge_cost(typename P::parameters const& params,
                         ways::routing const& r,
                         typename P::node const from,
                         typename P::node const to) {
  if constexpr (cch<P>::uses_customized_cost_overlay()) {
    auto const e = find_cch_edge_ref<P>(r, from, to);
    if (e.weight_ != nullptr) {
      return e.weight_->cost_;
    }
  }
  if (auto const* s = find_cch_shortcut<P>(r, from, to); s != nullptr) {
    return cch<P>::shortcut_cost(params, s->distance_);
  }
  if (auto const direct = get_direct_cch_distance<P>(r, from, to);
      direct.has_value()) {
    return cch<P>::shortcut_cost(params, *direct);
  }
  return kInfeasible;
}

template <Profile P>
double add_direct_cch_path(ways const& w,
                           typename P::node const from,
                           typename P::node const to,
                           cost_t const expected_cost,
                           std::vector<path::segment>& segments) {
  auto const dist = get_direct_cch_distance<P>(*w.r_, from, to);
  utl::verify(dist.has_value(), "no direct CCH base edge node/{} -> node/{}",
              to_idx(w.node_to_osm_[from.get_node()]),
              to_idx(w.node_to_osm_[to.get_node()]));

  auto conn = std::optional<connecting_way>{};
  auto const& r = *w.r_;
  auto const from_ways = r.node_ways_[from.get_node()];
  auto const from_indices = r.node_in_way_idx_[from.get_node()];
  auto const consider_way = [&](way_pos_t const from_way_pos) {
    auto const way = from_ways[from_way_pos];
    auto const from_idx = from_indices[from_way_pos];
    auto const nodes = r.way_nodes_[way];
    auto try_connect = [&](std::uint16_t const to_idx) {
      if (nodes[to_idx] != to.get_node()) {
        return;
      }
      auto const lower_idx = std::min(from_idx, to_idx);
      auto const is_loop =
          r.is_loop(way) &&
          static_cast<unsigned>(std::abs(static_cast<int>(from_idx) -
                                         static_cast<int>(to_idx))) ==
              nodes.size() - 2U;
      conn = connecting_way{way,
                            from_idx,
                            to_idx,
                            is_loop,
                            r.get_way_node_distance(way, lower_idx),
                            elevation_storage::elevation{}};
    };
    if (from_idx != 0U) {
      try_connect(static_cast<std::uint16_t>(from_idx - 1U));
    }
    if (from_idx + 1U < nodes.size()) {
      try_connect(static_cast<std::uint16_t>(from_idx + 1U));
    }
  };

  if constexpr (requires { from.way_; }) {
    if (from.way_ < from_ways.size()) {
      consider_way(from.way_);
    }
  }
  for (auto i = way_pos_t{0U}; !conn.has_value() && i != from_ways.size();
       ++i) {
    consider_way(i);
  }

  utl::verify(conn.has_value(), "no direct CCH way node/{} -> node/{}",
              to_idx(w.node_to_osm_[from.get_node()]),
              to_idx(w.node_to_osm_[to.get_node()]));

  auto const& [way, from_idx, to_idx, is_loop, distance, elevation] = *conn;
  auto& segment = segments.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.duration_ = duration_from_cost(expected_cost);
  segment.elevation_ = elevation;
  segment.mode_ = to.get_mode();

  auto const is_reverse = (from_idx > to_idx) ^ is_loop;
  if (is_reverse) {
    segment.from_level_ = r.way_properties_[way].to_level();
    segment.to_level_ = r.way_properties_[way].from_level();
  } else {
    segment.from_level_ = r.way_properties_[way].from_level();
    segment.to_level_ = r.way_properties_[way].to_level();
  }
  segment.from_ = r.way_nodes_[way][from_idx];
  segment.to_ = r.way_nodes_[way][to_idx];

  auto j = 0U;
  auto active = false;
  for (auto const [osm_idx, coord] :
       infinite(reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                        is_reverse),
                is_loop)) {
    utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U, "infinite loop");
    if (!active && w.node_to_osm_[segment.from_] == osm_idx) {
      active = true;
    }
    if (active) {
      if (w.node_to_osm_[segment.from_] == osm_idx) {
        // Again "from" node, then it's shorter to start from here.
        segment.polyline_.clear();
      }

      segment.polyline_.emplace_back(coord);
      if (w.node_to_osm_[segment.to_] == osm_idx) {
        break;
      }
    }
  }

  return distance;
}

template <Profile P>
double add_cch_path(typename P::parameters const& params,
                    ways const& w,
                    typename P::node const from,
                    typename P::node const to,
                    cost_t const expected_cost,
                    std::vector<path::segment>& segments,
                    direction const dir,
                    std::uint32_t const depth,
                    bool const turn_at_source,
                    bool const turn_at_target) {
  if constexpr (cch<P>::uses_customized_cost_overlay()) {
    auto const e =
        turn_at_source || turn_at_target
            ? find_cch_transition_ref<P>(params, *w.r_, from, to, expected_cost,
                                         turn_at_source, turn_at_target)
            : find_cch_edge_ref<P>(*w.r_, from, to);
    if (e.weight_ != nullptr) {
      auto const cost = e.weight_->cost_;
      auto const distance = e.weight_->distance_;
      auto const via = e.weight_->via_;
      auto const from_way = e.weight_->from_way_;
      auto const to_way = e.weight_->to_way_;
      auto const from_dir = e.weight_->from_dir_;
      auto const to_dir = e.weight_->to_dir_;
      auto const edge_from =
          P::create_node(from.get_node(), kNoLevel, from_way, from_dir);
      auto const edge_to =
          P::create_node(to.get_node(), kNoLevel, to_way, to_dir);
      // CCH DEBUG: log every selected overlay edge, including recursively
      // unpacked base edges, so the query path is not confused with only the
      // top-level shortcut breadcrumbs shown in the debug UI.
      if constexpr (kCchRouteDebugOutput) {
        fmt::println(
            "cch selected edge | depth {} | kind {} | node/{} -> node/{} | "
            "ranks {} -> {} | {} | cost {} | expected {} | dist {} | via "
            "node/{} | boundary {}:{} -> {}:{}",
            depth, via == node_idx_t::invalid() ? "base" : "shortcut",
            to_idx(w.node_to_osm_[from.get_node()]),
            to_idx(w.node_to_osm_[to.get_node()]),
            w.r_->node_importance_[from.get_node()],
            w.r_->node_importance_[to.get_node()], e.up_ ? "up" : "down", cost,
            expected_cost, distance,
            via == node_idx_t::invalid() ? 0U : to_idx(w.node_to_osm_[via]),
            static_cast<unsigned>(from_way), to_str(from_dir),
            static_cast<unsigned>(to_way), to_str(to_dir));
      }
      auto const via_node = e.weight_->via_;
      if (via_node != node_idx_t::invalid()) {
        if constexpr (kCchRouteDebugOutput) {
          // Keep a visible breadcrumb for each selected shortcut before it is
          // recursively unpacked into original graph edges.
          segments.push_back(path::segment{
              .polyline_ = {w.get_node_pos(from.get_node()).as_latlng(),
                            w.get_node_pos(to.get_node()).as_latlng()},
              .from_level_ = level_t{0.F},
              .to_level_ = level_t{0.F},
              .from_ = from.get_node(),
              .to_ = to.get_node(),
              .way_ = way_idx_t::invalid(),
              .cost_ = expected_cost,
              .dist_ = e.weight_->distance_,
              .mode_ = to.get_mode(),
              .cch_debug_shortcut_ = true,
              .cch_debug_depth_ = depth,
              .cch_debug_via_ = via_node});
        }
        // Customized CCH edges can represent a path through a lower-rank
        // via-node, even when the edge is also an original graph edge.
        auto const via_in = P::create_node(
            via_node, kNoLevel, e.weight_->via_in_way_, e.weight_->via_in_dir_);
        auto const via_out =
            P::create_node(via_node, kNoLevel, e.weight_->via_out_way_,
                           e.weight_->via_out_dir_);
        auto const first_cost =
            get_cch_edge_cost<P>(params, *w.r_, edge_from, via_in);
        auto const second_cost =
            get_cch_edge_cost<P>(params, *w.r_, via_out, edge_to);
        return add_cch_path<P>(params, w, edge_from, via_in, first_cost,
                               segments, dir, depth + 1U) +
               add_cch_path<P>(params, w, via_out, edge_to, second_cost,
                               segments, dir, depth + 1U);
      }
      return add_direct_cch_path<P>(w, edge_from, edge_to, e.weight_->cost_,
                                    segments);
    } else {
      if constexpr (kCchRouteDebugOutput) {
        fmt::println(
            "cch edge depth {} node/{} -> node/{} | missing overlay weight",
            depth, to_idx(w.node_to_osm_[from.get_node()]),
            to_idx(w.node_to_osm_[to.get_node()]));
      }
    }
  }
  if (auto const* s = find_cch_shortcut<P>(*w.r_, from, to); s != nullptr) {
    // CCH predecessor edges can be shortcuts. Recursively unpack them through
    // their contracted via-node until only original graph edges remain.
    // CCH DEBUG: raw shortcut fallback breadcrumb retained for comparison with
    // customized overlay reconstruction.
    if constexpr (kCchRouteDebugOutput) {
      fmt::println(
          "cch selected edge | depth {} | kind raw-shortcut-fallback | node/{} "
          "-> node/{} | ranks {} -> {} | cost {} | expected {} | dist {} | via "
          "node/{}",
          depth, to_idx(w.node_to_osm_[from.get_node()]),
          to_idx(w.node_to_osm_[to.get_node()]),
          w.r_->node_importance_[from.get_node()],
          w.r_->node_importance_[to.get_node()],
          get_cch_edge_cost<P>(params, *w.r_, from, to), expected_cost,
          s->distance_, to_idx(w.node_to_osm_[s->via_]));
    }
    if constexpr (kCchRouteDebugOutput) {
      segments.push_back(path::segment{
          .polyline_ = {w.get_node_pos(from.get_node()).as_latlng(),
                        w.get_node_pos(to.get_node()).as_latlng()},
          .from_level_ = level_t{0.F},
          .to_level_ = level_t{0.F},
          .from_ = from.get_node(),
          .to_ = to.get_node(),
          .way_ = way_idx_t::invalid(),
          .cost_ = expected_cost,
          .dist_ = s->distance_,
          .mode_ = to.get_mode(),
          .cch_debug_shortcut_ = true,
          .cch_debug_depth_ = depth,
          .cch_debug_via_ = s->via_});
    }
    auto const via = P::create_node(s->via_, kNoLevel, way_pos_t{0U}, dir);
    auto const first_cost = get_cch_edge_cost<P>(params, *w.r_, from, via);
    auto const second_cost = get_cch_edge_cost<P>(params, *w.r_, via, to);
    return add_cch_path<P>(params, w, from, via, first_cost, segments, dir,
                           depth + 1U) +
           add_cch_path<P>(params, w, via, to, second_cost, segments, dir,
                           depth + 1U);
  }
  return add_direct_cch_path<P>(w, from, to, expected_cost, segments);
}

template <Profile P, typename Search>
std::optional<std::tuple<candidate_node, way_idx_t, typename P::node, path>>
best_candidate(typename P::parameters const& params,
               ways const& w,
               Search& search,
               level_t const lvl,
               match_view_t const& m,
               cost_t const max,
               direction const dir,
               std::optional<routing_time_t> const start_time,
               bool should_continue,
               way_idx_t const start_way,
               double const limit_squared_max_matching_distance) {
  auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max(),
                        .duration_ = kMaxDuration};
  auto best_node = P::node::invalid();
  auto best = candidate_node{};
  auto have_best = false;

  auto const get_best = [&](way_idx_t const dest_way, candidate_node const& x) {
    P::resolve_all(*w.r_, x.node_, lvl, [&](auto&& node) {
      auto const target_cost = search.get_cost(node);
      if (target_cost == kInfeasible || target_cost > best_cost.cost_) {
        return;
      }

      auto const target_duration =
          search.cost_.at(node.get_key()).duration(node);
      if (!P::is_dest_reachable(params, *w.r_, w.timezones_, node, dest_way,
                                flip(opposite(dir), x.way_dir_), dir,
                                start_time, target_duration)) {
        return;
      }

      auto const dest_way_cost = P::way_cost(
          params, *w.r_, w.timezones_, dest_way,
          w.r_->way_properties_[dest_way], flip(opposite(dir), x.way_dir_),
          static_cast<distance_t>(x.dist_to_node_), start_time, target_duration,
          dir);
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
    });
  };

  auto const start_component = w.r_->way_component_[start_way];
  auto component_seen_ctr = 0;
  auto const n = m.size();
  for (auto j = std::size_t{0U}; j != n; ++j) {
    auto const dest_way = m.way_[j];
    if (start_component != w.r_->way_component_[dest_way]) {
      continue;
    }
    if (!should_continue && ++component_seen_ctr > 1) {
      break;
    }
    if (std::pow(m.dist_to_way_[j], 2) > limit_squared_max_matching_distance &&
        j > kBottomKDefinitelyConsidered) {
      break;
    }

    for (auto const& x : {m.left(j), m.right(j)}) {
      if (x.valid()) {
        get_best(dest_way, x);
      }
    }

    if (have_best) {
      return best_cost.cost_ < max ? std::optional{std::tuple{
                                         best, dest_way, best_node, best_cost}}
                                   : std::nullopt;
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

  b.reset({.profile_ = params,
           .w_ = &w,
           .max_ = std::max(kMinCostSettled, max),
           .dir_ = dir,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from,
           .end_loc_ = to});
  if (!b.search_bounds_valid_) {
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
              b.add_start(label, start_cost.duration_);
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
                b.add_end(label);
              });
        }
      }
      if (b.pq2_.empty()) {
        continue;
      }
      auto const should_continue = b.run();

      if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
        if (should_continue) {
          continue;
        }
        return std::nullopt;
      }

      auto const cost = b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);

      if (cost >= max) {
        return std::nullopt;
      }

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

  d.reset({.profile_ = params,
           .w_ = &w,
           .max_ = std::max(kMinCostSettled, max),
           .dir_ = dir,
           .start_time_ = start_time,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from,
           .end_loc_ = to});
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
              d.add_start({node, start_cost.cost_}, start_cost.duration_);
            });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    should_continue = d.run() && should_continue;

    auto const c = best_candidate<P>(params, w, d, to.lvl_, to_match, max, dir,
                                     start_time, should_continue, start_way,
                                     limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                            to, start_way, start_left, start_right, wc, nc,
                            node, p.cost_, dir, start_time);
    }
  }

  return std::nullopt;
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

  auto const sp = search_params<typename P::parameters>{
      .profile_ = params,
      .w_ = &w,
      .max_ = std::max(kMinCostSettled, max),
      .dir_ = dir,
      .start_time_ = start_time,
      .blocked_ = blocked,
      .sharing_ = sharing,
      .elevations_ = elevations,
      .start_loc_ = from,
      .end_loc_ = to};
  a.reset(sp);
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

    a.reset(sp);
    auto component_seen_ctr = 0;
    for (auto j = std::size_t{0U}; j != to_match.size(); ++j) {
      auto const end_way = to_match.way_[j];
      if (w.r_->way_component_[start_way] != w.r_->way_component_[end_way]) {
        continue;
      }
      if (!should_continue && ++component_seen_ctr > 1) {
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
            a.add_destination(node);
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
              a.add_start(typename P::label{node, start_cost.cost_},
                          start_cost.duration_);
            });
      }
    }

    if (a.pq_.empty()) {
      continue;
    }

    should_continue = a.run() && should_continue;

    auto const c = best_candidate<P>(params, w, a, to.lvl_, to_match, max, dir,
                                     start_time, should_continue, start_way,
                                     limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, a, from,
                            to, start_way, start_left, start_right, wc, nc,
                            node, p.cost_, dir, start_time);
    }
  }

  return std::nullopt;
}

template <Profile P>
std::optional<path> route_dijkstra_bidir(typename P::parameters const& params,
                                         ways const& w,
                                         lookup const& l,
                                         dijkstra_bidir<P>& d,
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

  auto should_continue = true;
  for (auto i = std::size_t{0}; i != from_match.size(); ++i) {
    auto const start = cch_candidate(from_match, i);
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    if (utl::none_of(to_match.way_, [&](way_idx_t const end_way) {
          return w.r_->way_component_[start.way_] ==
                 w.r_->way_component_[end_way];
        })) {
      continue;
    }

    for (auto j = std::size_t{0}; j != to_match.size(); ++j) {
      auto const end = cch_candidate(to_match, j);
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }

      // Keep one destination match per query run. This makes the
      // reconstruction use the exact end candidate that seeded the backward
      // queue instead of guessing among all destination candidates afterwards.
      d.reset(max);
      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(
              *w.r_, start.way_, nc->node_, from.lvl_, dir,
              [&](auto const node) { d.add_start(w, {node, nc->cost_}); });
        }
      }

      auto const end_way = end.way_;
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(*w.r_, end_way, nc->node_, to.lvl_,
                                opposite(dir), [&](auto const node) {
                                  d.add_destination(w, {node, nc->cost_});
                                });
        }
      }

      if (d.pqForward_.empty() || d.pqBackward_.empty()) {
        continue;
      }

      should_continue =
          d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
          should_continue;

      if (d.mu_ != kInfeasible) {
        return reconstruct_dijkstra_bidir<P>(params, w, l, blocked, sharing,
                                             elevations, d, from, to, start,
                                             end, dir);
      }
    }
  }

  return std::nullopt;
}

template <Profile P>
std::optional<path> route_cch(typename P::parameters const& params,
                              ways const& w,
                              lookup const& l,
                              cch<P>& c,
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

  auto should_continue = true;
  for (auto i = std::size_t{0}; i != from_match.size(); ++i) {
    auto const start = cch_candidate(from_match, i);
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    if (utl::none_of(to_match.way_, [&](way_idx_t const end_way) {
          return w.r_->way_component_[start.way_] ==
                 w.r_->way_component_[end_way];
        })) {
      continue;
    }

    for (auto j = std::size_t{0}; j != to_match.size(); ++j) {
      auto const end = cch_candidate(to_match, j);
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }

      // Keep one destination match per query run. This makes the
      // reconstruction use the exact end candidate that seeded the backward
      // queue instead of guessing among all destination candidates afterwards.
      c.reset(max);
      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(
              *w.r_, start.way_, nc->node_, from.lvl_, dir,
              [&](auto const node) { c.add_start(w, {node, nc->cost_}); });
        }
      }

      auto const end_way = end.way_;
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          P::resolve_start_node(*w.r_, end_way, nc->node_, to.lvl_,
                                opposite(dir), [&](auto const node) {
                                  c.add_destination(w, {node, nc->cost_});
                                });
        }
      }

      if (c.pqForward_.empty() || c.pqBackward_.empty()) {
        continue;
      }

      should_continue =
          c.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
          should_continue;

      if (c.mu_ != kInfeasible) {
        return reconstruct_cch<P>(params, w, l, blocked, sharing, elevations, c,
                                  from, to, start, end, dir);
      }
    }
  }

  return std::nullopt;
}

// Everything `reconstruct()` needs to build the path to one destination from
// a finished one-to-many search, besides the search state itself.
template <Profile P>
struct dest_candidate {
  way_idx_t dest_way_;
  candidate_node dest_nc_;
  typename P::node dest_node_;
};

template <Profile P>
struct one_to_many_state_impl final : public one_to_many_state {
  one_to_many_state_impl(std::vector<location> const& to,
                         match_view_t const& from_match)
      : to_{to}, candidates_(to.size()) {
    from_match_.start(from_match.lvl_);
    for (auto j = std::size_t{0U}; j != from_match.size(); ++j) {
      from_match_.add(from_match.dist_to_way_[j], from_match.way_[j],
                      from_match.nodes_[j]);
    }
    from_match_.finish();
  }

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
    if (!candidates_[k].has_value()) {
      return results_[k];  // direct path (from ~ to), nothing to reconstruct
    }
    auto const& c = *candidates_[k];
    auto const& sp = d_.params_;
    auto const from_match = from_match_[match_idx_t{0U}];

    // Get start node.
    // Note: this is not necessarily the candidate the search started from
    //       when this destination was settled, because if another destination
    //       required another start candidate, this destination might have
    //       been updated to a better cost from that other start candidate.
    //       -> results might differ from 1:1 search
    // TODO: remove once start/destination edges are in place
    auto root = c.dest_node_;
    while (auto const pred = d_.cost_.at(root.get_key()).pred(root)) {
      root = *pred;
    }

    // Find start candidate.
    auto const root_cost = d_.get_cost(root);
    auto const it = utl::find_if(from_match.nodes_, [&](auto const& n) {
      return is_start_candidate(n.left_, root.get_node(), root_cost) ||
             is_start_candidate(n.right_, root.get_node(), root_cost);
    });
    assert(it != end(from_match.nodes_));
    if (it == end(from_match.nodes_)) {
      return std::nullopt;
    }
    auto const start_idx =
        static_cast<std::size_t>(std::distance(begin(from_match.nodes_), it));

    return osr::reconstruct<P>(
        sp.profile_, w, l, sp.blocked_, sharing, sp.elevations_, d_,
        sp.start_loc_, to_[k], from_match.way_[start_idx],
        from_match.left(start_idx), from_match.right(start_idx), c.dest_way_,
        c.dest_nc_, c.dest_node_, results_[k]->cost_, sp.dir_, sp.start_time_);
  }

  // The search owns everything it ran with (parameters, blocked, sharing,
  // elevations, direction, start time, the start location) - see
  // `osr::search_params`.
  dijkstra<P> d_;
  std::vector<location> to_;
  match_result from_match_;
  std::vector<std::optional<dest_candidate<P>>> candidates_;
  std::vector<std::optional<path>> results_;
};

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
    std::function<bool(path const&)> const& do_reconstruct,
    std::vector<std::optional<dest_candidate<P>>>* const candidates = nullptr) {
  auto result = std::vector<std::optional<path>>{};
  result.resize(to_match.size());

  if (from_match.empty()) {
    return result;
  }

  auto const distance_lng_degrees = geo::approx_distance_lng_degrees(from.pos_);

  d.reset({.profile_ = params,
           .w_ = &w,
           .max_ = std::max(kMinCostSettled, max),
           .dir_ = dir,
           .start_time_ = start_time,
           .blocked_ = blocked,
           .sharing_ = sharing,
           .elevations_ = elevations,
           .start_loc_ = from});
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
              auto label = typename P::label{node, start_cost.cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              d.add_start(label, start_cost.duration_);
            });
      }
    }

    should_continue = d.run() && should_continue;

    auto found = 0U;
    for (auto k = std::size_t{0U}; k != result.size(); ++k) {
      auto const m =
          to_match[match_idx_t{static_cast<match_idx_t::value_t>(k)}];
      auto const& t = to[k];
      auto& r = result[k];
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t); direct.has_value()) {
        r = direct;
      } else {
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from.pos_, t.pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;
        if (std::pow(from_match.dist_to_way_[i], 2) >
                limit_squared_max_matching_distance &&
            i > kBottomKDefinitelyConsidered) {
          continue;
        }

        auto const c = best_candidate<P>(params, w, d, t.lvl_, m, max, dir,
                                         start_time, should_continue, start_way,
                                         limit_squared_max_matching_distance);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (candidates != nullptr) {
            (*candidates)[k] = dest_candidate<P>{wc, nc, n};
          }
          if (do_reconstruct(p)) {
            p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                               from, t, start_way, start_left, start_right, wc,
                               nc, n, p.cost_, dir, start_time);
            p.uses_elevator_ = true;
          }
          r = std::make_optional(p);
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

    auto b = bidirectional<P>{};
    return route_bidirectional(pp, w, l, b, from, to, from_match, to_match, max,
                               dir, blocked, sharing, elevations);
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
    std::optional<routing_time_t> const start_time) {
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::vector<std::optional<path>> {
        auto const& pp = std::get<typename P::parameters>(params);
        auto from_m = match_result{};
        l.match<P>(pp, from, false, dir, max_match_distance, blocked, from_m,
                   start_time);
        auto const from_match = from_m[match_idx_t{0U}];
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto to_match = match_result{};
        for (auto const& x : to) {
          l.match<P>(pp, x, true, dir, max_match_distance, blocked, to_match,
                     start_time);
        }
        auto d = dijkstra<P>{};
        return route(pp, w, l, d, from, to, from_match, to_match, max, dir,
                     start_time, blocked, sharing, elevations, do_reconstruct);
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

    auto d = dijkstra<P>{};
    return route_dijkstra(pp, w, l, d, from, to, from_match, to_match, max, dir,
                          start_time, blocked, sharing, elevations);
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

    auto a = astar<P>{};
    return route_astar(pp, w, l, a, from, to, from_match, to_match, max, dir,
                       start_time, blocked, sharing, elevations);
  });
}

std::optional<path> route_dijkstra_bidir(profile_parameters const& params,
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
    auto to_m = match_result{};
    l.complete_match<P>(pp, from, false, dir, max_match_distance, blocked,
                        std::nullopt, {}, from_m);
    l.complete_match<P>(pp, to, true, dir, max_match_distance, blocked,
                        std::nullopt, {}, to_m);
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_dijkstra_bidir<P>(pp, w, l, get_dijkstra_bidir<P>(), from, to,
                                   from_match, to_match, max, dir, blocked,
                                   sharing, elevations);
  });
}

std::optional<path> route_cch(profile_parameters const& params,
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
    auto to_m = match_result{};
    l.complete_match<P>(pp, from, false, dir, max_match_distance, blocked,
                        std::nullopt, {}, from_m);
    l.complete_match<P>(pp, to, true, dir, max_match_distance, blocked,
                        std::nullopt, {}, to_m);
    auto const from_match = from_m[match_idx_t{0U}];
    auto const to_match = to_m[match_idx_t{0U}];

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_cch<P>(pp, w, l, get_cch<P>(), from, to, from_match, to_match,
                        max, dir, blocked, sharing, elevations);
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
    cost_t const max,
    direction const dir,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    std::function<bool(path const&)> const& do_reconstruct,
    std::optional<routing_time_t> const start_time) {
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::unique_ptr<one_to_many_state> {
        auto s = std::make_unique<one_to_many_state_impl<P>>(to, from_match);
        if (from_match.empty()) {
          s->results_.resize(to.size());
          return s;
        }
        s->results_ = route(
            std::get<typename P::parameters>(params), w, l, s->d_, from, s->to_,
            s->from_match_[match_idx_t{0U}], to_match, max, dir, start_time,
            blocked, sharing, elevations, do_reconstruct, &s->candidates_);
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
        auto d = dijkstra<P>{};
        return route_dijkstra(std::get<typename P::parameters>(params), w, l, d,
                              from, to, from_match, to_match, max, dir,
                              start_time, blocked, sharing, elevations);
      });
    case routing_algorithm::kAStarBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        auto const& pp = std::get<typename P::parameters>(params);
        if constexpr (requires { P::kExactBidirectional; }) {
          if constexpr (!P::kExactBidirectional) {
            auto a = astar<P>{};
            return route_astar(pp, w, l, a, from, to, from_match, to_match, max,
                               dir, start_time, blocked, sharing, elevations);
          }
        }
        auto b = bidirectional<P>{};
        auto result =
            route_bidirectional(pp, w, l, b, from, to, from_match, to_match,
                                max, dir, blocked, sharing, elevations);
        if constexpr (requires(typename P::node const n) {
                        P::bidirectional_meet_cost(pp, *w.r_, n, n);
                      }) {
          if (!result.has_value()) {
            auto d = dijkstra<P>{};
            return route_dijkstra(pp, w, l, d, from, to, from_match, to_match,
                                  max, dir, start_time, blocked, sharing,
                                  elevations);
          }
        }
        return result;
      });
    case routing_algorithm::kDijkstraBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_dijkstra_bidir<P>(std::get<typename P::parameters>(params),
                                       w, l, get_dijkstra_bidir<P>(), from, to,
                                       from_match, to_match, max, dir, blocked,
                                       sharing, elevations);
      });
    case routing_algorithm::kCCH:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_cch<P>(std::get<typename P::parameters>(params), w, l,
                            get_cch<P>(), from, to, from_match, to_match, max,
                            dir, blocked, sharing, elevations);
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
                          std::optional<routing_time_t> const start_time) {
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
    case routing_algorithm::kDijkstraBi:
      return route_dijkstra_bidir(params, w, l, profile, from, to, max, dir,
                                  max_match_distance, blocked, sharing,
                                  elevations);
    case routing_algorithm::kCCH:
      return route_cch(params, w, l, profile, from, to, max, dir,
                       max_match_distance, blocked, sharing, elevations);
  }
  throw utl::fail("not implemented");
}

}  // namespace osr
