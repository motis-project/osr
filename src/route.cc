#include "osr/routing/route.h"

#include <cstdint>
#include <algorithm>
#include <optional>

#include "boost/thread/tss.hpp"

#include "utl/concat.h"
#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/lookup.h"
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
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
    // TODO: review POC implementation
    case cista::hash("dijkstra_bidir"): return routing_algorithm::kDijkstraBi;
    case cista::hash("cch"): return routing_algorithm::kCCH;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
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
      auto const expected_cost = static_cast<cost_t>(
          e.cost(forward_n) - b.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      forward_n, expected_cost, forward_segments, dir);
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
      backward_dist += add_path<P>(params, w, *w.r_, blocked, sharing,
                                   elevations, *pred, backward_n, expected_cost,
                                   backward_segments, opposite(dir));
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
  auto p = path{.cost_ = cost,
                .dist_ = total_dist,
                .elevation_ = path_elevation,
                .segments_ = forward_segments};

  b.cost2_.at(backward_n.get_key()).write(backward_n, p);
  return p;
}

template <Profile P>
path reconstruct(typename P::parameters const& params,
                 ways const& w,
                 lookup const& l,
                 bitvec<node_idx_t> const* blocked,
                 sharing_data const* sharing,
                 elevation_storage const* elevations,
                 dijkstra<P> const& d,
                 location const& from,
                 location const& to,
                 way_candidate const& start,
                 way_candidate const& dest,
                 node_candidate const& dest_nc,
                 typename P::node const dest_node,
                 cost_t const cost,
                 direction const dir) {

  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ = l.get_node_candidate_path(dest, dest_nc,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc.lvl_,
       .to_level_ = dest_nc.lvl_,
       .from_ =
           dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .to_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc.cost_,
       .dist_ = static_cast<distance_t>(dest_nc.dist_to_node_),
       .mode_ = dest_node.get_mode()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - d.get_cost(*pred));
      dist += add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                          n, expected_cost, segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_nc =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = l.get_node_candidate_path(
           start, start_nc, dir == direction::kBackward, from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
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
                .dist_ = start_nc.dist_to_node_ + dist + dest_nc.dist_to_node_,
                .elevation_ = path_elevation,
                .segments_ = segments};
  d.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
}

template <Profile P>
// TODO: review POC implementation
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
    way_candidate const& start,
    way_candidate const& dest,
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
      auto const expected_cost =
          static_cast<cost_t>(e.cost(forward_n) -
                              d.template get_cost<direction::kForward>(*pred));
      forward_dist +=
          add_path<P>(params, w, *w.r_, blocked, sharing, elevations, *pred,
                      forward_n, expected_cost, forward_segments, dir);
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
      {.polyline_ =
           l.get_node_candidate_path(start, start_nc, dir == direction::kBackward,
                                     from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
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
          expected_cost, backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const* dest_nc = static_cast<node_candidate const*>(nullptr);
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
      {.polyline_ = l.get_node_candidate_path(dest, *dest_nc,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc->lvl_,
       .to_level_ = dest_nc->lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc->cost_,
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
                    std::uint32_t depth = 0U);

template <Profile P>
// TODO: review POC implementation
std::optional<path> reconstruct_cch(
    typename P::parameters const& params,
    ways const& w,
    lookup const& l,
    bitvec<node_idx_t> const* blocked,
    sharing_data const* sharing,
    elevation_storage const* elevations,
    cch<P> const& c,
    location const& from,
    location const& to,
    way_candidate const& start,
    way_candidate const& dest,
    direction const dir) {
  (void)blocked;
  (void)sharing;
  (void)elevations;

  if (c.meet_.get_node() == node_idx_t::invalid()) {
    return std::nullopt;
  }

  // Walk from the meeting node back to the selected start candidate through the
  // predecessor chain produced by the forward search.
  auto forward_n = c.meet_;
  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;
  while (true) {
    auto const& e = c.costForward_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(forward_n) -
                              c.template get_cost<direction::kForward>(*pred));
      forward_dist += add_cch_path<P>(params, w, *pred, forward_n,
                                      expected_cost, forward_segments, dir);
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
      {.polyline_ =
           l.get_node_candidate_path(start, start_nc, dir == direction::kBackward,
                                     from),
       .from_level_ = start_nc.lvl_,
       .to_level_ = start_nc.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_nc.cost_,
       .dist_ = static_cast<distance_t>(start_nc.dist_to_node_),
       .mode_ = forward_n.get_mode()});

  // Walk from the meeting node back to the selected destination candidate
  // through the predecessor chain produced by the backward search.
  auto backward_n = c.meet_;
  auto backward_segments = std::vector<path::segment>{};
  auto backward_dist = 0.0;
  while (true) {
    auto const& e = c.costBackward_.at(backward_n.get_key());
    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) -
                              c.template get_cost<direction::kBackward>(*pred));
      backward_dist += add_cch_path<P>(params, w, backward_n, *pred,
                                       expected_cost, backward_segments, dir);
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const* dest_nc = static_cast<node_candidate const*>(nullptr);
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
      {.polyline_ = l.get_node_candidate_path(dest, *dest_nc,
                                              dir == direction::kForward, to),
       .from_level_ = dest_nc->lvl_,
       .to_level_ = dest_nc->lvl_,
       .from_ = dir == direction::kForward ? backward_n.get_node()
                                           : node_idx_t::invalid(),
       .to_ = dir == direction::kBackward ? backward_n.get_node()
                                          : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest_nc->cost_,
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
  bool up_{};
};

template <Profile P>
cch_edge_ref find_cch_edge_ref(ways::routing const& r,
                               typename P::node const from,
                               typename P::node const to) {
  auto const up = r.node_importance_[from.get_node()] <
                  r.node_importance_[to.get_node()];
  auto const low = up ? from.get_node() : to.get_node();
  auto const high = up ? to.get_node() : from.get_node();
  for (auto const& e : r.cch_edge_weights_[low]) {
    if (e.to_ == high) {
      return {.edge_ = &e, .up_ = up};
    }
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
    if (e.edge_ != nullptr) {
      return e.up_ ? e.edge_->up_cost_ : e.edge_->down_cost_;
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

  segments.push_back(path::segment{
      .polyline_ = {w.get_node_pos(from.get_node()).as_latlng(),
                    w.get_node_pos(to.get_node()).as_latlng()},
      .from_level_ = level_t{0.F},
      .to_level_ = level_t{0.F},
      .from_ = from.get_node(),
      .to_ = to.get_node(),
      .way_ = way_idx_t::invalid(),
      .cost_ = expected_cost,
      .dist_ = *dist,
      .mode_ = to.get_mode()});
  return *dist;
}

template <Profile P>
double add_cch_path(typename P::parameters const& params,
                    ways const& w,
                    typename P::node const from,
                    typename P::node const to,
                    cost_t const expected_cost,
                    std::vector<path::segment>& segments,
                    direction const dir,
                    std::uint32_t const depth) {
  if constexpr (cch<P>::uses_customized_cost_overlay()) {
    auto const e = find_cch_edge_ref<P>(*w.r_, from, to);
    if (e.edge_ != nullptr) {
      auto const cost = e.up_ ? e.edge_->up_cost_ : e.edge_->down_cost_;
      auto const distance =
          e.up_ ? e.edge_->up_distance_ : e.edge_->down_distance_;
      auto const via =
          e.up_ ? e.edge_->up_via_ : e.edge_->down_via_;
      auto const from_way =
          e.up_ ? e.edge_->up_from_way_ : e.edge_->down_from_way_;
      auto const to_way = e.up_ ? e.edge_->up_to_way_ : e.edge_->down_to_way_;
      auto const from_dir =
          e.up_ ? e.edge_->up_from_dir_ : e.edge_->down_from_dir_;
      auto const to_dir = e.up_ ? e.edge_->up_to_dir_ : e.edge_->down_to_dir_;
      if (depth == 0U || cost == kInfeasible) {
        fmt::println(
            "cch edge depth {} node/{} -> node/{} | ranks {} -> {} | {} | "
            "cost {} | expected {} | dist {} | via node/{} | boundary {}:{} "
            "-> {}:{}",
            depth, to_idx(w.node_to_osm_[from.get_node()]),
            to_idx(w.node_to_osm_[to.get_node()]),
            w.r_->node_importance_[from.get_node()],
            w.r_->node_importance_[to.get_node()], e.up_ ? "up" : "down", cost,
            expected_cost, distance,
            via == node_idx_t::invalid() ? 0U : to_idx(w.node_to_osm_[via]),
            static_cast<unsigned>(from_way), to_str(from_dir),
            static_cast<unsigned>(to_way), to_str(to_dir));
      }
    } else {
      fmt::println("cch edge depth {} node/{} -> node/{} | missing overlay edge",
                   depth, to_idx(w.node_to_osm_[from.get_node()]),
                   to_idx(w.node_to_osm_[to.get_node()]));
    }
    auto const via_node =
        e.edge_ == nullptr
            ? node_idx_t::invalid()
            : (e.up_ ? e.edge_->up_via_ : e.edge_->down_via_);
    if (via_node != node_idx_t::invalid()) {
      // Customized CCH edges can represent a path through a lower-rank via-node,
      // even when the edge is also an original graph edge.
      auto const via = P::create_node(via_node, kNoLevel, way_pos_t{0U}, dir);
      auto const first_cost = get_cch_edge_cost<P>(params, *w.r_, from, via);
      auto const second_cost = get_cch_edge_cost<P>(params, *w.r_, via, to);
      return add_cch_path<P>(params, w, from, via, first_cost, segments, dir,
                             depth + 1U) +
             add_cch_path<P>(params, w, via, to, second_cost, segments, dir,
                             depth + 1U);
    }
  }
  if (auto const* s = find_cch_shortcut<P>(*w.r_, from, to); s != nullptr) {
    // CCH predecessor edges can be shortcuts. Recursively unpack them through
    // their contracted via-node until only original graph edges remain.
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

template <Profile P>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename P::node,
                         path>>
best_candidate(typename P::parameters const& params,
               ways const& w,
               dijkstra<P>& d,
               level_t const lvl,
               match_view_t m,
               cost_t const max,
               direction const dir,
               bool should_continue,
               way_candidate const& start,
               double const limit_squared_max_matching_distance) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    P::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!P::is_dest_reachable(params, *w.r_, node, dest.way_,
                                flip(opposite(dir), x->way_dir_), dir)) {
        return;
      }

      auto const target_cost = d.get_cost(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < best_cost.cost_) {
        best_node = node;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const [j, dest] : utl::enumerate(m)) {
    if (w.r_->way_component_[start.way_] != w.r_->way_component_[dest.way_]) {
      continue;
    }
    if (!should_continue && component_seen(w, m, j, 10)) {
      continue;
    }
    if (std::pow(dest.dist_to_way_, 2) > limit_squared_max_matching_distance &&
        j > kBottomKDefinitelyConsidered) {
      break;
    }
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid()) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost.cost_ < best_cost.cost_) {
          best = x;
          best_node = x_node;
          best_cost = x_cost;
        }
      }
    }

    if (best != nullptr) {
      return best_cost.cost_ < max
                 ? std::optional{std::tuple{best, &dest, best_node, best_cost}}
                 : std::nullopt;
    }
  }
  return std::nullopt;
}

template <Profile P>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename P::node,
                         path>>
best_candidate_bidir(typename P::parameters const& params,
                     ways const& w,
                     dijkstra_bidir<P>& d,
                     level_t const lvl,
                     match_view_t m,
                     cost_t const max,
                     direction const dir,
                     bool should_continue,
                     way_candidate const& start,
                     double const limit_squared_max_matching_distance) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    P::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!P::is_dest_reachable(params, *w.r_, node, dest.way_,
                                flip(opposite(dir), x->way_dir_), dir)) {
        return;
      }

      auto const target_cost = d.template get_cost<direction::kForward>(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < best_cost.cost_) {
        best_node = node;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const [j, dest] : utl::enumerate(m)) {
    if (w.r_->way_component_[start.way_] != w.r_->way_component_[dest.way_]) {
      continue;
    }
    if (!should_continue && component_seen(w, m, j, 10)) {
      continue;
    }
    if (std::pow(dest.dist_to_way_, 2) > limit_squared_max_matching_distance &&
        j > kBottomKDefinitelyConsidered) {
      break;
    }
    auto best_node = P::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid()) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost.cost_ < best_cost.cost_) {
          best = x;
          best_node = x_node;
          best_cost = x_cost;
        }
      }
    }

    if (best != nullptr) {
      return best_cost.cost_ < max
                 ? std::optional{std::tuple{best, &dest, best_node, best_cost}}
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
        .dist_ = dist,
        .segments_ = {path::segment{.polyline_ = {from.pos_, to.pos_},
                                    .from_level_ = from.lvl_,
                                    .to_level_ = to.lvl_,
                                    .from_ = node_idx_t::invalid(),
                                    .to_ = node_idx_t::invalid(),
                                    .way_ = way_idx_t::invalid(),
                                    .cost_ = 60U,
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
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              b.add_start(params, w, label, sharing);
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
std::optional<path> route_dijkstra(typename P::parameters const& params,
                                   ways const& w,
                                   lookup const& l,
                                   dijkstra<P>& d,
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

  auto const limit_squared_max_matching_distance =
      std::pow(geo::distance(from.pos_, to.pos_), 2) /
      kMaxMatchingDistanceSquaredRatio;

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
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir,
            [&](auto const node) { d.add_start(w, {node, nc->cost_}); });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    should_continue =
        d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
        should_continue;

    auto const c = best_candidate(params, w, d, to.lvl_, to_match, max, dir,
                                  should_continue, start,
                                  limit_squared_max_matching_distance);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<P>(params, w, l, blocked, sharing, elevations, d, from,
                            to, start, *wc, *nc, node, p.cost_, dir);
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

    for (auto const& end : to_match) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }

      // Keep one destination match per prototype run. This makes the
      // reconstruction use the exact end candidate that seeded the backward
      // queue instead of guessing among all destination candidates afterwards.
      // TODO: review POC implementation
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
          P::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) { d.add_destination(w, {node, nc->cost_}); });
        }
      }

      if (d.pqForward_.empty() || d.pqBackward_.empty()) {
        continue;
      }

      should_continue =
          d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
          should_continue;

      if (d.mu_ != kInfeasible) {
        // TODO: review POC implementation
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

    for (auto const& end : to_match) {
      if (w.r_->way_component_[start.way_] != w.r_->way_component_[end.way_]) {
        continue;
      }

      // Keep one destination match per prototype run. This makes the
      // reconstruction use the exact end candidate that seeded the backward
      // queue instead of guessing among all destination candidates afterwards.
      // TODO: review POC implementation
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
          P::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) { c.add_destination(w, {node, nc->cost_}); });
        }
      }

      if (c.pqForward_.empty() || c.pqBackward_.empty()) {
        continue;
      }

      should_continue =
          c.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
          should_continue;

      if (c.mu_ != kInfeasible) {
        // TODO: review POC implementation
        return reconstruct_cch<P>(params, w, l, blocked, sharing, elevations, c,
                                  from, to, start, end, dir);
      }
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
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const dir,
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

  d.reset(max);
  auto should_continue = true;
  for (auto const [i, start] : utl::enumerate(from_match)) {
    if (!should_continue && component_seen(w, from_match, i)) {
      continue;
    }
    auto const start_way = start.way_;
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        P::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename P::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              d.add_start(w, label);
            });
      }
    }

    should_continue =
        d.run(params, w, *w.r_, max, blocked, sharing, elevations, dir) &&
        should_continue;

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t); direct.has_value()) {
        r = direct;
      } else {
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from.pos_, t.pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;

        auto const c =
            best_candidate(params, w, d, t.lvl_, m, max, dir, should_continue,
                           start, limit_squared_max_matching_distance);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (do_reconstruct(p)) {
            p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d,
                               from, t, start, *wc, *nc, n, p.cost_, dir);
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
    std::function<bool(path const&)> const& do_reconstruct) {
  return with_profile(
      profile, [&]<Profile P>(P&&) -> std::vector<std::optional<path>> {
        auto const& pp = std::get<typename P::parameters>(params);
        auto const from_match =
            l.match<P>(pp, from, false, dir, max_match_distance, blocked);
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto const to_match = utl::to_vec(to, [&](auto&& x) {
          return l.match<P>(pp, x, true, dir, max_match_distance, blocked);
        });
        return route(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                     to_match, max, dir, blocked, sharing, elevations,
                     do_reconstruct);
      });
}

std::optional<path> route_dijkstra(profile_parameters const& params,
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

    return route_dijkstra(pp, w, l, get_dijkstra<P>(), from, to, from_match,
                          to_match, max, dir, blocked, sharing, elevations);
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
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked);

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
    auto const from_match =
        l.match<P>(pp, from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<P>(pp, to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_cch<P>(pp, w, l, get_cch<P>(), from, to, from_match, to_match,
                        max, dir, blocked, sharing, elevations);
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
    std::function<bool(path const&)> const& do_reconstruct) {
  if (from_match.empty()) {
    return std::vector<std::optional<path>>(to.size());
  }
  return with_profile(profile, [&]<Profile P>(P&&) {
    return route(std::get<typename P::parameters>(params), w, l,
                 get_dijkstra<P>(), from, to, from_match, to_match, max, dir,
                 blocked, sharing, elevations, do_reconstruct);
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
                          routing_algorithm algo) {
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }

  switch (algo) {
    case routing_algorithm::kDijkstra:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_dijkstra(std::get<typename P::parameters>(params), w, l,
                              get_dijkstra<P>(), from, to, from_match, to_match,
                              max, dir, blocked, sharing, elevations);
      });
    case routing_algorithm::kAStarBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_bidirectional(std::get<typename P::parameters>(params), w,
                                   l, get_bidirectional<P>(), from, to,
                                   from_match, to_match, max, dir, blocked,
                                   sharing, elevations);
      });
    case routing_algorithm::kDijkstraBi:
      return with_profile(profile, [&]<Profile P>(P&&) {
        return route_dijkstra_bidir<P>(
            std::get<typename P::parameters>(params), w, l,
            get_dijkstra_bidir<P>(), from, to, from_match, to_match, max, dir,
            blocked, sharing, elevations);
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
                          routing_algorithm algo) {
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing ||
      profile == search_profile::kCarParkingWheelchair ||
      profile == search_profile::kCarParking) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(params, w, l, profile, from, to, max, dir,
                            max_match_distance, blocked, sharing, elevations);
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
