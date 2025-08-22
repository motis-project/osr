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
#include "osr/routing/dijkstra.h"
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

template <typename Profile>
bidirectional<Profile>& get_bidirectional() {
  static auto s = boost::thread_specific_ptr<bidirectional<Profile>>{};
  if (s.get() == nullptr) {
    s.reset(new bidirectional<Profile>{});
  }
  return *s.get();
}

template <typename Profile>
dijkstra<Profile>& get_dijkstra() {
  static auto s = boost::thread_specific_ptr<dijkstra<Profile>>{};
  if (s.get() == nullptr) {
    s.reset(new dijkstra<Profile>{});
  }
  return *s.get();
}

struct connecting_way {
  constexpr bool valid() const { return way_ != way_idx_t::invalid(); }

  way_idx_t way_{way_idx_t::invalid()};
  std::uint16_t from_{}, to_{};
  bool is_loop_{};
  std::uint16_t distance_{};
  elevation_storage::elevation elevation_;
};

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("bidirectional"): return routing_algorithm::kAStarBi;
  }
  throw utl::fail("unknown routing algorithm: {}", s);
}

template <direction SearchDir, bool WithBlocked, typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   ways::routing const& r,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_cost, routing_parameters const rp) {
  auto conn = std::optional<connecting_way>{};
  Profile::template adjacent<SearchDir, WithBlocked>(
      r, from, blocked, sharing, elevations,
      [&](typename Profile::node const target, std::uint32_t const cost,
          distance_t const dist, way_idx_t const way, std::uint16_t const a_idx,
          std::uint16_t const b_idx,
          elevation_storage::elevation const elevation, bool) {
        if (target == to && cost == expected_cost) {
          auto const is_loop = way != way_idx_t::invalid() && r.is_loop(way) &&
                               static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                   r.way_nodes_[way].size() - 2U;
          conn = {way, a_idx, b_idx, is_loop, dist, elevation};
        }
      }, rp);
  utl::verify(
      conn.has_value(), "no connecting way node/{} -> node/{} found {} {}",
      (sharing == nullptr || from.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[from.get_node()])
          : 0,
      (sharing == nullptr || to.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[to.get_node()])
          : 0,
      expected_cost, expected_cost);
  return *conn;
}

template <typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_cost,
                                   direction const dir, routing_parameters const rp) {
  auto const call = [&]<bool WithBlocked>() {
    if (dir == direction::kForward) {
      return find_connecting_way<direction::kForward, WithBlocked, Profile>(
          w, *w.r_, blocked, sharing, elevations, from, to, expected_cost, rp);
    } else {
      return find_connecting_way<direction::kBackward, WithBlocked, Profile>(
          w, *w.r_, blocked, sharing, elevations, from, to, expected_cost, rp);
    }
  };

  if (blocked == nullptr) {
    return call.template operator()<false>();
  } else {
    return call.template operator()<true>();
  }
}

template <typename Profile>
double add_path(ways const& w,
                ways::routing const& r,
                bitvec<node_idx_t> const* blocked,
                sharing_data const* sharing,
                elevation_storage const* elevations,
                typename Profile::node const from,
                typename Profile::node const to,
                cost_t const expected_cost,
                std::vector<path::segment>& path,
                direction const dir, routing_parameters const rp) {
  auto const& [way, from_idx, to_idx, is_loop, distance, elevation] =
      find_connecting_way<Profile>(w, blocked, sharing, elevations, from, to,
                                   expected_cost, dir, rp);

  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.elevation_ = elevation;
  segment.mode_ = to.get_mode();

  if (way != way_idx_t::invalid()) {
    auto const start_idx = dir == direction::kBackward ? to_idx : from_idx;
    auto const end_idx = dir == direction::kBackward ? from_idx : to_idx;
    auto const is_reverse = (start_idx > end_idx) ^ is_loop;

    if (is_reverse) {
      segment.from_level_ = r.way_properties_[way].to_level();
      segment.to_level_ = r.way_properties_[way].from_level();
    } else {
      segment.from_level_ = r.way_properties_[way].from_level();
      segment.to_level_ = r.way_properties_[way].to_level();
    }
    segment.from_ = r.way_nodes_[way][start_idx];
    segment.to_ = r.way_nodes_[way][end_idx];

    for (auto const [osm_idx, coord] : infinite(
             reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                     is_reverse),
             is_loop)) {
      utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U,
                  "infinite loop");
      if (!active && w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
        active = true;
      }
      if (active) {
        if (w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
          // Again "from" node, then it's shorter to start from here.
          segment.polyline_.clear();
        }

        segment.polyline_.emplace_back(coord);
        if (w.node_to_osm_[r.way_nodes_[way][end_idx]] == osm_idx) {
          break;
        }
      }
    }
  } else {
    auto const get_node_pos = [&](node_idx_t const n) -> geo::latlng {
      if (n == node_idx_t::invalid()) {
        return {};
      } else if (w.is_additional_node(n)) {
        return sharing->get_additional_node_coordinates(n);
      } else {
        return w.get_node_pos(n).as_latlng();
      }
    };

    segment.from_level_ = level_t{0.0F};
    segment.to_level_ = level_t{0.0F};
    segment.from_ =
        dir == direction::kBackward ? to.get_node() : from.get_node();
    segment.to_ = dir == direction::kBackward ? from.get_node() : to.get_node();
    segment.polyline_ = {get_node_pos(segment.from_),
                         get_node_pos(segment.to_)};
  }

  return distance;
}

template <typename Profile>
path reconstruct_bi(ways const& w,
                    lookup const& l,
                    bitvec<node_idx_t> const* blocked,
                    sharing_data const* sharing,
                    elevation_storage const* elevations,
                    bidirectional<Profile> const& b,
                    location const& from,
                    location const& to,
                    way_candidate const& start,
                    way_candidate const& dest,
                    cost_t const cost,
                    direction const dir, routing_parameters const rp) {
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
          add_path<Profile>(w, *w.r_, blocked, sharing, elevations, *pred,
                            forward_n, expected_cost, forward_segments, dir, rp);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_node_candidate =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;

  forward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path<Profile>(
           start, start_node_candidate, false, from),
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
      backward_dist += add_path<Profile>(w, *w.r_, blocked, sharing, elevations,
                                         *pred, backward_n, expected_cost,
                                         backward_segments, opposite(dir), rp);
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const& dest_node_candidate =
      backward_n.get_node() == dest.left_.node_ ? dest.left_ : dest.right_;

  backward_segments.push_back(
      {.polyline_ = l.get_node_candidate_path<Profile>(
           dest, dest_node_candidate, true, to),
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

template <typename Profile>
path reconstruct(ways const& w,
                 lookup const& l,
                 bitvec<node_idx_t> const* blocked,
                 sharing_data const* sharing,
                 elevation_storage const* elevations,
                 dijkstra<Profile> const& d,
                 location const& from,
                 location const& to,
                 way_candidate const& start,
                 way_candidate const& dest,
                 node_candidate const& dest_nc,
                 typename Profile::node const dest_node,
                 cost_t const cost,
                 direction const dir, routing_parameters const rp) {

  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ = l.get_node_candidate_path<Profile>(
           dest, dest_nc, dir == direction::kForward, to),
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
      dist += add_path<Profile>(w, *w.r_, blocked, sharing, elevations, *pred,
                                n, expected_cost, segments, dir, rp);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_nc =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = l.get_node_candidate_path<Profile>(
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

template <typename Profile>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename Profile::node,
                         path>>
best_candidate(ways const& w,
               dijkstra<Profile>& d,
               level_t const lvl,
               match_view_t m,
               cost_t const max,
               direction const dir,
               bool should_continue,
               way_candidate const& start,
               double const limit_squared_max_matching_distance,
                                   routing_parameters const rp) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = Profile::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    Profile::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!Profile::is_dest_reachable(*w.r_, node, dest.way_,
                                      flip(opposite(dir), x->way_dir_), dir, rp)) {
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
    auto best_node = Profile::node::invalid();
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

template <typename Profile>
std::optional<path> route_bidirectional(ways const& w,
                                        lookup const& l,
                                        bidirectional<Profile>& b,
                                        location const& from,
                                        location const& to,
                                        match_view_t from_match,
                                        match_view_t to_match,
                                        cost_t const max,
                                        direction const dir,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations, routing_parameters const rp) {
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  b.reset(max, from, to, rp);
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
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename Profile::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              b.add_start(w, label, sharing, rp);
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
          Profile::resolve_start_node(
              *w.r_, end_way, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) {
                auto label = typename Profile::label{node, nc->cost_};
                label.track(label, *w.r_, end_way, node.get_node(), false);
                b.add_end(w, label, sharing, rp);
              });
        }
      }
      if (b.pq2_.empty()) {
        continue;
      }
      auto const should_continue =
          b.run(w, *w.r_, max, blocked, sharing, elevations, dir, rp);

      if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
        if (should_continue) {
          continue;
        }
        return std::nullopt;
      }

      auto const cost = b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);

      return reconstruct_bi(w, l, blocked, sharing, elevations, b, from, to,
                            start, end, cost, dir, rp);
    }
    b.pq1_.clear();
    b.pq2_.clear();
    b.cost2_.clear();
    b.max_reached_2_ = false;
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<path> route_dijkstra(ways const& w,
                                   lookup const& l,
                                   dijkstra<Profile>& d,
                                   location const& from,
                                   location const& to,
                                   match_view_t from_match,
                                   match_view_t to_match,
                                   cost_t const max,
                                   direction const dir,
                                   bitvec<node_idx_t> const* blocked,
                                   sharing_data const* sharing,
                                   elevation_storage const* elevations,
                                   routing_parameters const rp) {
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
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir,
            [&](auto const node) { d.add_start(w, {node, nc->cost_}, rp); });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    should_continue = d.run(w, *w.r_, max, blocked, sharing, elevations, dir, rp) &&
                      should_continue;

    auto const c =
        best_candidate(w, d, to.lvl_, to_match, max, dir, should_continue,
                       start, limit_squared_max_matching_distance, rp);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<Profile>(w, l, blocked, sharing, elevations, d, from,
                                  to, start, *wc, *nc, node, p.cost_, dir, rp);
    }
  }

  return std::nullopt;
}

template <typename Profile>
std::vector<std::optional<path>> route(
    ways const& w,
    lookup const& l,
    dijkstra<Profile>& d,
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
  routing_parameters const rp) {
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
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename Profile::label{node, nc->cost_};
              label.track(label, *w.r_, start_way, node.get_node(), false);
              d.add_start(w, label, rp);
            });
      }
    }

    should_continue = d.run(w, *w.r_, max, blocked, sharing, elevations, dir, rp) &&
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
            best_candidate(w, d, t.lvl_, m, max, dir, should_continue, start,
                           limit_squared_max_matching_distance, rp);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (do_reconstruct(p)) {
            p = reconstruct<Profile>(w, l, blocked, sharing, elevations, d,
                                     from, t, start, *wc, *nc, n, p.cost_, dir, rp);
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

std::optional<path> route_bidirectional(ways const& w,
                                        lookup const& l,
                                        search_profile const profile,
                                        location const& from,
                                        location const& to,
                                        cost_t const max,
                                        direction const dir,
                                        double const max_match_distance,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations, routing_parameters const rp) {
  return with_profile(
      profile, [&]<typename Profile>(Profile&&) -> std::optional<path> {
        auto const from_match =
            l.match<Profile>(from, false, dir, max_match_distance, blocked, rp);
        auto const to_match =
            l.match<Profile>(to, true, dir, max_match_distance, blocked, rp);

        if (from_match.empty() || to_match.empty()) {
          return std::nullopt;
        }

        return route_bidirectional(w, l, get_bidirectional<Profile>(), from, to,
                                   from_match, to_match, max, dir, blocked,
                                   sharing, elevations, rp);
      });
}

std::vector<std::optional<path>> route(
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
    std::function<bool(path const&)> const& do_reconstruct, routing_parameters const rp) {
  return with_profile(
      profile,
      [&]<typename Profile>(Profile&&) -> std::vector<std::optional<path>> {
        auto const from_match =
            l.match<Profile>(from, false, dir, max_match_distance, blocked, rp);
        if (from_match.empty()) {
          return std::vector<std::optional<path>>(to.size());
        }
        auto const to_match = utl::to_vec(to, [&](auto&& x) {
          return l.match<Profile>(x, true, dir, max_match_distance, blocked, rp);
        });
        return route(w, l, get_dijkstra<Profile>(), from, to, from_match,
                     to_match, max, dir, blocked, sharing, elevations,
                     do_reconstruct, rp);
      });
}

std::optional<path> route_dijkstra(ways const& w,
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
                          routing_parameters const rp) {
  return with_profile(
      profile, [&]<typename Profile>(Profile&&) -> std::optional<path> {
        auto const from_match =
            l.match<Profile>(from, false, dir, max_match_distance, blocked, rp);
        auto const to_match =
            l.match<Profile>(to, true, dir, max_match_distance, blocked, rp);

        if (from_match.empty() || to_match.empty()) {
          return std::nullopt;
        }

        return route_dijkstra(w, l, get_dijkstra<Profile>(), from, to,
                              from_match, to_match, max, dir, blocked, sharing,
                              elevations, rp);
      });
}

std::vector<std::optional<path>> route(
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
    std::function<bool(path const&)> const& do_reconstruct, routing_parameters const rp) {
  if (from_match.empty()) {
    return std::vector<std::optional<path>>(to.size());
  }
  return with_profile(profile, [&]<typename Profile>(Profile&&) {
    return route(w, l, get_dijkstra<Profile>(), from, to, from_match, to_match,
                 max, dir, blocked, sharing, elevations, do_reconstruct, rp);
  });
}

std::optional<path> route(ways const& w,
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
                          routing_algorithm algo, routing_parameters const rp) {
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }

  switch (algo) {
    case routing_algorithm::kDijkstra:
      return with_profile(profile, [&]<typename Profile>(Profile&&) {
        return route_dijkstra(w, l, get_dijkstra<Profile>(), from, to,
                              from_match, to_match, max, dir, blocked, sharing,
                              elevations, rp);
      });
    case routing_algorithm::kAStarBi:
      return with_profile(profile, [&]<typename Profile>(Profile&&) {
        return route_bidirectional(w, l, get_bidirectional<Profile>(), from, to,
                                   from_match, to_match, max, dir, blocked,
                                   sharing, elevations, rp);
      });
  }
  throw utl::fail("not implemented");
}

std::optional<path> route(ways const& w,
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
                          routing_parameters const rp) {
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing ||
      profile == search_profile::kCarParkingWheelchair ||
      profile == search_profile::kCarParking) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(w, l, profile, from, to, max, dir,
                            max_match_distance, blocked, sharing, elevations, rp);
    case routing_algorithm::kAStarBi:
      return route_bidirectional(w, l, profile, from, to, max, dir,
                                 max_match_distance, blocked, sharing,
                                 elevations, rp);
  }
  throw utl::fail("not implemented");
}

}  // namespace osr
