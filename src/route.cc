#include "osr/routing/route.h"

#include <cstdint>
#include <algorithm>
#include <optional>

#include "boost/thread/tss.hpp"

#include "utl/concat.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

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
    case cista::hash("a_star"): return routing_algorithm::kAStar;
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
                                   cost_t const expected_cost) {
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
      });
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
                                   direction const dir) {
  auto const call = [&]<bool WithBlocked>() {
    if (dir == direction::kForward) {
      return find_connecting_way<direction::kForward, WithBlocked, Profile>(
          w, *w.r_, blocked, sharing, elevations, from, to, expected_cost);
    } else {
      return find_connecting_way<direction::kBackward, WithBlocked, Profile>(
          w, *w.r_, blocked, sharing, elevations, from, to, expected_cost);
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
                direction const dir) {
  auto const& [way, from_idx, to_idx, is_loop, distance, elevation] =
      find_connecting_way<Profile>(w, blocked, sharing, elevations, from, to,
                                   expected_cost, dir);

  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.elevation_ = elevation;
  segment.mode_ = to.get_mode();

  if (way != way_idx_t::invalid()) {
    segment.from_level_ = r.way_properties_[way].from_level();
    segment.to_level_ = r.way_properties_[way].to_level();
    segment.from_ = r.way_nodes_[way][from_idx];
    segment.to_ = r.way_nodes_[way][to_idx];

    for (auto const [osm_idx, coord] : infinite(
             reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                     (from_idx > to_idx) ^ is_loop),
             is_loop)) {
      utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U,
                  "infinite loop");
      if (!active && w.node_to_osm_[r.way_nodes_[way][from_idx]] == osm_idx) {
        active = true;
      }
      if (active) {
        if (w.node_to_osm_[r.way_nodes_[way][from_idx]] == osm_idx) {
          // Again "from" node, then it's shorter to start from here.
          segment.polyline_.clear();
        }

        segment.polyline_.emplace_back(coord);
        if (w.node_to_osm_[r.way_nodes_[way][to_idx]] == osm_idx) {
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
    segment.from_ = from.get_node();
    segment.to_ = to.get_node();
    segment.polyline_ = {get_node_pos(segment.from_),
                         get_node_pos(segment.to_)};
  }

  return distance;
}

template <typename Profile>
path reconstruct_bi(ways const& w,
                    bitvec<node_idx_t> const* blocked,
                    sharing_data const* sharing,
                    elevation_storage const* elevations,
                    bidirectional<Profile> const& b,
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
      auto const expected_cost =
          static_cast<cost_t>(e.cost(forward_n) - b.get_cost_from_start(*pred));
      forward_dist +=
          add_path<Profile>(w, *w.r_, blocked, sharing, elevations, *pred,
                            forward_n, expected_cost, forward_segments, dir);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_node_candidate =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;

  forward_segments.push_back(
      {.polyline_ = start_node_candidate.path_,
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
          static_cast<cost_t>(e.cost(backward_n) - b.get_cost_from_end(*pred));
      backward_dist += add_path<Profile>(w, *w.r_, blocked, nullptr, elevations,
                                         *pred, backward_n, expected_cost,
                                         backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const& dest_node_candidate =
      backward_n.get_node() == dest.left_.node_ ? dest.left_ : dest.right_;

  backward_segments.push_back(
      {.polyline_ = dest_node_candidate.path_,
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

  std::reverse(forward_segments.begin(), forward_segments.end());
  forward_segments.insert(forward_segments.end(), backward_segments.begin(),
                          backward_segments.end());

  auto total_dist = start_node_candidate.dist_to_node_ + forward_dist +
                    backward_dist + dest_node_candidate.dist_to_node_;

  auto p =
      path{.cost_ = cost, .dist_ = total_dist, .segments_ = forward_segments};

  b.cost2_.at(backward_n.get_key()).write(backward_n, p);
  return p;
}

template <typename Profile>
path reconstruct(ways const& w,
                 bitvec<node_idx_t> const* blocked,
                 sharing_data const* sharing,
                 elevation_storage const* elevations,
                 dijkstra<Profile> const& d,
                 way_candidate const& start,
                 node_candidate const& dest,
                 typename Profile::node const dest_node,
                 cost_t const cost,
                 direction const dir) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ = dest.path_,
       .from_level_ = dest.lvl_,
       .to_level_ = dest.lvl_,
       .from_ = node_idx_t::invalid(),
       .to_ = node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = dest.cost_,
       .dist_ = static_cast<distance_t>(dest.dist_to_node_),
       .mode_ = dest_node.get_mode()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - d.get_cost(*pred));
      dist += add_path<Profile>(w, *w.r_, blocked, sharing, elevations, *pred,
                                n, expected_cost, segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_node =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = start_node.path_,
       .from_level_ = start_node.lvl_,
       .to_level_ = start_node.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = start_node.cost_,
       .dist_ = static_cast<distance_t>(start_node.dist_to_node_),
       .mode_ = n.get_mode()});
  std::reverse(begin(segments), end(segments));
  auto path_elevation = elevation_storage::elevation{};
  for (auto const& segment : segments) {
    path_elevation += segment.elevation_;
  }
  auto p = path{.cost_ = cost,
                .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
                .elevation_ = path_elevation,
                .segments_ = segments};
  d.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
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
               direction const dir) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = Profile::node::invalid();
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    Profile::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!Profile::is_dest_reachable(*w.r_, node, dest.way_,
                                      flip(opposite(dir), x->way_dir_),
                                      opposite(dir))) {
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

  auto checked = 0U;
  for (auto const& dest : m) {
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

    ++checked;
    if (best != nullptr) {
      return best_cost.cost_ < max
                 ? std::optional{std::tuple{best, &dest, best_node, best_cost}}
                 : std::nullopt;
    } else if (checked == 10U) {
      break;
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
                                        bidirectional<Profile>& b,
                                        location const& from,
                                        location const& to,
                                        match_view_t from_match,
                                        match_view_t to_match,
                                        cost_t const max,
                                        direction const dir,
                                        bitvec<node_idx_t> const* blocked,
                                        sharing_data const* sharing,
                                        elevation_storage const* elevations) {
  std::cout << "using bidir" << std::endl;
  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  b.reset(max, from, to);
  if (b.radius_ == max) {
    return std::nullopt;
  }
  auto init = true;
  for (auto const& start : from_match) {
    for (auto const& end : to_match) {
      for (auto const* nc : {&start.left_, &start.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          Profile::resolve_start_node(
              *w.r_, start.way_, nc->node_, from.lvl_, dir,
              [&](auto const node) { b.add_start(w, {node, nc->cost_}); });
        }
      }
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          Profile::resolve_start_node(
              *w.r_, end.way_, nc->node_, to.lvl_, opposite(dir),
              [&](auto const node) { b.add_end(w, {node, nc->cost_}); });
        }
      }
      if ((b.pq1_.empty() && b.pq2_.empty()) ||
          (init && (b.pq1_.empty() || b.pq2_.empty()))) {
        continue;
      }
      b.clear_mp();
      init = false;

      b.run(w, *w.r_, max, blocked, sharing, elevations, dir);

      if (b.meet_point_1_.get_node() == node_idx_t::invalid()) {
        continue;
      }

      auto const cost = b.get_cost_to_mp(b.meet_point_1_, b.meet_point_2_);

      return reconstruct_bi(w, blocked, sharing, elevations, b, start, end,
                            cost, dir);
    }
    b.pq2_.clear();
    b.pq2_.n_buckets(max + 1U);
    b.cost2_.clear();
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<path> route_dijkstra(ways const& w,
                                   dijkstra<Profile>& d,
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

  d.reset(max);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir,
            [&](auto const node) { d.add_start(w, {node, nc->cost_}); });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    d.run(w, *w.r_, max, blocked, sharing, elevations, dir);

    auto const c = best_candidate(w, d, to.lvl_, to_match, max, dir);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<Profile>(w, blocked, sharing, elevations, d, start,
                                  *nc, node, p.cost_, dir);
    }
  }

  return std::nullopt;
}

template <typename Profile>
std::vector<std::optional<path>> route(
    ways const& w,
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
    std::function<bool(path const&)> const& do_reconstruct) {
  auto result = std::vector<std::optional<path>>{};
  result.resize(to_match.size());

  if (from_match.empty()) {
    return result;
  }

  d.reset(max);
  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename Profile::label{node, nc->cost_};
              label.track(label, *w.r_, start.way_, node.get_node(), false);
              d.add_start(w, label);
            });
      }
    }

    d.run(w, *w.r_, max, blocked, sharing, elevations, dir);

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t); direct.has_value()) {
        r = direct;
      } else {
        auto const c = best_candidate(w, d, t.lvl_, m, max, dir);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (do_reconstruct(p)) {
            p = reconstruct<Profile>(w, blocked, sharing, elevations, d, start,
                                     *nc, n, p.cost_, dir);
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
                                        elevation_storage const* elevations) {
  auto const r =
      [&]<typename Profile>(bidirectional<Profile>& b) -> std::optional<path> {
    auto const from_match =
        l.match<Profile>(from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<Profile>(to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_bidirectional(w, b, from, to, from_match, to_match, max, dir,
                               blocked, sharing, elevations);
  };

  switch (profile) {
    case search_profile::kFoot:
      return r(get_bidirectional<foot<false, elevator_tracking>>());
    case search_profile::kWheelchair:
      return r(get_bidirectional<foot<true, elevator_tracking>>());
    case search_profile::kBike:
      return r(get_bidirectional<bike<kElevationNoCost>>());
    case search_profile::kBikeElevationLow:
      return r(get_bidirectional<bike<kElevationLowCost>>());
    case search_profile::kBikeElevationHigh:
      return r(get_bidirectional<bike<kElevationHighCost>>());
    case search_profile::kCar: return r(get_bidirectional<car>());
    case search_profile::kCarParking:
      return r(get_bidirectional<car_parking<false>>());
    case search_profile::kCarParkingWheelchair:
      return r(get_bidirectional<car_parking<true>>());
    case search_profile::kBikeSharing:
      return r(get_bidirectional<bike_sharing>());
    case search_profile::kCarSharing:
      return r(get_bidirectional<car_sharing<track_node_tracking>>());
  }
  throw utl::fail("not implemented");
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
    std::function<bool(path const&)> const& do_reconstruct) {
  auto const r = [&]<typename Profile>(
                     dijkstra<Profile>& d) -> std::vector<std::optional<path>> {
    auto const from_match =
        l.match<Profile>(from, false, dir, max_match_distance, blocked);
    if (from_match.empty()) {
      return std::vector<std::optional<path>>(to.size());
    }
    auto const to_match = utl::to_vec(to, [&](auto&& x) {
      return l.match<Profile>(x, true, dir, max_match_distance, blocked);
    });
    return route(w, d, from, to, from_match, to_match, max, dir, blocked,
                 sharing, elevations, do_reconstruct);
  };

  switch (profile) {
    case search_profile::kFoot:
      return r(get_dijkstra<foot<false, elevator_tracking>>());
    case search_profile::kWheelchair:
      return r(get_dijkstra<foot<true, elevator_tracking>>());
    case search_profile::kBike:
      return r(get_dijkstra<bike<kElevationNoCost>>());
    case search_profile::kBikeElevationLow:
      return r(get_dijkstra<bike<kElevationLowCost>>());
    case search_profile::kBikeElevationHigh:
      return r(get_dijkstra<bike<kElevationHighCost>>());
    case search_profile::kCar: return r(get_dijkstra<car>());
    case search_profile::kCarParking:
      return r(get_dijkstra<car_parking<false>>());
    case search_profile::kCarParkingWheelchair:
      return r(get_dijkstra<car_parking<true>>());
    case search_profile::kBikeSharing: return r(get_dijkstra<bike_sharing>());
    case search_profile::kCarSharing:
      return r(get_dijkstra<car_sharing<track_node_tracking>>());
  }

  throw utl::fail("not implemented");
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
                                   elevation_storage const* elevations) {
  auto const r =
      [&]<typename Profile>(dijkstra<Profile>& d) -> std::optional<path> {
    auto const from_match =
        l.match<Profile>(from, false, dir, max_match_distance, blocked);
    auto const to_match =
        l.match<Profile>(to, true, dir, max_match_distance, blocked);

    if (from_match.empty() || to_match.empty()) {
      return std::nullopt;
    }

    return route_dijkstra(w, d, from, to, from_match, to_match, max, dir,
                          blocked, sharing, elevations);
  };

  switch (profile) {
    case search_profile::kFoot:
      return r(get_dijkstra<foot<false, elevator_tracking>>());
    case search_profile::kWheelchair:
      return r(get_dijkstra<foot<true, elevator_tracking>>());
    case search_profile::kBike:
      return r(get_dijkstra<bike<kElevationNoCost>>());
    case search_profile::kBikeElevationLow:
      return r(get_dijkstra<bike<kElevationLowCost>>());
    case search_profile::kBikeElevationHigh:
      return r(get_dijkstra<bike<kElevationHighCost>>());
    case search_profile::kCar: return r(get_dijkstra<car>());
    case search_profile::kCarParking:
      return r(get_dijkstra<car_parking<false>>());
    case search_profile::kCarParkingWheelchair:
      return r(get_dijkstra<car_parking<true>>());
    case search_profile::kBikeSharing: return r(get_dijkstra<bike_sharing>());
    case search_profile::kCarSharing:
      return r(get_dijkstra<car_sharing<track_node_tracking>>());
  }

  throw utl::fail("not implemented");
}

std::vector<std::optional<path>> route(
    ways const& w,
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

  auto const r = [&]<typename Profile>(
                     dijkstra<Profile>& d) -> std::vector<std::optional<path>> {
    return route(w, d, from, to, from_match, to_match, max, dir, blocked,
                 sharing, elevations, do_reconstruct);
  };

  switch (profile) {
    case search_profile::kFoot:
      return r(get_dijkstra<foot<false, elevator_tracking>>());
    case search_profile::kWheelchair:
      return r(get_dijkstra<foot<true, elevator_tracking>>());
    case search_profile::kBike:
      return r(get_dijkstra<bike<kElevationNoCost>>());
    case search_profile::kBikeElevationLow:
      return r(get_dijkstra<bike<kElevationLowCost>>());
    case search_profile::kBikeElevationHigh:
      return r(get_dijkstra<bike<kElevationHighCost>>());
    case search_profile::kCar: return r(get_dijkstra<car>());
    case search_profile::kCarParking:
      return r(get_dijkstra<car_parking<false>>());
    case search_profile::kCarParkingWheelchair:
      return r(get_dijkstra<car_parking<true>>());
    case search_profile::kBikeSharing: return r(get_dijkstra<bike_sharing>());
    case search_profile::kCarSharing:
      return r(get_dijkstra<car_sharing<track_node_tracking>>());
  }

  throw utl::fail("not implemented");
}

std::optional<path> route(ways const& w,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations) {
  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  auto const r =
      [&]<typename Profile>(dijkstra<Profile>& d) -> std::optional<path> {
    return route_dijkstra(w, d, from, to, from_match, to_match, max, dir,
                          blocked, sharing, elevations);
  };

  switch (profile) {
    case search_profile::kFoot:
      return r(get_dijkstra<foot<false, elevator_tracking>>());
    case search_profile::kWheelchair:
      return r(get_dijkstra<foot<true, elevator_tracking>>());
    case search_profile::kBike:
      return r(get_dijkstra<bike<kElevationNoCost>>());
    case search_profile::kBikeElevationLow:
      return r(get_dijkstra<bike<kElevationLowCost>>());
    case search_profile::kBikeElevationHigh:
      return r(get_dijkstra<bike<kElevationHighCost>>());
    case search_profile::kCar: return r(get_dijkstra<car>());
    case search_profile::kCarParking:
      return r(get_dijkstra<car_parking<false>>());
    case search_profile::kCarParkingWheelchair:
      return r(get_dijkstra<car_parking<true>>());
    case search_profile::kBikeSharing: return r(get_dijkstra<bike_sharing>());
    case search_profile::kCarSharing:
      return r(get_dijkstra<car_sharing<track_node_tracking>>());
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
                          routing_algorithm algo) {
  if (profile == search_profile::kBikeSharing ||
      profile == search_profile::kCarSharing) {
    algo = routing_algorithm::kDijkstra;  // TODO
  }
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(w, l, profile, from, to, max, dir,
                            max_match_distance, blocked, sharing, elevations);
    case routing_algorithm::kAStarBi:
      return route_bidirectional(w, l, profile, from, to, max, dir,
                                 max_match_distance, blocked, sharing,
                                 elevations);
    // placeholder for future AStar implementation
    case routing_algorithm::kAStar:
      return route_bidirectional(w, l, profile, from, to, max, dir,
                                 max_match_distance, blocked, sharing,
                                 elevations);
  }
  throw utl::fail("not implemented");
}

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

template bidirectional<foot<true, osr::noop_tracking>>&
get_bidirectional<foot<true, osr::noop_tracking>>();

template bidirectional<foot<false, osr::noop_tracking>>&
get_bidirectional<foot<false, osr::noop_tracking>>();

template dijkstra<foot<true, osr::noop_tracking>>&
get_dijkstra<foot<true, osr::noop_tracking>>();

template dijkstra<foot<false, osr::noop_tracking>>&
get_dijkstra<foot<false, osr::noop_tracking>>();

}  // namespace osr
