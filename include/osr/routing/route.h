#pragma once

#include <string_view>
#include <vector>

#include "fmt/ranges.h"

#include "utl/concat.h"
#include "utl/power_set_intersection.h"
#include "utl/to_vec.h"

#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/types.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kCar,
};

search_profile to_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  struct segment {
    std::vector<geo::latlng> polyline_;
    level_t from_level_;
    level_t to_level_;
    way_idx_t way_;
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  std::vector<segment> segments_{};
};

search_profile to_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("foot"): return search_profile::kFoot;
    case cista::hash("wheelchair"): return search_profile::kWheelchair;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("car"): return search_profile::kCar;
  }
  throw utl::fail("{} is not a valid profile", s);
}

std::string_view to_str(search_profile const p) {
  switch (p) {
    case search_profile::kFoot: return "foot";
    case search_profile::kWheelchair: return "wheelchair";
    case search_profile::kCar: return "car";
    case search_profile::kBike: return "bike";
  }
  throw utl::fail("{} is not a valid profile", static_cast<std::uint8_t>(p));
}

struct connecting_way {
  way_idx_t way_;
  std::uint16_t from_, to_;
  bool is_loop_;
  std::uint16_t distance_;
};

template <direction SearchDir, typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   ways::routing const& r,
                                   typename Profile::node const to,
                                   typename Profile::node const from,
                                   cost_t const expected_cost) {
  auto conn = std::optional<connecting_way>{};
  Profile::template adjacent<SearchDir>(
      r, from,
      [&](typename Profile::node const target, cost_t const cost,
          distance_t const dist, way_idx_t const way, std::uint16_t const a_idx,
          std::uint16_t const b_idx) {
        if (target == to && cost == expected_cost) {
          auto const is_loop = r.is_loop(way) &&
                               static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                   r.way_nodes_[way].size() - 2U;
          conn = {way, a_idx, b_idx, is_loop, dist};
        }
      });
  utl::verify(conn.has_value(), "no connecting way node/{} -> node/{} found",
              w.node_to_osm_[from.get_node()], w.node_to_osm_[to.get_node()]);
  return *conn;
}

template <typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_cost,
                                   direction const dir) {
  if (dir == direction::kForward) {
    return find_connecting_way<direction::kForward, Profile>(w, *w.r_, from, to,
                                                             expected_cost);
  } else {
    return find_connecting_way<direction::kBackward, Profile>(
        w, *w.r_, from, to, expected_cost);
  }
}

template <typename Profile>
double add_path(ways const& w,
                ways::routing const& r,
                typename Profile::node const from,
                typename Profile::node const to,
                cost_t const expected_cost,
                std::vector<path::segment>& path,
                direction const dir) {
  auto const& [way, from_idx, to_idx, is_loop, distance] =
      find_connecting_way<Profile>(w, from, to, expected_cost, dir);
  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.from_level_ = r.way_properties_[way].from_level();
  segment.to_level_ = r.way_properties_[way].to_level();

  for (auto const [osm_idx, coord] :
       infinite(reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                        (from_idx > to_idx) ^ is_loop),
                is_loop)) {
    utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U, "infinite loop");
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
  return distance;
}

template <typename Profile>
path reconstruct(ways const& w,
                 dijkstra<Profile> const& d,
                 way_candidate const& start,
                 node_candidate const& dest,
                 typename Profile::node const dest_node,
                 cost_t const cost,
                 direction const dir) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{{.polyline_ = dest.path_,
                                              .from_level_ = dest.lvl_,
                                              .to_level_ = dest.lvl_,
                                              .way_ = way_idx_t::invalid()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_dist = e.cost(n) - d.get_cost(*pred);
      dist +=
          add_path<Profile>(w, *w.r_, n, *pred, expected_dist, segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_node =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back({.polyline_ = start_node.path_,
                      .from_level_ = start_node.lvl_,
                      .to_level_ = start_node.lvl_,
                      .way_ = way_idx_t::invalid()});
  std::reverse(begin(segments), end(segments));
  return {.cost_ = cost,
          .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
          .segments_ = segments};
}

template <typename Profile>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename Profile::node,
                         cost_t>>
best_candidate(ways const& w,
               dijkstra<Profile>& d,
               level_t const lvl,
               match_t const& m,
               cost_t const max,
               direction const dir) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = typename Profile::node{};
    auto best_cost = std::numeric_limits<cost_t>::max();
    Profile::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!Profile::is_reachable(*w.r_, node, dest.way_,
                                 flip(opposite(dir), x->way_dir_),
                                 opposite(dir))) {
        return;
      }

      auto const target_cost = d.get_cost(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < max && total_cost < best_cost) {
        best_node = node;
        best_cost = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const& dest : m) {
    auto best_node = typename Profile::node{};
    auto best_cost = std::numeric_limits<cost_t>::max();
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->cost_ < max) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost < max && x_cost < best_cost) {
          best = x;
          best_node = x_node;
          best_cost = static_cast<cost_t>(x_cost);
        }
      }
    }

    if (best != nullptr) {
      return std::tuple{best, &dest, best_node, best_cost};
    }
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          dijkstra<Profile>& d,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir) {
  auto const from_match = l.match<Profile>(from, false, dir);
  auto const to_match = l.match<Profile>(to, true, dir);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  d.reset(max);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve(
            *w.r_, start.way_, nc->node_, from.lvl_,
            [&](auto const node) { d.add_start({node, nc->cost_}); });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    d.run(*w.r_, max, dir);

    auto const c = best_candidate(w, d, to.lvl_, to_match, max, dir);
    if (c.has_value()) {
      auto const [nc, wc, node, cost] = *c;
      return reconstruct<Profile>(w, d, start, *nc, node, cost, dir);
    }
  }

  return std::nullopt;
}

template <typename Profile>
std::vector<std::optional<path>> route(ways const& w,
                                       lookup const& l,
                                       dijkstra<Profile>& d,
                                       location const& from,
                                       std::vector<location> const& to,
                                       cost_t const max,
                                       direction const dir) {
  auto const from_match = l.match<Profile>(from, false, dir);
  auto const to_match =
      utl::to_vec(to, [&](auto&& x) { return l.match<Profile>(x, true, dir); });

  auto result = std::vector<std::optional<path>>{};
  result.resize(to.size());

  if (from_match.empty()) {
    return result;
  }

  d.reset(max);
  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve(
            *w.r_, start.way_, nc->node_, from.lvl_,
            [&](auto const node) { d.add_start({node, nc->cost_}); });
      }
    }

    d.run(*w.r_, max, dir);

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else {
        auto const c = best_candidate(w, d, t.lvl_, m, max, dir);
        if (c.has_value()) {
          r = std::make_optional(path{.cost_ = std::get<3>(*c)});
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

}  // namespace osr