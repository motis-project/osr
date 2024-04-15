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
  kBike,
  kCar,
};

search_profile to_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  struct segment {
    std::vector<geo::latlng> polyline_;
    level_t level_;
    way_idx_t way_;
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  std::vector<segment> segments_{};
};

search_profile to_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("foot"): return search_profile::kFoot;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("car"): return search_profile::kCar;
  }
  throw utl::fail("{} is not a valid profile", s);
}

std::string_view to_str(search_profile const p) {
  switch (p) {
    case search_profile::kFoot: return "pedestrian";
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

template <typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_dist) {
  auto const from_ways = utl::zip(w.node_ways_[from.get_node()],
                                  w.node_in_way_idx_[from.get_node()]);
  auto const to_ways =
      utl::zip(w.node_ways_[to.get_node()], w.node_in_way_idx_[to.get_node()]);
  auto conn = std::optional<connecting_way>{};
  utl::power_set_intersection(
      from_ways, to_ways,
      [&](auto&& a, auto&& b) {
        auto const& [a_way, a_idx] = a;
        auto const& [b_way, b_idx] = b;
        auto const is_loop = w.is_loop(a_way) &&
                             static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                 w.way_nodes_[a_way].size() - 2U;
        auto const is_neighbor = std::abs(a_idx - b_idx) == 1;
        auto const distance = static_cast<cost_t>(
            w.way_node_dist_[a_way][is_loop ? (std::max(a_idx, b_idx) %
                                               w.way_node_dist_[a_way].size())
                                            : std::min(a_idx, b_idx)]);
        auto const dist =
            Profile::way_cost(w.way_properties_[a_way],
                              ((a_idx > b_idx) ^ is_loop)
                                  ? direction::kForward
                                  : direction::kBackward,
                              distance) +
            Profile::node_cost(w.node_properties_[from.get_node()]);
        if (expected_dist == dist && (is_neighbor || is_loop)) {
          conn = {a_way, a_idx, b_idx, is_loop, distance};
          return;
        }
      },
      [](auto&& a, auto&& b) {
        auto const& [a_way, _0] = a;
        auto const& [b_way, _1] = b;
        return a_way < b_way;
      });
  utl::verify(conn.has_value(), "no connecting way {} -> {} found",
              w.node_to_osm_[from.get_node()], w.node_to_osm_[to.get_node()]);
  return *conn;
}

template <typename Profile>
double add_path(ways const& w,
                typename Profile::node const from,
                typename Profile::node const to,
                cost_t const expected_cost,
                std::vector<path::segment>& path) {
  auto const& [way, from_idx, to_idx, is_loop, distance] =
      find_connecting_way<Profile>(w, from, to, expected_cost);
  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.level_ = w.way_properties_[way].get_level();
  for (auto const [osm_idx, coord] :
       infinite(reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                        (from_idx > to_idx) ^ is_loop),
                is_loop)) {
    utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U, "infinite loop");
    if (!active && w.node_to_osm_[w.way_nodes_[way][from_idx]] == osm_idx) {
      active = true;
    }
    if (active) {
      segment.polyline_.emplace_back(coord);
      if (w.node_to_osm_[w.way_nodes_[way][to_idx]] == osm_idx) {
        break;
      }
    }
  }
  std::reverse(begin(segment.polyline_), end(segment.polyline_));
  return distance;
}

template <typename Profile>
path reconstruct(ways const& w,
                 dijkstra<Profile> const& d,
                 way_candidate const& start,
                 node_candidate const& dest,
                 way_idx_t const dest_way,
                 typename Profile::node const dest_node,
                 cost_t const cost) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{
      {.polyline_ = dest.path_,
       .level_ = w.way_properties_[dest_way].get_level(),
       .way_ = way_idx_t::invalid()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n);
    auto const pred = e.pred();
    if (pred.has_value()) {
      auto const expected_dist = e.cost() - d.get_cost(*pred);
      dist += add_path<Profile>(w, n, *pred, expected_dist, segments);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_node =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back({.polyline_ = start_node.path_,
                      .level_ = w.way_properties_[start.way_].get_level(),
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
               match_t const& m,
               cost_t const max) {
  for (auto const& dest : m) {
    auto best_node = typename Profile::node{};
    auto best_dist = std::numeric_limits<cost_t>::max();
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->cost_ < max) {
        Profile::resolve(w, dest.way_, x->node_,
                         w.way_properties_[dest.way_].get_level(),
                         [&](auto&& node) {
                           auto const target_cost = d.get_cost(node);
                           if (target_cost == kInfeasible) {
                             return;
                           }

                           auto const total_dist = target_cost + x->cost_;
                           if (total_dist < max && total_dist < best_dist) {
                             best = x;
                             best_node = node;
                             best_dist = static_cast<cost_t>(total_dist);
                           }
                         });
      }
    }
    if (best != nullptr) {
      return std::tuple{best, &dest, best_node, best_dist};
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
  auto const from_match = l.match<Profile>(from, false);
  auto const to_match = l.match<Profile>(to, true);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  d.reset(max);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve(
            w, start.way_, nc->node_, from.lvl_,
            [&](auto const node) { d.add_start({node, nc->cost_}); });
      }
    }

    d.run(w, max, dir);

    auto const c = best_candidate(w, d, to_match, max);
    if (c.has_value()) {
      auto const [nc, wc, node, cost] = *c;
      return reconstruct<Profile>(w, d, start, *nc, wc->way_, node, cost);
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
                                       direction const dir,
                                       Profile&& profile) {
  auto const from_match = l.match(from, false, profile);
  auto const to_match =
      utl::to_vec(to, [&](auto&& x) { return l.match(x, true, profile); });

  auto result = std::vector<std::optional<path>>{};
  result.resize(to.size());

  if (from_match.empty()) {
    return result;
  }

  d.reset(max);
  for (auto const& start : from_match) {
    if (start.left_.valid()) {
      d.add_start(w, start.way_, start.left_.node_, start.left_.cost_);
    }
    if (start.right_.valid()) {
      d.add_start(w, start.way_, start.right_.node_, start.right_.cost_);
    }

    d.run(w, max, dir);

    auto found = 0U;
    for (auto const [m, r] : utl::zip(to_match, result)) {
      if (r.has_value()) {
        ++found;
      } else {
        auto const [_, _1, _2, cost] = best_candidate(w, m, d, max);
        if (cost != kInfeasible) {
          r = std::make_optional(path{.cost_ = cost});
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