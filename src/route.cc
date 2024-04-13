#include "osr/route.h"

#include "fmt/ranges.h"

#include "utl/concat.h"
#include "utl/to_vec.h"

#include "osr/dijkstra.h"
#include "osr/infinite.h"
#include "osr/lookup.h"
#include "osr/reverse.h"
#include "osr/weight.h"
#include "utl/power_set_intersection.h"

namespace osr {

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

template <typename EdgeWeightFn>
connecting_way find_connecting_way(ways const& w,
                                   node_idx_t const from,
                                   node_idx_t const to,
                                   dist_t const expected_dist,
                                   EdgeWeightFn&& edge_weight) {
  auto const from_ways = utl::zip(w.node_ways_[from], w.node_in_way_idx_[from]);
  auto const to_ways = utl::zip(w.node_ways_[to], w.node_in_way_idx_[to]);
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
        auto const distance =
            w.way_node_dist_[a_way][is_loop ? std::max(a_idx, b_idx)
                                            : std::min(a_idx, b_idx)];
        auto const dist =
            edge_weight(w.way_properties_[a_way],
                        ((a_idx > b_idx) ^ is_loop) ? direction::kForward
                                                    : direction::kBackward,
                        distance);
        if (expected_dist == dist && (is_neighbor || is_loop)) {
          conn = {a_way, a_idx, b_idx, is_loop, distance};
        }
      },
      [](auto&& a, auto&& b) {
        auto const& [a_way, _0] = a;
        auto const& [b_way, _1] = b;
        return a_way < b_way;
      });
  utl::verify(conn.has_value(), "no connecting way found");
  return *conn;
}

template <typename EdgeWeightFn>
double add_path(ways const& w,
                node_idx_t const from,
                node_idx_t const to,
                dist_t const expected_dist,
                std::vector<path::segment>& path,
                EdgeWeightFn&& edge_weight) {
  auto const& [way, from_idx, to_idx, is_loop, distance] =
      find_connecting_way(w, from, to, expected_dist, edge_weight);
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

template <typename EdgeWeightFn>
path reconstruct(ways const& w,
                 dijkstra_state const& s,
                 way_idx_t const dest_way,
                 way_candidate const& start,
                 node_candidate const& dest,
                 EdgeWeightFn&& edge_weight) {
  (void)(edge_weight);

  auto n = dest.node_;
  auto segments = std::vector<path::segment>{
      {.polyline_ = dest.path_,
       .level_ = w.way_properties_[dest_way].get_level(),
       .way_ = way_idx_t::invalid()}};
  auto dist = 0.0;
  auto last = n;
  auto way_pos = w.get_way_pos(n, dest_way);  // TODO potentially 2x for loops?
  trace("RECONSTRUCT: dest_node={}, dest_way={}", w.node_to_osm_[n],
        w.way_osm_idx_[dest_way]);
  auto i = 0U;
  while (n != node_idx_t::invalid()) {
    ++i;
    if (i > 100) {
      abort();
    }

    last = n;
    auto const& e = s.get(n, way_pos);
    utl::verify(e.has_value(), "node={}, way={} not found", w.node_to_osm_[n],
                w.way_osm_idx_[w.node_ways_[n][way_pos]]);
    trace(
        "RECONSTRUCT: node={} [way={}, way_pos={}], pred={} [way={}, "
        "way_pos={}]",
        w.node_to_osm_[n], w.way_osm_idx_[w.node_ways_[n][way_pos]], way_pos,
        (e->pred_ == node_idx_t::invalid() ? osm_node_idx_t{0U}
                                           : w.node_to_osm_[e->pred_]),
        (e->pred_ == node_idx_t::invalid()
             ? osm_way_idx_t{0U}
             : w.way_osm_idx_[w.node_ways_[e->pred_][e->pred_way_pos_]]),
        e->pred_way_pos_);
    if (e->pred_ != node_idx_t::invalid()) {
      auto const expected_dist =
          e->dist_ - s.get(e->pred_, e->pred_way_pos_)->dist_;
      dist += add_path(w, n, e->pred_, expected_dist, segments, edge_weight);
    }
    n = e->pred_;
    way_pos = e->pred_way_pos_;
  }
  auto const& start_node =
      last == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back({.polyline_ = start_node.path_,
                      .level_ = w.way_properties_[start.way_].get_level(),
                      .way_ = way_idx_t::invalid()});
  std::reverse(begin(segments), end(segments));
  return {.time_ = static_cast<dist_t>(
              start_node.weight_ +
              s.get(n, w.get_way_pos(last, start.way_))->dist_ + dest.weight_),
          .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
          .segments_ = segments};
}

std::tuple<way_candidate const*, node_candidate const*, dist_t> best_candidate(
    ways const& w,
    match_t const& m,
    dijkstra_state const& s,
    dist_t const max_dist) {
  for (auto const& dest : m) {
    auto best_dist = std::numeric_limits<dist_t>::max();
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->dist_to_node_ < max_dist) {
        auto const target_dist =
            s.get_dist(x->node_, w.get_way_pos(x->node_, dest.way_));
        if (target_dist == kInfeasible) {
          continue;
        }

        auto const total_dist = target_dist + x->weight_;
        if (total_dist < max_dist && total_dist < best_dist) {
          best_dist = static_cast<dist_t>(total_dist);
          best = x;
        }
      }
    }

    if (best != nullptr) {
      return {&dest, best, best_dist};
    }
  }
  return {nullptr, nullptr, kInfeasible};
}

template <typename EdgeWeightFn>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          location const& from,
                          location const& to,
                          dist_t const max_dist,
                          dijkstra_state& s,
                          EdgeWeightFn&& edge_weight_fn) {
  auto const from_match = l.match(from, false, edge_weight_fn);
  auto const to_match = l.match(to, true, edge_weight_fn);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  s.reset(max_dist);

  for (auto const& start : from_match) {
    if (start.left_.valid()) {
      s.add_start(w, start.way_, start.left_.node_, start.left_.weight_);
    }
    if (start.right_.valid()) {
      s.add_start(w, start.way_, start.right_.node_, start.right_.weight_);
    }

    dijkstra<direction::kForward>(w, s, max_dist, edge_weight_fn);

    auto const [dest_way, dest_node, _] =
        best_candidate(w, to_match, s, max_dist);
    if (dest_node != nullptr) {
      return reconstruct(w, s, dest_way->way_, start, *dest_node,
                         edge_weight_fn);
    }
  }

  return std::nullopt;
}

std::optional<path> route(ways const& w,
                          lookup const& l,
                          location const& from,
                          location const& to,
                          dist_t const max_dist,
                          search_profile const profile,
                          dijkstra_state& s) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, from, to, max_dist, s, foot{});
    case search_profile::kBike:
      return route(w, l, from, to, max_dist, s, bike{});
    case search_profile::kCar: return route(w, l, from, to, max_dist, s, car{});
  }
  throw utl::fail("invalid profile");
}

template <typename EdgeWeightFn>
std::vector<std::optional<path>> route(ways const& w,
                                       lookup const& l,
                                       location const& from,
                                       std::vector<location> const& to,
                                       dist_t const max_dist,
                                       direction const dir,
                                       dijkstra_state& s,
                                       EdgeWeightFn&& edge_weight_fn) {
  auto const from_match = l.match(from, false, edge_weight_fn);
  auto const to_match = utl::to_vec(
      to, [&](auto&& x) { return l.match(x, true, edge_weight_fn); });

  auto result = std::vector<std::optional<path>>{};
  result.resize(to.size());

  if (from_match.empty()) {
    return result;
  }

  s.reset(max_dist);
  for (auto const& start : from_match) {
    if (start.left_.valid()) {
      s.add_start(w, start.way_, start.left_.node_, start.left_.weight_);
    }
    if (start.right_.valid()) {
      s.add_start(w, start.way_, start.right_.node_, start.right_.weight_);
    }

    dijkstra(w, s, max_dist, dir, edge_weight_fn);

    auto found = 0U;
    for (auto const [m, r] : utl::zip(to_match, result)) {
      if (r.has_value()) {
        ++found;
      } else {
        auto const [_, _1, time] = best_candidate(w, m, s, max_dist);
        if (time != kInfeasible) {
          r = std::make_optional(path{.time_ = time});
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

std::vector<std::optional<path>> route(ways const& w,
                                       lookup const& l,
                                       location const& from,
                                       std::vector<location> const& to,
                                       dist_t const max_dist,
                                       search_profile const profile,
                                       direction const dir,
                                       dijkstra_state& s) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, from, to, max_dist, dir, s, foot{});
    case search_profile::kBike:
      return route(w, l, from, to, max_dist, dir, s, bike{});
    case search_profile::kCar:
      return route(w, l, from, to, max_dist, dir, s, car{});
  }
  throw utl::fail("invalid profile");
}

}  // namespace osr