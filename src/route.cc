#include "osr/route.h"

#include "fmt/ranges.h"

#include "utl/concat.h"
#include "utl/to_vec.h"

#include "osr/dijkstra.h"
#include "osr/infinite.h"
#include "osr/lookup.h"
#include "osr/reverse.h"
#include "osr/weight.h"

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
                                   dijkstra_state const& s,
                                   node_idx_t const from,
                                   node_idx_t const to,
                                   EdgeWeightFn&& edge_weight) {
  auto const from_ways = utl::zip(w.node_ways_[from], w.node_in_way_idx_[from]);
  auto const to_ways = utl::zip(w.node_ways_[to], w.node_in_way_idx_[to]);
  auto const expected_dist = s.get_dist(from) - s.get_dist(to);
  auto a = begin(from_ways), b = begin(to_ways);
  while (a != end(from_ways) && b != end(to_ways)) {
    auto const& [a_way, a_idx] = *a;
    auto const& [b_way, b_idx] = *b;

    if (a_way < b_way) {
      ++a;
    } else if (b_way < a_way) {
      ++b;
    } else {
      auto const is_loop =
          w.is_loop(a_way) && static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
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

      auto const print_debug = [&]() {
        [[unlikely]];
        fmt::println(
            "no match: way={}, n={}, a_idx={} [osm={}], b_idx={} [osm={}], "
            "dist_match={} "
            "[expected={}, actual={}], is_neighbor={}, "
            "is_loop={}",
            w.way_osm_idx_[a_way], w.way_nodes_[a_way].size(), a_idx,
            w.node_to_osm_[w.way_nodes_[a_way][a_idx]], b_idx,
            w.node_to_osm_[w.way_nodes_[b_way][b_idx]], (expected_dist == dist),
            expected_dist, dist, is_neighbor, is_loop);
      };

      if (expected_dist == dist && (is_neighbor || is_loop)) {
        if (expected_dist != dist) {
          print_debug();
        }

        return {a_way, a_idx, b_idx, is_loop, distance};
      } else {
        print_debug();
      }

      ++a;
      ++b;
    }
  }

  fmt::println("from={}, to={}", w.node_to_osm_[from], w.node_to_osm_[to]);
  fmt::println("from ways:");
  for (auto const& [from_way, from_idx] : from_ways) {
    fmt::println(" {} {}", w.way_osm_idx_[from_way], from_idx);
  }
  fmt::println("to ways:");
  for (auto const& [to_way, to_idx] : to_ways) {
    fmt::println(" {} {}", w.way_osm_idx_[to_way], to_idx);
  }

  throw utl::fail("no connecting way found");
}

template <typename EdgeWeightFn>
double add_path(ways const& w,
                dijkstra_state const& s,
                node_idx_t const from,
                node_idx_t const to,
                std::vector<geo::latlng>& path,
                EdgeWeightFn&& edge_weight) {
  auto const& [way, from_idx, to_idx, is_loop, distance] =
      find_connecting_way(w, s, from, to, edge_weight);
  auto j = 0U;
  auto active = false;
  for (auto const [osm_idx, coord] :
       infinite(reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                        (from_idx > to_idx) ^ is_loop),
                is_loop)) {
    utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U, "infinite loop");
    if (!active && w.node_to_osm_[w.way_nodes_[way][from_idx]] == osm_idx) {
      active = true;
    }
    if (active) {
      path.emplace_back(coord);
      if (w.node_to_osm_[w.way_nodes_[way][to_idx]] == osm_idx) {
        break;
      }
    }
  }
  return distance;
}

template <typename EdgeWeightFn>
path reconstruct(ways const& w,
                 dijkstra_state const& s,
                 way_candidate const& start,
                 node_candidate const& dest,
                 EdgeWeightFn&& edge_weight) {
  auto n = dest.node_;
  auto polyline = dest.path_;
  auto dist = 0.0;
  auto last = n;
  while (n != node_idx_t::invalid()) {
    last = n;
    auto const& e = s.dist_.at(n);
    if (n == e.pred_) {
      abort();
    }
    if (e.pred_ != node_idx_t::invalid()) {
      dist += add_path(w, s, n, e.pred_, polyline, edge_weight);
    }
    n = e.pred_;
  }
  auto const& start_node =
      last == start.left_.node_ ? start.left_ : start.right_;
  utl::concat(polyline, start_node.path_);
  std::reverse(begin(polyline), end(polyline));
  return {.time_ = static_cast<dist_t>(start_node.weight_ + s.get_dist(n) +
                                       dest.weight_),
          .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
          .polyline_ = std::move(polyline)};
}

std::pair<node_candidate const*, dist_t> best_candidate(match_t const& m,
                                                        dijkstra_state const& s,
                                                        dist_t const max_dist) {
  for (auto const& dest : m) {
    auto best_dist = std::numeric_limits<dist_t>::max();
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->dist_to_node_ < max_dist) {
        auto const target_dist = s.get_dist(x->node_);
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
      return {best, best_dist};
    }
  }
  return {nullptr, kInfeasible};
}

template <typename EdgeWeightFn>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
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
      s.add_start(start.left_.node_, start.left_.weight_);
    }
    if (start.right_.valid()) {
      s.add_start(start.right_.node_, start.right_.weight_);
    }

    dijkstra<direction::kForward>(w, s, max_dist, edge_weight_fn);

    auto const [best, _] = best_candidate(to_match, s, max_dist);
    if (best != nullptr) {
      return reconstruct(w, s, start, *best, edge_weight_fn);
    }
  }

  return std::nullopt;
}

std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
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
                                       geo::latlng const& from,
                                       std::vector<geo::latlng> const& to,
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
      s.add_start(start.left_.node_, start.left_.weight_);
    }
    if (start.right_.valid()) {
      s.add_start(start.right_.node_, start.right_.weight_);
    }

    dijkstra(w, s, max_dist, dir, edge_weight_fn);

    auto found = 0U;
    for (auto const [m, r] : utl::zip(to_match, result)) {
      if (r.has_value()) {
        ++found;
      } else {
        auto const [_, time] = best_candidate(m, s, max_dist);
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
                                       geo::latlng const& from,
                                       std::vector<geo::latlng> const& to,
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