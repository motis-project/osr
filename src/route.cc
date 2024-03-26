#include "osr/route.h"

#include "osr/dijkstra.h"
#include "osr/lookup.h"
#include "utl/timer.h"

namespace osr {

search_profile read_profile(std::string_view s) {
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

dist_t get_dist(ways const& w,
                way_idx_t const way,
                std::uint16_t const from,
                std::uint16_t const to) {
  auto sum = dist_t{0U};
  for (auto i = from; i != to; ++i) {
    sum += w.way_node_dist_[way][i];
  }
  return sum;
}

void add_path(ways const& w,
              node_idx_t const from,
              node_idx_t const to,
              std::vector<geo::latlng>& path) {
  struct candidate {
    way_idx_t way_;
    std::uint16_t from_, to_;
    bool forward_;
  };

  // Find ways that are going through from AND to.
  // Ways are sorted, so we compute the set intersection in O(N).
  auto const from_ways = utl::zip(w.node_ways_[from], w.node_in_way_idx_[from]);
  auto const to_ways = utl::zip(w.node_ways_[to], w.node_in_way_idx_[to]);
  auto a = begin(from_ways), b = begin(to_ways);
  auto intersection = std::vector<candidate>{};
  while (a != end(from_ways) && b != end(to_ways)) {
    auto const& [a_way, a_idx] = *a;
    auto const& [b_way, b_idx] = *b;
    if (a_way < b_way) {
      ++a;
    } else if (b_way < a_way) {
      ++b;
    } else {
      intersection.emplace_back(a_way, std::min(a_idx, b_idx),
                                std::max(a_idx, b_idx), a_idx < b_idx);
      ++a;
      ++b;
    }
  }

  // Use the way that goes through both with the shortest distance.
  // TODO: check with the profile if the way can be used!
  // TODO: use duration instead of distance (like the SSSP algorithm).
  auto best_dist = kInfeasible;
  auto best_i = 0U;
  auto i = 0U;
  for (auto const& [way, from_idx, to_idx, _] : intersection) {
    auto const dist = get_dist(w, way, from_idx, to_idx);
    if (dist < best_dist) {
      best_i = i;
      best_dist = dist;
    }
    ++i;
  }

  // Go through way, collect all coordinates into the polyline.
  auto const& [way, from_idx, to_idx, forward] = intersection[best_i];
  auto active = false;
  auto tmp = std::vector<geo::latlng>{};
  for (auto const [osm_idx, coord] :
       utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way])) {
    if (!active && w.node_to_osm_[w.way_nodes_[way][from_idx]] == osm_idx) {
      active = true;
    }
    if (active) {
      tmp.emplace_back(coord);

      if (w.node_to_osm_[w.way_nodes_[way][to_idx]] == osm_idx) {
        break;
      }
    }
  }

  if (!forward) {
    std::reverse(begin(tmp), end(tmp));
  }
  path.insert(end(path), begin(tmp), end(tmp));
}

path reconstruct(ways const& w,
                 dijkstra_state const& s,
                 node_idx_t const dest,
                 std::vector<geo::latlng> const& dest_path,
                 dist_t const dist,
                 start_dist const& start) {
  auto p = path{.time_ = dist, .polyline_ = dest_path};
  auto n = dest;
  auto last = n;
  while (n != node_idx_t::invalid()) {
    last = n;
    auto const& e = s.dist_.at(n);
    if (n == e.pred_) {
      abort();
    }
    if (e.pred_ != node_idx_t::invalid()) {
      add_path(w, n, e.pred_, p.polyline_);
    }
    n = e.pred_;
  }
  auto [left_node, _, left_poly] = start.init_left_;
  auto [right_node, _1, right_poly] = start.init_right_;

  if (left_node == last) {
    std::reverse(begin(left_poly), end(left_poly));
    p.polyline_.insert(end(p.polyline_), begin(left_poly), end(left_poly));
  } else {
    std::reverse(begin(right_poly), end(right_poly));
    p.polyline_.insert(end(p.polyline_), begin(right_poly), end(right_poly));
  }

  std::reverse(begin(p.polyline_), end(p.polyline_));

  return p;
}

template <typename EdgeWeightFn>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t const max_dist,
                          routing_state& s,
                          EdgeWeightFn&& edge_weight_fn) {

  l.find(from, s.start_candidates_, edge_weight_fn);
  utl::verify(!s.start_candidates_.empty(),
              "no start candidates around {} found", from);

  l.find(to, s.dest_candidates_, edge_weight_fn);
  utl::verify(!s.dest_candidates_.empty(),
              "no destination candidates around {} found", to);

  s.dijkstra_state_.reset(max_dist);

  auto const& start = s.start_candidates_.front();
  for (auto const x : {&start.init_left_, &start.init_right_}) {
    auto const& [node, dist, _] = *x;
    if (node != node_idx_t::invalid() && dist < max_dist) {
      s.dijkstra_state_.add_start(node, dist);
    }
  }

  dijkstra(w, s.dijkstra_state_, max_dist, edge_weight_fn);

  auto const dest = s.dest_candidates_.front();
  auto best_dist = std::numeric_limits<dist_t>::max();
  auto best_node = node_idx_t::invalid();
  auto best_dest_path = static_cast<std::vector<geo::latlng> const*>(nullptr);
  for (auto const x : {&dest.init_left_, &dest.init_right_}) {
    auto const& [node, dist, dest_path] = *x;
    if (node != node_idx_t::invalid() && dist < max_dist) {
      auto const target_dist = s.dijkstra_state_.get_dist(node);
      if (target_dist == kInfeasible) {
        continue;
      }

      auto const total_dist = target_dist + dist;
      if (total_dist < max_dist && total_dist < best_dist) {
        best_dist = total_dist;
        best_node = node;
        best_dest_path = &dest_path;
      }
    }
  }

  if (best_node == node_idx_t::invalid()) {
    return std::nullopt;
  }

  return reconstruct(w, s.dijkstra_state_, best_node, *best_dest_path,
                     best_dist, start);
}

std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t const max_dist,
                          routing_state& s,
                          search_profile const profile) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, from, to, max_dist, s, foot{});
    case search_profile::kBike:
      return route(w, l, from, to, max_dist, s, bike{});
    case search_profile::kCar: return route(w, l, from, to, max_dist, s, car{});
  }
  throw utl::fail("invalid profile");
}

}  // namespace osr