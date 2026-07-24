#include "osr/routing/parking_matching.h"

#include <functional>
#include <optional>

#include "osr/location.h"
#include "osr/routing/profile.h"

namespace osr {

namespace {

geo::box get_bounding_box(ways const& w, vec<way_idx_t> const& component) {
  // ?? TODO Optional matching distance ??
  auto bbox = geo::box{};
  for (auto const way_idx : component) {
    for (auto const pos : w.way_polylines_[way_idx]) {
      bbox.extend(pos);
    }
  }
  auto const max_matching_distance = 20.0;
  bbox.extend(max_matching_distance);
  return bbox;
}

struct fit {
  bool worse_than(way_candidate const& c, bool const is_designated) const {
    if (best_ == std::nullopt) {
      return true;
    }
    return score(*best_, is_preferred_) < score(c, is_designated);
  }

  double static score(way_candidate const& c, bool const is_preferred) {
    // Penalize not designated ways
    // Add shift to maybe (?) find preferred way, like nearest footpath
    return -((is_preferred ? 1.0 : 5.0) * (c.dist_to_way_ + 2.5));
  }

  std::optional<way_candidate> best_{std::nullopt};
  bool is_preferred_{false};  // For example parking-aisle
  std::size_t component_size_{};
  // double min_dist_{std::numeric_limits<double>::infinity()};
};

template <Profile P>
std::optional<way_candidate> find_closest(
    ways const& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra,
    vec<way_idx_t> const& component_ways,
    vec_map<component_idx_t, std::size_t> const& component_sizes,
    std::function<std::tuple<bool, bool>(way_extra_properties const&)> const&
        p) {
  auto const debug_this = component_ways[0] == 15609;  // TODO: drop - Drop

  auto const params = typename P::parameters{};
  auto const bbox = get_bounding_box(w, component_ways);
  auto const center = bbox.centroid();
  auto const center_loc = location{.pos_ = center, .lvl_ = kNoLevel};
  auto const approx_distance_lng_degrees =
      geo::approx_distance_lng_degrees(center);

  auto const find_best = [&](way_idx_t const way_idx) -> way_candidate {
    auto const [squared_dist, best, segment_idx] =
        geo::approx_squared_distance_to_polyline<
            std::tuple<double, geo::latlng, size_t>>(
            center, w.way_polylines_[way_idx], approx_distance_lng_degrees);
    auto wc = way_candidate{.dist_to_way_ = std::sqrt(squared_dist),
                            .way_ = way_idx,
                            .closest_point_on_way_ = best,
                            .segment_idx_ = static_cast<unsigned>(segment_idx)};
    wc.left_ =
        l.find_next_node<P>(params, wc, center_loc, direction::kBackward,
                            kNoLevel, false, direction::kForward, nullptr,
                            approx_distance_lng_degrees, best, segment_idx);
    wc.right_ =
        l.find_next_node<P>(params, wc, center_loc, direction::kForward,
                            kNoLevel, false, direction::kForward, nullptr,
                            approx_distance_lng_degrees, best, segment_idx);
    return wc;
  };

  auto best_fit = fit{.component_size_ = component_ways.size() + 1};
  l.find(bbox, [&](way_idx_t const way_idx) {
    auto const component = w.r_->way_component_[way_idx];
    if (component_sizes[component] < best_fit.component_size_) {
      return;
    }
    auto const [is_usable, is_preferred] = p(way_extra[way_idx]);
    if (!is_usable) {
      if (debug_this) {
        fmt::println("Not usable: {} (osm: {})", way_idx,
                     w.way_osm_idx_[way_idx]);
      }
      return;
    }
    auto const candidate = static_cast<way_candidate>(find_best(way_idx));
    // START DEBUG
    if (debug_this) {
      fmt::println("way: {} (osm: {})   (left: {}  right: {})", candidate.way_,
                   w.way_osm_idx_[candidate.way_], candidate.left_.valid(),
                   candidate.right_.valid());
    }
    // END DEBUG
    if (best_fit.worse_than(candidate, is_preferred)) {
      best_fit.best_ = candidate;
      best_fit.component_size_ = component_sizes[component];
      // best_fit.min_dist_ = candidate.dist_to_way_;
      best_fit.is_preferred_ = is_preferred;
    }
  });

  if (best_fit.best_ == std::nullopt) {
    fmt::println("Unmatched component: {}  center: {}  way:  {} (osm: {})",
                 w.r_->way_component_[component_ways[0]], center,
                 component_ways[0], w.way_osm_idx_[component_ways[0]]);
  } else {
    fmt::println(
        "Matched component: {}  center: {}  way:  {} (osm: {})  "
        "->   way {} (osm: {}  component: {})",
        w.r_->way_component_[component_ways[0]], center, component_ways[0],
        w.way_osm_idx_[component_ways[0]], best_fit.best_->way_,
        w.way_osm_idx_[best_fit.best_->way_],
        w.r_->way_component_[best_fit.best_->way_]);
  }

  return best_fit.best_;
}

bool contains(vec<way_idx_t> const& v, way_idx_t way_idx) {
  return utl::any_of(v, [&](way_idx_t const o) { return o == way_idx; });
}

vec<way_idx_t> component_ways(ways const& w, way_idx_t const way_idx) {
  auto queue = vec{way_idx};
  auto seen = vec<way_idx_t>{};
  while (!queue.empty()) {
    auto curr = queue.back();
    queue.pop_back();
    seen.push_back(curr);
    for (auto const n : w.r_->way_nodes_[curr]) {
      for (auto const c : w.r_->node_ways_[n]) {
        if (!contains(seen, c) && !contains(queue, c)) {
          queue.push_back(c);
        }
      }
    }
  }
  return seen;
}

}  // namespace

void connect_parking_ways(
    ways& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra,
    unsigned const n_components) {
  // Compute component sizes
  auto component_sizes =
      vec_map<component_idx_t, std::size_t>(n_components, std::size_t{0U});
  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto const component = w.r_->way_component_[way_idx];
    utl::verify(static_cast<std::size_t>(component.v_) < n_components,
                "Invalid component index {} (>= {})", component, n_components);
    ++component_sizes[component];
  }

  w.r_->has_parking_edges_.resize(w.n_nodes());
  auto const add_parking_edge = [&](node_idx_t const node_idx,
                                    parking_edge_idx_t const parking_edge_idx) {
    w.r_->node_parking_edges_.emplace_back(node_idx, parking_edge_idx);
    w.r_->has_parking_edges_.set(node_idx);
  };

  auto const max_isolated_component_size = 20U;  // 100U;
  // Find parking ways with small component sizes
  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto const component = w.r_->way_component_[way_idx];
    auto const p = w.r_->way_properties_[way_idx];
    if (p.is_parking() &&
        component_sizes[component] <= max_isolated_component_size) {
      // TODO: Check only first occurrence is handled - e.g. 812579362 +
      // 812579361 (??)
      auto const parking_component = component_ways(w, way_idx);
      auto const osm_way = w.way_osm_idx_[way_idx];
      utl::verify(w.way_osm_nodes_[way_idx].size() > 0, "Empty way");
      auto const& osm_node = w.way_osm_nodes_[way_idx][0U];
      auto const node_idx = w.find_node_idx(osm_node);
      if (!node_idx.has_value()) {
        fmt::println("Skipped node: {}", osm_node);
        continue;
      }
      utl::verify(*node_idx != node_idx_t::invalid(), "Invalid node {}",
                  osm_node);
      utl::verify(w.r_->node_positions_.size() > node_idx->v_,
                  "Too short: {} >= {}", *node_idx,
                  w.r_->node_positions_.size());
      auto const& pos = w.get_node_pos(*node_idx);
      fmt::println("Connecting {} (osm: {}) ({}, {}) ...", way_idx, osm_way,
                   pos.lat(), pos.lng());

      auto closest_car = find_closest<car>(
          w, l, way_extra, parking_component, component_sizes,
          [&](way_extra_properties const& props) -> std::tuple<bool, bool> {
            return {props.is_car_usable(), props.is_parking_aisle()};
          });
      auto closest_foot = find_closest<foot<false>>(
          w, l, way_extra, parking_component, component_sizes,
          [&](way_extra_properties const& props) -> std::tuple<bool, bool> {
            return {props.is_foot_usable(), props.is_preferred_footpath()};
          });
      if (closest_car.has_value()) {
        if (closest_foot.has_value()) {
          fmt::println("MATCH: {} (osm: {})  ->  {} (osm: {})",
                       closest_car->way_, w.way_osm_idx_[closest_car->way_],
                       closest_foot->way_, w.way_osm_idx_[closest_foot->way_]);

          auto const parking_edge_idx =
              parking_edge_idx_t{w.r_->parking_edges_.size()};
          w.r_->parking_edges_.emplace_back(
              closest_car->left_.node_, closest_car->right_.node_,
              point::from_latlng(closest_car->closest_point_on_way_),
              point::from_latlng(closest_foot->closest_point_on_way_),
              closest_foot->left_.node_, closest_foot->right_.node_);
          if (closest_car->left_.valid()) {
            add_parking_edge(closest_car->left_.node_, parking_edge_idx);
          }
          if (closest_car->right_.valid()) {
            add_parking_edge(closest_car->right_.node_, parking_edge_idx);
          }
          if (closest_foot->left_.valid()) {
            add_parking_edge(closest_foot->left_.node_, parking_edge_idx);
          }
          if (closest_foot->right_.valid()) {
            add_parking_edge(closest_foot->right_.node_, parking_edge_idx);
          }
        } else {
          fmt::println("FAILED: No matching foot way");
        }
      } else {
        fmt::println("FAILED: No matching car way");
      }
      fmt::println("Closest (car): {} (osm: {}) ({}, {}) ...", way_idx, osm_way,
                   pos.lat(), pos.lng());
    }
  }
  utl::sort(w.r_->node_parking_edges_);
}

bool is_parking_way(ways::routing const& r, way_idx_t const way_idx) {
  return way_idx >= r.way_component_.size() &&
         way_idx < r.way_component_.size() + r.parking_edges_.size();
}

geo::polyline parking_way_polyline(ways::routing const& r,
                                   way_idx_t const way_idx,
                                   node_idx_t const from,
                                   node_idx_t const to) {
  utl::verify(is_parking_way(r, way_idx), "way {} is not a valid parking edge",
              way_idx);
  auto const& parking_edge =
      r.parking_edges_[ways::routing::parking_edge::decode_parking_edge(
          r, way_idx)];
  return {r.node_positions_[from], parking_edge.car_extra_point_,
          parking_edge.foot_extra_point_, r.node_positions_[to]};
}

}  // namespace osr
