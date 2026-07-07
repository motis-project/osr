#include "osr/routing/parking_matching.h"

#include <optional>

#include "utl/get_or_create.h"

#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/types.h"

namespace osr {

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

struct best_candidate {
  way_candidate best_;
  double min_dist_{std::numeric_limits<double>::infinity()};
  double min_dist_accessible_{std::numeric_limits<double>::infinity()};
  double min_dist_designated_{std::numeric_limits<double>::infinity()};
  double min_dist_designated_accessible_{
      std::numeric_limits<double>::infinity()};
  bool is_accessible;
  bool is_designated_;  // For example parking-aisle
};

template <Profile P>
auto find_closest(ways const& w,
                  lookup const& l,
                  vec<way_idx_t> const& component_ways,
                  component_idx_t const parking_component) {
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
  auto const is_accessible = [](way_candidate const& wc) -> bool {
    return wc.left_.valid() || wc.right_.valid();
  };
  auto const is_designated = [&](way_candidate const& wc) -> bool {
    auto const p = w.r_->way_properties_[wc.way_];
    return p.is_parking_aisle();
  };

  auto bests = hash_map<component_idx_t, best_candidate>{};
  l.find(bbox, [&](way_idx_t const way_idx) {
    auto const component = w.r_->way_component_[way_idx];
    if (component == parking_component) {
      return;
    }
    auto created = false;
    auto const candidate = static_cast<way_candidate>(find_best(way_idx));
// START DEBUG
    if (parking_component == 462) {
      fmt::println("way: {} (osm: {})   (left: {}  right: {})", candidate.way_, w.way_osm_idx_[candidate.way_], candidate.left_.valid(), candidate.right_.valid());
    }
// END DEBUG
    auto& curr = utl::get_or_create(bests, component, [&]() {
      created = true;
      auto const accessible = is_accessible(candidate);
      auto const designated = is_designated(candidate);
      auto const dist = candidate.dist_to_way_;
      auto const inf = std::numeric_limits<double>::infinity();
      return best_candidate{.best_ = std::move(candidate),
                            .min_dist_ = dist,
                            .min_dist_accessible_ = accessible ? dist : inf,
                            .min_dist_designated_ = designated ? dist : inf,
                            .min_dist_designated_accessible_ =
                                (accessible && designated) ? dist : inf,
                            .is_accessible = accessible,
                            .is_designated_ = designated};
    });
    if (created) {
      return;
    }
    auto const accessible = is_accessible(candidate);
    auto const designated = is_designated(candidate);
    auto const dist = candidate.dist_to_way_;
    if (dist < curr.min_dist_accessible_) {
      curr.min_dist_ = dist;
    }
    if (accessible && dist < curr.min_dist_accessible_) {
      curr.min_dist_accessible_ = dist;
    }
    if (designated && dist < curr.min_dist_designated_) {
      curr.min_dist_designated_ = dist;
    }
    if (accessible && designated &&
        dist < curr.min_dist_designated_accessible_) {
      curr.min_dist_designated_accessible_ = dist;
    }
    // if (accessible && dist < curr.
    if (!designated && curr.is_designated_) {
      return;
    }

    if (parking_component == 462) {
      fmt::println("testing...");
    }
    if ((designated && !curr.is_designated_) ||
        (candidate.dist_to_way_ < curr.best_.dist_to_way_)) {
      if (parking_component == 462) {
        fmt::println("UPDATE");
      }
      curr.best_ = candidate;
      curr.is_designated_ = designated;
    }
  });

  auto s = std::string{"["};
  for (auto const& [k, v] : bests) {
    s += fmt::format("{} ({}), ", v.best_.closest_point_on_way_, k);
  }
  fmt::println("Connections {} ({}) -> {}      - bbox: [{}, {}]",
               parking_component, center, s, bbox.min_, bbox.max_);

  // TODO: return - Update return value
  return -1;
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

void connect_parking_ways(ways const& w,
                          lookup const& l,
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
      if (component_sizes[component] > 1) {
        fmt::println("Component ways ({}): {}", component_sizes[component],
                     parking_component);
      }
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

      [[maybe_unused]] auto closest_car =
          find_closest<car>(w, l, parking_component, component);
      fmt::println("Closest (car): {} (osm: {}) ({}, {}) ...", way_idx, osm_way,
                   pos.lat(), pos.lng());
    }
  }
}

}  // namespace osr
