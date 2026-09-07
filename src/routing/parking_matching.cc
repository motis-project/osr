#include "osr/routing/parking_matching.h"

#include <concepts>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>

#include "osr/routing/additional_connection.h"
#include "utl/erase_if.h"

#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/for_each_parking_edge.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace {

vec_map<component_idx_t, std::size_t> compute_component_sizes(
    ways const& w, unsigned const n_components) {
  auto component_sizes =
      vec_map<component_idx_t, std::size_t>(n_components, std::size_t{0U});
  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const component = w.r_->way_component_[way_idx_t{i}];
    utl::verify(static_cast<std::size_t>(component.v_) < n_components,
                "Invalid component index {} (>= {})", component, n_components);
    ++component_sizes[component];
  }
  return component_sizes;
}

std::tuple<geo::latlng, double, component_idx_t> analyze_surroundings(
    ways const& w,
    lookup const& l,
    way_idx_t const way_idx,
    vec_map<component_idx_t, std::size_t> const& component_sizes) {
  // Compute bounding box
  auto const get_bounding_box = [&]() {
    constexpr auto const kExtensionDistance = 20.0;
    auto bbox = geo::box{};
    for (auto const pos : w.way_polylines_[way_idx]) {
      bbox.extend(pos);
    }
    bbox.extend(kExtensionDistance);
    return bbox;
  };
  // Identify component_idx of largest nearby component
  auto const get_largest_component_idx = [&](geo::box const& bbox) {
    auto largest_componet = component_idx_t::invalid();
    auto largest_size = 0UL;
    l.find(bbox, [&](way_idx_t const candidate) {
      auto const component = w.r_->way_component_[candidate];
      auto const size = component_sizes[component];
      if (size > largest_size) {
        largest_size = size;
        largest_componet = component;
      }
    });
    return largest_componet;
  };

  auto const bbox = get_bounding_box();
  auto const center = bbox.centroid();
  return {center, geo::approx_distance_lng_degrees(center),
          get_largest_component_idx(bbox)};
}

template <Profile P>
std::optional<way_candidate> find_closest(
    [[maybe_unused]] ways const& w,
    lookup const& l,
    location const& loc,
    direction const dir,
    [[maybe_unused]] component_idx_t const matching_component,
    std::function<double(double, way_idx_t)> const& score) {
  auto const params = typename P::parameters{};

  auto best = std::optional<way_candidate>{};
  auto matches = match_result{};
  l.match<P>(params, loc, false, dir, 250.0, nullptr, matches, std::nullopt);
  auto best_score = std::numeric_limits<double>::lowest();

  for (auto i = match_idx_t{0U}; i < match_idx_t{matches.size()}; ++i) {
    auto const match = matches[i];
    for (auto j = 0U; j < match.size(); ++j) {
      auto const way_idx = match.way_[j];
      if (w.r_->way_component_[way_idx] != matching_component) {
        continue;
      }
      auto const wc = way_candidate{
          .dist_to_way_ = match.dist_to_way_[j],
          .way_ = way_idx,
          .left_ = match.left(j),
          .right_= match.right(j),
          .closest_point_on_way_ = loc.pos_,
      };
      auto const s = score(match.dist_to_way_[j], match.way_[j]);
      if (s > best_score) {
        // best = {match};
        best = wc;
        best_score = s;
      }
    }
  }
  return best;
}

}  // namespace

void connect_parking_ways(
    ways& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra,
    unsigned const n_components) {
  auto const component_sizes = compute_component_sizes(w, n_components);

  auto const is_connected =
      [&](way_idx_t const way_idx,
          std::function<bool(way_properties const&)> const& pred) {
        return utl::any_of(
            w.r_->way_nodes_[way_idx], [&](node_idx_t const node_idx) {
              return utl::any_of(
                  w.r_->node_ways_[node_idx],
                  [&](way_idx_t const connecting_way) {
                    return connecting_way != way_idx &&
                           pred(w.r_->way_properties_[connecting_way]);
                  });
            });
      };

  auto const is_car_accessible = [&](way_properties const& props) {
    return props.is_car_accessible();
  };
  auto const is_foot_accessible = [&](way_properties const& props) {
    return props.is_foot_accessible();
  };

  auto const score = [&](double const dist_to_way, bool const is_preferred) {
    // Penalize not designated ways
    // Add shift to find nearby preferred ways, like nearest footpath
    // Lower penalty to not match with ways too far away
    return -((1 + ((is_preferred ? 0.0 : 4.0) / (dist_to_way + 1.0))) *
             (dist_to_way + 2.5));
  };
  auto const car_score = [&](double const dist_to_way,
                             way_idx_t const way_idx) -> double {
    return score(dist_to_way, way_extra[way_idx].is_parking_aisle());
  };
  auto const foot_score = [&](double const dist_to_way,
                              way_idx_t const way_idx) -> double {
    return score(dist_to_way, way_extra[way_idx].is_preferred_footpath());
  };

  auto const get_connected_way =
      [&](way_idx_t const way_idx, geo::latlng const& center,
          double const approx_distance_lng_degrees, bool const is_from,
          std::function<bool(way_properties const&)> const& pred)
      -> std::optional<way_candidate> {
    auto node = node_idx_t::invalid();
    auto min_dist = 0.0;
    auto lvl = kNoLevel;
    auto idx = 0U;
    for (auto const [i, node_idx] : utl::enumerate(w.r_->way_nodes_[way_idx])) {
      for (auto const connecting_way : w.r_->node_ways_[node_idx]) {
        auto const props = w.r_->way_properties_[connecting_way];
        if (connecting_way != way_idx && pred(props)) {
          auto const dist = geo::approx_squared_distance(
              center, w.r_->node_positions_[node_idx],
              approx_distance_lng_degrees);
          if (node == node_idx_t::invalid() || dist < min_dist) {
            node = node_idx;
            min_dist = dist;
            lvl = props.from_level();
            idx = static_cast<unsigned>(i);
            break;
          }
        }
      }
    }
    utl::verify(node != node_idx_t::invalid(),
                "Connected way must have at least one connected node");
    auto const cost = static_cast<cost_t>(std::rint(
        min_dist * foot<false>::parameters{}.speed_meters_per_second_));
    // TODO: MK - Will direction be used? Or can we use kForward only?
    return std::optional{
        idx == 0
            ? way_candidate{.dist_to_way_ = min_dist,
                            .way_ = way_idx,
                            .left_ =
                                {
                                    .lvl_ = lvl,
                                    .way_dir_ = is_from ? direction::kBackward
                                                        : direction::kForward,
                                    .node_ = node,
                                    .dist_to_node_ = min_dist,
                                    .cost_ = cost,
                                },
                            .right_ = {},
                            .closest_point_on_way_ =
                                w.r_->node_positions_[node].as_latlng(),
                            .segment_idx_ = 0U}
            : way_candidate{.dist_to_way_ = min_dist,
                            .way_ = way_idx,
                            .left_ = {},
                            .right_ =
                                {
                                    .lvl_ = lvl,
                                    .way_dir_ = is_from ? direction::kForward
                                                        : direction::kBackward,
                                    .node_ = node,
                                    .dist_to_node_ = min_dist,
                                    .cost_ = cost,
                                },
                            .closest_point_on_way_ =
                                w.r_->node_positions_[node].as_latlng(),
                            .segment_idx_ = idx - 1U}};
  };

  auto const make_connection =
      [&](geo::latlng const& center, double const approx_distance_lng_degrees,
          way_candidate const& car_offset,
          geo::polyline_candidate const& car_entrance,
          geo::polyline_candidate const& foot_entrance,
          way_candidate const& foot_offset) -> vec<point> {
    auto const is_closer = [&](geo::latlng const& c, geo::latlng const& other) {
      return geo::approx_squared_distance(c, other,
                                          approx_distance_lng_degrees) <
             geo::approx_squared_distance(c, center,
                                          approx_distance_lng_degrees);
    };

    auto conn = vec<point>{};
    conn.push_back(point::from_latlng(car_offset.closest_point_on_way_));
    if (is_closer(car_offset.closest_point_on_way_, car_entrance.best_)) {
      conn.push_back(point::from_latlng(car_entrance.best_));
    }
    if (is_closer(foot_offset.closest_point_on_way_, foot_entrance.best_)) {
      conn.push_back(point::from_latlng(foot_entrance.best_));
    }
    conn.push_back(point::from_latlng(foot_offset.closest_point_on_way_));

    return conn;
  };

  w.r_->has_additional_connections_.resize(w.n_nodes());

  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto const p = w.r_->way_properties_[way_idx];

    if (!p.is_parking()) {
      continue;
    }
    auto const is_car_connected = is_connected(way_idx, is_car_accessible);
    auto const is_foot_connected = is_connected(way_idx, is_foot_accessible);
    if (is_car_connected && is_foot_connected) {
      continue;
    }

    auto const [center, approx_distance_lng_degrees, matching_component] =
        analyze_surroundings(w, l, way_idx, component_sizes);
    if (matching_component == component_idx_t::invalid()) {
      continue;
    }

    auto const is_same_component =
        w.r_->way_component_[way_idx] == matching_component;

    auto const loc = location{.pos_ = center, .lvl_ = kNoLevel};
    auto const foot_offset =
        (is_same_component && is_foot_connected)
            ? get_connected_way(way_idx, center, approx_distance_lng_degrees,
                                false, is_foot_accessible)
            : find_closest<foot<false>>(w, l, loc, direction::kForward,
                                        matching_component, foot_score);
    auto const car_offset =
        (is_same_component && is_car_connected)
            ? get_connected_way(way_idx, center, approx_distance_lng_degrees,
                                true, is_car_accessible)
            : find_closest<car>(w, l, loc, direction::kBackward,
                                matching_component, car_score);
    if (!foot_offset.has_value() || !car_offset.has_value()) {
      fmt::println(
          "WARNING: No usable way candidate found for way {}"
          " (osm: {}, centroid: {})",
          way_idx, w.way_osm_idx_[way_idx], center);
      continue;
    }
    if (!foot_offset->left_.valid() && !foot_offset->right_.valid()) {
      fmt::println("Connected footpath not usable! Way: {}  at: {}",
                   foot_offset->way_, foot_offset->closest_point_on_way_);
      continue;
    }
    if (!car_offset->left_.valid() && !car_offset->right_.valid()) {
      fmt::println("Connected carpath not usable! Way: {}  at: {}",
                   car_offset->way_, car_offset->closest_point_on_way_);
      continue;
    }

    auto const car_entrance = geo::approx_squared_distance_to_polyline(
        car_offset->closest_point_on_way_, w.way_polylines_[way_idx],
        approx_distance_lng_degrees);
    auto const foot_entrance = geo::approx_squared_distance_to_polyline(
        foot_offset->closest_point_on_way_, w.way_polylines_[way_idx],
        approx_distance_lng_degrees);
    auto conn =
        make_connection(center, approx_distance_lng_degrees, *car_offset,
                        car_entrance, foot_entrance, *foot_offset);
    add_additional_connection(*w.r_, to_offset(w, *car_offset, conn.front()),
                              to_offset(w, *foot_offset, conn.back()),
                              std::move(conn), true);
  }
  utl::sort(w.r_->additional_node_connections_);
}

}  // namespace osr
