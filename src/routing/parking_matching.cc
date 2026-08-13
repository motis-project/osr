#include "osr/routing/parking_matching.h"

#include <concepts>
#include <functional>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>

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

template <typename C>
  requires std::ranges::forward_range<C> &&
           std::same_as<std::iter_value_t<C>, point>
geo::polyline to_polyline(C const& c) {
  auto p = geo::polyline{};
  for (auto const& x : c) {
    p.push_back(x.as_latlng());
  }
  return p;
}

ways::routing::parking_edge::offset to_offset(ways const& w,
                                              way_candidate const& wc) {
  auto offset = ways::routing::parking_edge::offset{
      .additional_point_ = point::from_latlng(wc.closest_point_on_way_),
      .way_ = wc.way_,
      .segment_ = wc.segment_idx_,
      .left_ = wc.left_.node_,
      .right_ = wc.right_.node_,
      .dist_left_ = 0U,
      .dist_right_ = 0U};

  if (wc.left_.node_ != node_idx_t::invalid()) {
    offset.dist_left_ =
        geo::length(to_polyline(parking_edge_offset_polyline(w, offset, true)));
  }
  if (wc.right_.node_ != node_idx_t::invalid()) {
    offset.dist_right_ = geo::length(
        to_polyline(parking_edge_offset_polyline(w, offset, false)));
  }

  return offset;
}

geo::box get_bounding_box(ways const& w, way_idx_t const& way_idx) {
  // ?? TODO Optional matching distance ??
  auto bbox = geo::box{};
  for (auto const pos : w.way_polylines_[way_idx]) {
    bbox.extend(pos);
  }
  auto const max_matching_distance = 20.0;
  bbox.extend(max_matching_distance);
  return bbox;
}

vec_map<component_idx_t, std::size_t> compute_component_sizes(
    ways const& w, unsigned const n_components) {
  auto component_sizes =
      vec_map<component_idx_t, std::size_t>(n_components, std::size_t{0U});
  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto const component = w.r_->way_component_[way_idx];
    utl::verify(static_cast<std::size_t>(component.v_) < n_components,
                "Invalid component index {} (>= {})", component, n_components);
    ++component_sizes[component];
  }
  return component_sizes;
}

component_idx_t find_largest_component(
    ways const& w,
    lookup const& l,
    geo::box const& bbox,
    vec_map<component_idx_t, std::size_t> const& component_sizes) {
  auto largest_componet = component_idx_t::invalid();
  auto largest_size = 0UL;
  l.find(bbox, [&](way_idx_t const way_idx) {
    auto const component = w.r_->way_component_[way_idx];
    auto const size = component_sizes[component];
    if (size > largest_size) {
      largest_size = size;
      largest_componet = component;
    }
  });
  return largest_componet;
}

std::tuple<geo::box, geo::latlng, double> get_bbox(ways const& w,
                                                   way_idx_t const& way_idx) {
  auto const bbox = get_bounding_box(w, way_idx);
  auto const center = bbox.centroid();
  return {bbox, center, geo::approx_distance_lng_degrees(center)};
}

template <Profile P>
std::optional<way_candidate> find_closest(
    ways const& w,
    lookup const& l,
    location const& loc,
    direction const dir,
    component_idx_t const matching_component,
    std::function<double(way_candidate const&)> const& score) {
  auto const params = typename P::parameters{};

  auto way_candidates = l.match<P>(params, loc, false, dir, 250.0, nullptr,
                                   std::nullopt, std::nullopt);
  utl::erase_if(way_candidates, [&](way_candidate const& wc) {
    return w.r_->way_component_[wc.way_] != matching_component;
  });
  if (way_candidates.size() == 0) {
    return std::nullopt;
  }
  auto const best = utl::max_element(
      way_candidates, [&](way_candidate const& a, way_candidate const& b) {
        return score(a) < score(b);
      });

  return std::optional{*best};
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
  auto const car_score = [&](way_candidate const& wc) -> double {
    return score(wc.dist_to_way_, way_extra[wc.way_].is_parking_aisle());
  };
  auto const foot_score = [&](way_candidate const& wc) -> double {
    return score(wc.dist_to_way_, way_extra[wc.way_].is_preferred_footpath());
  };

  auto const get_connected_way =
      [&](way_idx_t const way_idx, geo::latlng const& center,
          double const approx_distance_lng_degrees,
          std::function<bool(way_properties const&)> const& pred)
      -> std::optional<way_candidate> {
    auto node = node_idx_t::invalid();
    auto min_dist = 0.0;
    auto lvl = kNoLevel;
    auto segment = 0U;
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
            segment = static_cast<unsigned>(i);
            break;
          }
        }
      }
    }
    utl::verify(node != node_idx_t::invalid(),
                "Connected way must have at least one connected node");
    auto const cost = static_cast<cost_t>(std::rint(
        min_dist * foot<false>::parameters{}.speed_meters_per_second_));
    return std::optional{way_candidate{
        .dist_to_way_ = min_dist,
        .way_ = way_idx,
        .left_ = {.lvl_ = lvl,
                  .way_dir_ = direction::kForward,
                  .node_ = node,
                  .dist_to_node_ = min_dist,
                  .cost_ = cost,
                  .path_ = {}},
        .right_ = {},
        .closest_point_on_way_ = w.r_->node_positions_[node].as_latlng(),
        .segment_idx_ = segment}};
  };

  auto const make_connection = [&](geo::box const& bbox,
                                   double const approx_distance_lng_degrees,
                                   way_candidate const& car_offset,
                                   geo::polyline_candidate const& car_entrance,
                                   geo::polyline_candidate const& foot_entrance,
                                   way_candidate const& foot_offset)
      -> std::tuple<vec<point>, std::uint16_t> {
    auto const center = bbox.centroid();
    auto const is_closer = [&](geo::latlng const& c, geo::latlng const& other) {
      return geo::approx_squared_distance(c, other,
                                          approx_distance_lng_degrees) <
             geo::approx_squared_distance(c, center,
                                          approx_distance_lng_degrees);
    };

    auto conn = vec<point>{};
    conn.emplace_back(point::from_latlng(car_offset.closest_point_on_way_));
    if (is_closer(car_offset.closest_point_on_way_, car_entrance.best_)) {
      conn.emplace_back(point::from_latlng(car_entrance.best_));
    }
    if (is_closer(foot_offset.closest_point_on_way_, foot_entrance.best_)) {
      conn.emplace_back(point::from_latlng(foot_entrance.best_));
    }
    conn.emplace_back(point::from_latlng(foot_offset.closest_point_on_way_));

    return {conn, static_cast<std::uint16_t>(geo::length(to_polyline(conn)))};
  };

  w.r_->has_parking_edges_.resize(w.n_nodes());
  auto const add_parking_edge = [&](node_idx_t const node_idx,
                                    parking_edge_idx_t const parking_edge_idx) {
    w.r_->node_parking_edges_.emplace_back(node_idx, parking_edge_idx);
    w.r_->has_parking_edges_.set(node_idx);
  };

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

    auto const [bbox, center, approx_distance_lng_degrees] =
        get_bbox(w, way_idx);
    auto const matching_component =
        find_largest_component(w, l, bbox, component_sizes);
    if (matching_component == component_idx_t::invalid()) {
      continue;
    }

    auto const is_same_component =
        w.r_->way_component_[way_idx] == matching_component;

    auto const loc = location{.pos_ = center, .lvl_ = kNoLevel};
    auto const foot_offset =
        (is_same_component && is_foot_connected)
            ? get_connected_way(way_idx, center, approx_distance_lng_degrees,
                                is_foot_accessible)
            : find_closest<foot<false>>(w, l, loc, direction::kForward,
                                        matching_component, foot_score);
    auto const car_offset =
        (is_same_component && is_car_connected)
            ? get_connected_way(way_idx, center, approx_distance_lng_degrees,
                                is_car_accessible)
            : find_closest<car>(w, l, loc, direction::kBackward,
                                matching_component, car_score);
    if (!foot_offset.has_value() || !car_offset.has_value()) {
      fmt::println(
          "WARNING: No usable way candidate found for way {}"
          " (osm: {}, centroid: {})",
          way_idx, w.way_osm_idx_[way_idx], bbox.centroid());
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

    if (way_idx == 1643 || way_idx == 14200 ||
        way_idx == 14201) {  // DEBUG only
      fmt::println(
          "DEBUG OFFSETS: way {}  foot_connected: {}  car_connected: {}  "
          "has_foot: {}  has_car: {}",
          way_idx, is_foot_connected, is_car_connected, foot_offset.has_value(),
          foot_offset.has_value());
    }

    auto const car_entrance = geo::approx_squared_distance_to_polyline(
        car_offset->closest_point_on_way_, w.way_polylines_[way_idx],
        approx_distance_lng_degrees);
    auto const foot_entrance = geo::approx_squared_distance_to_polyline(
        foot_offset->closest_point_on_way_, w.way_polylines_[way_idx],
        approx_distance_lng_degrees);
    auto const parking_edge_idx =
        parking_edge_idx_t{w.r_->parking_edges_.size()};
    auto [conn, dist] =
        make_connection(bbox, approx_distance_lng_degrees, *car_offset,
                        car_entrance, foot_entrance, *foot_offset);
    w.r_->parking_edges_.emplace_back(std::move(conn),
                                      to_offset(w, *car_offset),
                                      to_offset(w, *foot_offset), dist);
    // if (way_idx == 1643) {  // DEBUG only
    if (way_idx == 8341) {  // DEBUG only
      fmt::println(
          "Added nodes: car/left: {}  car/right: {}  foot/left: {}  "
          "foot/right: {}",
          car_offset->left_.valid(), car_offset->right_.valid(),
          foot_offset->left_.valid(), foot_offset->right_.valid());
    }
    if (car_offset->left_.valid()) {
      add_parking_edge(car_offset->left_.node_, parking_edge_idx);
    }
    if (car_offset->right_.valid()) {
      add_parking_edge(car_offset->right_.node_, parking_edge_idx);
    }
    if (foot_offset->left_.valid()) {
      add_parking_edge(foot_offset->left_.node_, parking_edge_idx);
    }
    if (foot_offset->right_.valid()) {
      add_parking_edge(foot_offset->right_.node_, parking_edge_idx);
    }
  }
  utl::sort(w.r_->node_parking_edges_);
}

bool is_parking_way(ways::routing const& r, way_idx_t const way_idx) {
  return way_idx >= r.way_component_.size() &&
         way_idx < r.way_component_.size() + r.parking_edges_.size();
}

geo::polyline parking_way_polyline(ways const& w,
                                   way_idx_t const way_idx,
                                   direction const dir,
                                   node_idx_t const from,
                                   node_idx_t const to) {
  auto const& r = *w.r_;
  utl::verify(is_parking_way(r, way_idx), "way {} is not a valid parking edge",
              way_idx);
  auto const& parking_edge =
      r.parking_edges_[ways::routing::parking_edge::decode_parking_edge(
          r, way_idx)];
  if (ways::routing::parking_edge::decode_parking_edge(r, way_idx) == 4) {
    fmt::println(
        "DEBUG PARKING EDGE:  FROM: Way: {} ({}) ({} -> {}),   TO: Way: {} "
        "({}) ({} -> {})",
        parking_edge.from_.way_, parking_edge.from_.segment_,
        parking_edge.from_.left_, parking_edge.from_.right_,
        parking_edge.to_.way_, parking_edge.to_.segment_,
        parking_edge.to_.left_, parking_edge.to_.right_);
  }

  return parking_edge_polyline(w, parking_edge,
                               dir == direction::kForward ? from : to,
                               dir == direction::kForward ? to : from);
}

}  // namespace osr
