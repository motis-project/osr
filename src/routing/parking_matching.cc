#include "osr/routing/parking_matching.h"
#include <concepts>

#include <functional>
#include <optional>

#include "utl/concat.h"

#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace {

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
};

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

std::tuple<geo::box, double> get_bbox(ways const& w, way_idx_t const& way_idx) {
  auto const bbox = get_bounding_box(w, way_idx);
  auto const center = bbox.centroid();
  return {bbox, geo::approx_distance_lng_degrees(center)};
}

template <Profile P>
std::optional<way_candidate> find_closest(
    ways const& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra,
    geo::box const& bbox,
    double const approx_distance_lng_degrees,
    auto const& component_sizes,
    std::function<std::tuple<bool, bool>(way_extra_properties const&)> const&
        pred) {
  auto const params = typename P::parameters{};
  // auto const center = point::from_latlng(bbox.centroid());
  auto const center = bbox.centroid();
  auto const loc = location{.pos_ = center, .lvl_ = kNoLevel};
  auto const find_best = [&](way_idx_t const candidate_way) -> way_candidate {
    auto const [squared_dist, best, segment_idx] =
        geo::approx_squared_distance_to_polyline<
            std::tuple<double, geo::latlng, size_t>>(
            center, w.way_polylines_[candidate_way],
            approx_distance_lng_degrees);
    auto wc = way_candidate{.dist_to_way_ = std::sqrt(squared_dist),
                            .way_ = candidate_way,
                            .closest_point_on_way_ = best,
                            .segment_idx_ = static_cast<unsigned>(segment_idx)};
    wc.left_ =
        l.find_next_node<P>(params, wc, loc, direction::kBackward, kNoLevel,
                            false, direction::kForward, nullptr,
                            approx_distance_lng_degrees, best, segment_idx);
    wc.right_ =
        l.find_next_node<P>(params, wc, loc, direction::kForward, kNoLevel,
                            false, direction::kForward, nullptr,
                            approx_distance_lng_degrees, best, segment_idx);
    return wc;
  };

  auto best_fit = fit{.component_size_ = 2};
  l.find(bbox, [&](way_idx_t const candidate_way) {
    auto const component = w.r_->way_component_[candidate_way];
    if (component_sizes[component] < best_fit.component_size_) {
      return;
    }
    auto const [is_usable, is_preferred] = pred(way_extra[candidate_way]);
    if (!is_usable) {
      return;
    }
    auto const candidate = static_cast<way_candidate>(find_best(candidate_way));
    if (best_fit.worse_than(candidate, is_preferred)) {
      best_fit.best_ = candidate;
      best_fit.component_size_ = component_sizes[component];
      best_fit.is_preferred_ = is_preferred;
    }
  });

  return best_fit.best_;
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

  auto const get_best =
      [&](way_idx_t const way_idx, geo::box const& bbox,
          double const approx_distance_lng_degrees,
          std::function<bool(way_properties const&)> const& pred)
      -> std::optional<way_candidate> {
    auto const center = bbox.centroid();
    auto best = way_idx_t::invalid();
    auto best_dist = 0.0;
    auto best_node = node_idx_t::invalid();
    for (auto const node_idx : w.r_->way_nodes_[way_idx]) {
      for (auto const connecting_way : w.r_->node_ways_[node_idx]) {
        if (connecting_way != way_idx &&
            pred(w.r_->way_properties_[connecting_way])) {
          auto const dist = geo::approx_squared_distance(
              center, w.r_->node_positions_[node_idx],
              approx_distance_lng_degrees);
          if (best == way_idx_t::invalid() || dist < best_dist) {
            best = way_idx;
            best_dist = dist;
            best_node = node_idx;
            break;
          }
        }
      }
    }
    return std::optional{way_candidate{
        .dist_to_way_ = best_dist,
        .way_ = best,
        .left_ = {kNoLevel,
                  direction::kForward,
                  best_node,
                  best_dist,
                  cost_t{0},
                  {}},
        .right_ = {},
        .closest_point_on_way_ = w.r_->node_positions_[best_node].as_latlng(),
        .segment_idx_ = 0}};
  };

  auto const make_connection =
      [&](geo::box const& bbox, double const approx_distance_lng_degrees,
          way_candidate const& car_offset,
          geo::polyline_candidate const& car_entrance,
          geo::polyline_candidate const& foot_entrance,
          way_candidate const& foot_offset) -> vec<point> {
    auto conn = vec<point>{};
    auto const center = bbox.centroid();
    auto const is_closer = [&](geo::latlng const& c, geo::latlng const& other) {
      return geo::approx_squared_distance(c, other,
                                          approx_distance_lng_degrees) <
             geo::approx_squared_distance(c, center,
                                          approx_distance_lng_degrees);
    };

    conn.emplace_back(point::from_latlng(car_offset.closest_point_on_way_));
    if (is_closer(car_offset.closest_point_on_way_, car_entrance.best_)) {
      conn.emplace_back(point::from_latlng(car_entrance.best_));
    }
    if (is_closer(foot_offset.closest_point_on_way_, foot_entrance.best_)) {
      conn.emplace_back(point::from_latlng(foot_entrance.best_));
    }
    conn.emplace_back(point::from_latlng(foot_offset.closest_point_on_way_));

    return conn;
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
    auto const is_car_connected = is_connected(way_idx, is_car_accessible);
    auto const is_foot_connected = is_connected(way_idx, is_foot_accessible);
    if (way_idx == 1643) {  // DEBUG only
      fmt::println(
          "TEST: way {}  parking: {}  foot: {}  car: {}  components: {}  "
          "total: {}",
          way_idx, p.is_parking(), is_foot_connected, is_car_connected,
          component_sizes[w.r_->way_component_[way_idx]] == 1,
          p.is_parking() &&
              (component_sizes[w.r_->way_component_[way_idx]] == 1 ||
               !is_car_connected || !is_foot_connected));
    }
    if (p.is_parking() &&
        (component_sizes[w.r_->way_component_[way_idx]] == 1 ||
         !is_car_connected || !is_foot_connected)) {
      auto const [bbox, approx_distance_lng_degrees] = get_bbox(w, way_idx);
      auto const foot_offset =
          is_foot_connected
              ? get_best(way_idx, bbox, approx_distance_lng_degrees,
                         is_foot_accessible)
              : find_closest<foot<false>>(
                    w, l, way_extra, bbox, approx_distance_lng_degrees,
                    component_sizes,
                    [&](way_extra_properties const& props)
                        -> std::tuple<bool, bool> {
                      return {props.is_foot_usable(),
                              props.is_preferred_footpath()};
                    });
      auto const car_offset =
          is_car_connected
              ? get_best(way_idx, bbox, approx_distance_lng_degrees,
                         is_car_accessible)
              : find_closest<car>(w, l, way_extra, bbox,
                                  approx_distance_lng_degrees, component_sizes,
                                  [&](way_extra_properties const& props)
                                      -> std::tuple<bool, bool> {
                                    return {props.is_car_usable(),
                                            props.is_parking_aisle()};
                                  });
      if (way_idx == 1643 || way_idx == 14200 ||
          way_idx == 14201) {  // DEBUG only
        fmt::println(
            "DEBUG OFFSETS: way {}  foot_connected: {}  car_connected: {}  "
            "has_foot: {}  has_car: {}",
            way_idx, is_foot_connected, is_car_connected,
            foot_offset.has_value(), foot_offset.has_value());
      }
      if (!foot_offset.has_value()) {
        fmt::println(
            "WARNING: No footpath candidate found for way {} (osm: {}, "
            "centroid: {})",
            way_idx, w.way_osm_idx_[way_idx], bbox.centroid());
        continue;
      }
      if (!car_offset.has_value()) {
        fmt::println(
            "WARNING: No car usable candidate found for way {} (osm: {}, "
            "centroid: {})",
            way_idx, w.way_osm_idx_[way_idx], bbox.centroid());
        continue;
      }

      auto const car_entrance = geo::approx_squared_distance_to_polyline(
          car_offset->closest_point_on_way_, w.way_polylines_[way_idx],
          approx_distance_lng_degrees);
      auto const foot_entrance = geo::approx_squared_distance_to_polyline(
          foot_offset->closest_point_on_way_, w.way_polylines_[way_idx],
          approx_distance_lng_degrees);
      auto const parking_edge_idx =
          parking_edge_idx_t{w.r_->parking_edges_.size()};
      w.r_->parking_edges_.emplace_back(
          car_offset->left_.node_, car_offset->right_.node_,
          make_connection(bbox, approx_distance_lng_degrees, *car_offset,
                          car_entrance, foot_entrance, *foot_offset),
          foot_offset->left_.node_, foot_offset->right_.node_);
      if (way_idx == 1643) {  // DEBUG only
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
  auto line = geo::polyline{};
  line.emplace_back(r.node_positions_[from]);
  for (auto const& p : parking_edge.connection_) {
    line.emplace_back(p);
  }
  line.emplace_back(r.node_positions_[to]);
  return line;
}

}  // namespace osr
