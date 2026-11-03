#include "osr/routing/parking_matching.h"

#include <concepts>
#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <optional>
#include <ranges>

#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/additional_connection.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace {

using offset = ways::routing::full_additional_connection::offset;

using matching_result_t = std::pair<offset, point>;

// Radius for extended bounding box to find nearby ways
constexpr auto const kExtensionDistance = 20.0;

struct connected_way_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }
  offset to_offset() const {
    return {
        .left_ = idx_ == 0 ? offset::side{.node_ = node_, .dist_ = 0U}
                           : offset::side{},
        .right_ = idx_ != 0 ? offset::side{.node_ = node_, .dist_ = 0U}
                            : offset::side{},
        .way_ = way_,
    };
  }
  way_idx_t way_{way_idx_t::invalid()};
  node_idx_t node_{node_idx_t::invalid()};
  double dist_{0.0};
  unsigned idx_{0U};
};

struct best_matching_candidate {
  bool valid() const { return component_ != 0U; }

  bool update_component(std::optional<std::uint32_t> const component) {
    if (!component.has_value() || (valid() && component_ < component)) {
      return false;
    }
    if (!valid() || component < component_) {
      component_ = *component;
      score_ = std::numeric_limits<double>::lowest();
    }
    return true;
  }

  void update(way_idx_t const way_idx,
              match_result::view const& match,
              unsigned const j,
              double const score) {
    if (score <= score_) {
      return;
    }
    auto const& left = match.left(j);
    auto const& right = match.right(j);
    utl::verify(left.way_dir_ == opposite(right.way_dir_),
                "expected opposite directions for matching nodes");
    offset_ = {
        .left_ = {.node_ = left.node_,
                  .dist_ = static_cast<std::uint16_t>(left.dist_to_node_)},
        .right_ = {.node_ = right.node_,
                   .dist_ = static_cast<std::uint16_t>(right.dist_to_node_)},
        .way_ = way_idx};
    dir_ = left.way_dir_;
    score_ = score;
  }

  offset offset_{};
  direction dir_{};
  double score_{std::numeric_limits<double>::lowest()};
  std::uint32_t component_{0U};
};

template <Profile P>
std::optional<matching_result_t> find_closest_offset(
    ways const& w,
    lookup const& l,
    location const& loc,
    direction const dir,
    component_class const cc,
    std::function<double(double, way_idx_t)> const& score) {
  auto const params = typename P::parameters{};
  auto const& class_components = w.r_->get_class_components(cc);

  auto best = best_matching_candidate{};

  auto matches = match_result{};
  l.match<P>(params, loc, false, dir, 250.0, nullptr, matches, std::nullopt);

  for (auto i = match_idx_t{0U}; i < match_idx_t{matches.size()}; ++i) {
    auto const match = matches[i];
    for (auto j = 0U; j < match.size(); ++j) {
      auto const way_idx = match.way_[j];
      auto const way_component = class_components.get(way_idx);
      if (best.update_component(way_component)) {
        auto const s = score(match.dist_to_way_[j], way_idx);
        best.update(way_idx, match, j, s);
      }
    }
  }

  if (best.valid()) {
    utl::verify(best.offset_.left_.valid() || best.offset_.right_.valid(),
                "match returned without valid nodes");
    auto const use_left = best.offset_.left_.valid();
    auto const path = l.get_node_candidate_path(
        best.offset_.way_,
        use_left ? best.offset_.left_.node_ : best.offset_.right_.node_,
        use_left ? best.dir_ : opposite(best.dir_), false, loc);
    utl::verify(!path.empty(), "Path should not be empty. way: {} query: {}",
                best.offset_.way_, loc.pos_);
    return {{std::move(best.offset_), point::from_latlng(path.front())}};
  } else {

    std::clog << fmt::format(
        "osr: No matching way found for location {} (component_class={})\n",
        loc.pos_, to_str(cc));
    return std::nullopt;
  }
}

double score(double const dist_to_way, bool const is_preferred) {
  // Penalize not designated ways
  // Add shift to find nearby preferred ways, like nearest footpath
  // Lower penalty to not match with ways too far away
  return -((1 + ((is_preferred ? 0.0 : 4.0) / (dist_to_way + 1.0))) *
           (dist_to_way + 2.5));
}

}  // namespace

template <ParkingProfile P>
void connect_parking_ways(
    ways& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra,
    std::function<bool(way_properties const&)> const& is_street_accessible,
    std::function<double(double, way_idx_t)> const& street_way_score) {
  auto const is_foot_accessible = [&](way_properties const& props) {
    return props.is_foot_accessible();
  };

  auto const foot_way_score = [&](double const dist_to_way,
                                  way_idx_t const way_idx) -> double {
    return score(dist_to_way, way_extra[way_idx].is_preferred_footpath());
  };

  auto const get_parking_space_center = [&](way_idx_t const way_idx) {
    auto bbox = geo::box{};
    for (auto const pos : w.way_polylines_[way_idx]) {
      bbox.extend(pos);
    }
    bbox.extend(kExtensionDistance);
    return bbox.centroid();
  };

  auto const is_connected = [&](way_idx_t const way_idx,
                                class_components const& components) {
    return components.get(way_idx).has_value();
  };

  auto const get_closest_connected_way =
      [&](way_idx_t const way_idx, geo::latlng const& center,
          double const approx_distance_lng_degrees,
          std::function<bool(way_properties const&)> const& is_accessible)
      -> std::optional<matching_result_t> {
    auto best = connected_way_candidate{};
    for (auto const [i, node_idx] : utl::enumerate(w.r_->way_nodes_[way_idx])) {
      for (auto const connecting_way : w.r_->node_ways_[node_idx]) {
        auto const way_props = w.r_->way_properties_[connecting_way];
        if (connecting_way != way_idx && is_accessible(way_props)) {
          auto const dist = geo::approx_squared_distance(
              center, w.r_->node_positions_[node_idx],
              approx_distance_lng_degrees);
          if (best.node_ == node_idx_t::invalid() || dist < best.dist_) {
            best = {
                .way_ = connecting_way,
                .node_ = node_idx,
                .dist_ = dist,
                .idx_ = static_cast<unsigned>(i),
            };
            // We can pick any way using this node as candidate
            break;
          }
        }
      }
    }
    if (!best.valid()) {
      // TODO: MK - Why can inaccessible ways have a non trivial component?
      std::clog << fmt::format(
          "osr: No accessible way connected to way with component size > 1: "
          "Matching failed for parking space {} with center {}.\n",
          way_idx, center);
      return std::nullopt;
    }
    return std::optional<matching_result_t>{
        {best.to_offset(), w.r_->node_positions_[best.node_]}};
  };

  auto const make_connection =
      [&](geo::latlng const& center, double const approx_distance_lng_degrees,
          geo::latlng street_way_point, geo::latlng street_connection_point,
          geo::latlng foot_connection_point,
          geo::latlng foot_way_point) -> vec<point> {
    auto const is_closer_than_center = [&](geo::latlng const& p,
                                           geo::latlng const& candidate) {
      return geo::approx_squared_distance(p, candidate,
                                          approx_distance_lng_degrees) <
             geo::approx_squared_distance(p, center,
                                          approx_distance_lng_degrees);
    };

    auto conn = vec<point>{};
    auto last = geo::latlng{};
    auto const add_point = [&](geo::latlng const p) {
      if (conn.empty() || p != last) {
        conn.push_back(point::from_latlng(p));
        last = p;
      }
    };
    add_point(street_way_point);
    if (is_closer_than_center(street_way_point, street_connection_point)) {
      add_point(street_connection_point);
    }
    if (is_closer_than_center(foot_way_point, foot_connection_point)) {
      add_point(foot_connection_point);
    }
    add_point(foot_way_point);

    return conn;
  };

  w.r_->has_additional_connections_.resize(w.n_nodes());

  auto const& foot_components =
      w.r_->get_class_components(component_class::kFoot);
  auto const& street_components =
      w.r_->get_class_components(P::kStreetComponentClass);
  for (auto i = 0U; i != w.n_ways(); ++i) {
    auto const way_idx = way_idx_t{i};
    auto const p = w.r_->way_properties_[way_idx];

    if (!p.is_parking()) {
      continue;
    }
    auto const is_street_connected = is_connected(way_idx, street_components);
    auto const is_foot_connected = is_connected(way_idx, foot_components);
    if (is_street_connected && is_foot_connected) {
      // Already connected => Nothing to do
      continue;
    }

    auto const center = get_parking_space_center(way_idx);
    auto const approx_distance_lng_degrees =
        geo::approx_distance_lng_degrees(center);

    auto const loc = location{.pos_ = center, .lvl_ = kNoLevel};

    auto const street_matching =
        is_street_connected ? get_closest_connected_way(
                                  way_idx, center, approx_distance_lng_degrees,
                                  is_street_accessible)
                            : find_closest_offset<typename P::streetp>(
                                  w, l, loc, direction::kBackward,
                                  P::kStreetComponentClass, street_way_score);
    if (!street_matching.has_value()) {
      std::clog << fmt::format(
          "osr: parking space matching failed for street offset on way {} "
          "(center: {})\n",
          *w.get_osm_way(way_idx), center);
      continue;
    }

    // Assume street accessible ways are usually also foot accessible
    // => matching ways should be in the same (foot) component
    auto const is_same_foot_component =
        foot_components.get(street_matching->first.way_) ==
        foot_components.get(way_idx);
    auto const foot_matching =
        (is_foot_connected && is_same_foot_component)
            ? get_closest_connected_way(way_idx, center,
                                        approx_distance_lng_degrees,
                                        is_foot_accessible)
            : find_closest_offset<foot<false>>(w, l, loc, direction::kForward,
                                               component_class::kFoot,
                                               foot_way_score);
    if (!foot_matching.has_value()) {
      std::clog << fmt::format(
          "osr: parking space matching failed for foot offset on way {} "
          "(center: {})\n",
          *w.get_osm_way(way_idx), center);
      continue;
    }

    auto [foot_offset, foot_way_point] = *foot_matching;
    auto [street_offset, street_way_point] = *street_matching;
    if (!foot_offset.left_.valid() && !foot_offset.right_.valid()) {
      fmt::println("Connected footpath not usable! Way: {}  at: {}",
                   foot_offset.way_, foot_way_point);
      continue;
    }
    if (!street_offset.left_.valid() && !street_offset.right_.valid()) {
      fmt::println("Connected street is not usable! Way: {}  at: {}",
                   street_offset.way_, street_way_point);
      continue;
    }

    auto const street_connection_point =
        geo::approx_squared_distance_to_polyline(street_way_point,
                                                 w.way_polylines_[way_idx],
                                                 approx_distance_lng_degrees);
    auto const foot_connection_point = geo::approx_squared_distance_to_polyline(
        foot_way_point, w.way_polylines_[way_idx], approx_distance_lng_degrees);
    auto conn = make_connection(center, approx_distance_lng_degrees,
                                street_way_point, street_connection_point.best_,
                                foot_connection_point.best_, foot_way_point);
    add_additional_connection(w, std::move(street_offset),
                              std::move(foot_offset), std::move(conn), true);
  }
}

void connect_parking_ways(
    ways& w,
    lookup const& l,
    vec_map<way_idx_t, way_extra_properties> const& way_extra) {
  auto const is_car_accessible = [&](way_properties const& props) {
    return props.is_car_accessible();
  };
  auto const car_way_score = [&](double const dist_to_way,
                                 way_idx_t const way_idx) -> double {
    return score(dist_to_way, way_extra[way_idx].is_parking_aisle());
  };

  connect_parking_ways<car_parking<false, true>>(
      w, l, way_extra, is_car_accessible, car_way_score);
  // TODO: MK - Add bike_parking

  utl::sort(w.r_->additional_node_connections_);
  // Ensure no duplicates are stored
  auto const last = std::unique(begin(w.r_->additional_node_connections_),
                                end(w.r_->additional_node_connections_));
  w.r_->additional_node_connections_.erase(
      last, end(w.r_->additional_node_connections_));
}

}  // namespace osr
