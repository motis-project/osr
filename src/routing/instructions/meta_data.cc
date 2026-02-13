#include "osr/routing/instructions/meta_data.h"

namespace osr {

  bool way_segment::is_way_aligned() const {
    return osm_node_range_.from_ <= osm_node_range_.to_;
  }

  way_segment way_segment::from(const ways& w, const osm_node_idx_t from, osm_node_idx_t const to, way_idx_t const way) {

    const auto& way_osm_nodes = w.way_osm_nodes_[way];
    const auto it_from = std::ranges::find(way_osm_nodes, from);
    const auto it_to = std::ranges::find(way_osm_nodes, to);

    std::uint16_t from_idx, to_idx;
    from_idx = to_idx = static_cast<std::uint16_t>(way_osm_nodes.size());

    if (it_from != way_osm_nodes.end() && it_to != way_osm_nodes.end()) {
      from_idx = static_cast<std::uint16_t>(std::distance(way_osm_nodes.begin(), it_from));
      to_idx = static_cast<std::uint16_t>(std::distance(way_osm_nodes.begin(), it_to));
    }

    return {
      .way_idx_ = way,
      .osm_node_range_ = {
        .from_ = from_idx,
        .to_ = to_idx,
      }
    };
  }

  bool is_accessible(ways const& w, way_segment const& ws, mode const m) {

    const auto w_props = w.r_->way_properties_[ws.way_idx_];
    switch(m) {
      case mode::kBike:
        if (! w_props.is_bike_accessible()) return false;
        break;
      case mode::kCar:
        if (! w_props.is_car_accessible()) return false;
        break;
    case mode::kFerry:
        if (! w_props.is_ferry_accessible()) return false;
        break;
      case mode::kRailway:
        if (! w_props.is_railway_accessible()) return false;
        break;
      case mode::kFoot:
      case mode::kWheelchair:
        if (!w_props.is_foot_accessible()) return false;
    }

    if (m == mode::kFoot || m == mode::kWheelchair) {
      return true;
    }

    const auto travel_dir = ws.is_way_aligned() ? direction::kForward : direction::kBackward;
    if (m == mode::kBike && w_props.is_oneway_bike() &&
        travel_dir == direction::kBackward) {
      return false;
        }
    if (m == mode::kCar && w_props.is_oneway_car() &&
        travel_dir == direction::kBackward) {
      return false;
        }

    return true;
  }

  void emplace_relative_way_segment(ways const& w,
                                    double const exit_angle,
                                    point const prev_hub,
                                    point const hub,
                                    way_segment const& ws,
                                    mode const m,
                                    std::vector<relative_way_segment>& left,
                                    std::vector<relative_way_segment>& right) {
    const auto& way_polyline = w.way_polylines_[ws.way_idx_];

    point const to = way_polyline[ws.osm_node_range_.to_];
    double const alt_angle = get_angle(prev_hub.as_latlng(), hub.as_latlng(), to.as_latlng());
    double const diff = alt_angle - exit_angle;
    if (double const relative_diff = normalize(diff); relative_diff < 0) {
      left.emplace_back(
        relative_diff,
        is_accessible(w, ws, m),
        ws
      );
    } else {
      right.emplace_back(
        relative_diff,
        is_accessible(w, ws, m),
        ws
      );
    }
  }

  traversed_node_hub traversed_node_hub::from(ways const& w, path::segment const& arrive_on, path::segment const& exit_on) {
    traversed_node_hub node_hub;

    utl::verify(arrive_on.to_ == exit_on.from_,
      "Given segments to not have common hub: {} vs. {}", arrive_on.to_, exit_on.from_);
    node_hub.hub_node_ = arrive_on.to_;

    osm_node_idx_t const osm_prev_hub_node = w.node_to_osm_[arrive_on.from_];
    osm_node_idx_t const osm_hub_node = w.node_to_osm_[node_hub.hub_node_];
    osm_node_idx_t const osm_next_hub_node = w.node_to_osm_[exit_on.to_];

    node_hub.arrive_hub_on_ = way_segment::from(w, osm_prev_hub_node, osm_hub_node, arrive_on.way_);
    node_hub.exit_from_hub_ = way_segment::from(w, osm_hub_node, osm_next_hub_node, exit_on.way_);

    const auto& arrive_on_polyline = w.way_polylines_[arrive_on.way_];
    const auto& exit_on_polyline = w.way_polylines_[exit_on.way_];

    const point hub_point = arrive_on_polyline[node_hub.arrive_hub_on_.osm_node_range_.to_];
    const auto arrive_on_second_to_last_idx = static_cast<std::uint16_t>(
      node_hub.arrive_hub_on_.osm_node_range_.to_ + (node_hub.arrive_hub_on_.is_way_aligned() ? -1 : 1)
    );
    const point from_point = arrive_on_polyline[arrive_on_second_to_last_idx];

    const auto exit_on_first_after_hub_idx = static_cast<std::uint16_t>(
      node_hub.exit_from_hub_.osm_node_range_.from_ + (node_hub.exit_from_hub_.is_way_aligned() ? 1 : -1)
    );
    const point to_point = exit_on_polyline[exit_on_first_after_hub_idx];

    node_hub.exit_angle_ = get_angle(from_point.as_latlng(), hub_point.as_latlng(), to_point.as_latlng());

    const auto& alt_ways = w.r_->node_ways_[node_hub.hub_node_];
    for (const auto alt_way : alt_ways) {
      const auto& osm_nodes = w.way_osm_nodes_[alt_way];

      const auto hub_node_idx = static_cast<unsigned long>(std::distance(osm_nodes.begin(), std::ranges::find(osm_nodes, osm_hub_node)));
      if (hub_node_idx > 0 && (alt_way != exit_on.way_ || node_hub.exit_from_hub_.is_way_aligned())) {
        way_segment alt_seg_rev = {
          .way_idx_ = alt_way,
          .osm_node_range_ = {
            .from_ = static_cast<std::uint16_t>(hub_node_idx),
            .to_ = static_cast<std::uint16_t>(hub_node_idx - 1)
          }
        };
        emplace_relative_way_segment(w, node_hub.exit_angle_, from_point, hub_point, alt_seg_rev,
                                     arrive_on.mode_, node_hub.alts_left_, node_hub.alts_right_);
      }
      if (hub_node_idx + 1 < osm_nodes.size() && (alt_way != exit_on.way_ || !node_hub.exit_from_hub_.is_way_aligned())) {
        way_segment alt_seg_forw = {
          .way_idx_ = alt_way,
          .osm_node_range_ = {
            .from_ = static_cast<std::uint16_t>(hub_node_idx),
            .to_ = static_cast<std::uint16_t>(hub_node_idx + 1)
          }
        };
        emplace_relative_way_segment(w, node_hub.exit_angle_, from_point, hub_point, alt_seg_forw,
                             arrive_on.mode_, node_hub.alts_left_, node_hub.alts_right_);
      }
    }
    std::ranges::sort(node_hub.alts_right_, std::ranges::less{}, &relative_way_segment::angle_);
    std::ranges::sort(node_hub.alts_left_, std::ranges::greater{}, &relative_way_segment::angle_);

    return node_hub;
  }



}