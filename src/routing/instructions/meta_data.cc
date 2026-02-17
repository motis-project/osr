#include "osr/routing/instructions/meta_data.h"

#include "osr/routing/instructions/instruction_util.h"

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
    
    namespace {
    
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
                                        bool const is_uturn,
                                        std::vector<relative_way_segment>& left,
                                        std::vector<relative_way_segment>& right) {
        const auto& way_polyline = w.way_polylines_[ws.way_idx_];
    
        point const to = way_polyline[ws.osm_node_range_.to_];
        double const alt_angle = get_angle(prev_hub.as_latlng(), hub.as_latlng(), to.as_latlng());
        double const diff = alt_angle - exit_angle;
        if (double const relative_diff = normalize(diff); relative_diff < 0) {
          right.emplace_back(
            relative_diff,
            is_accessible(w, ws, m),
            is_uturn,
            ws
          );
        } else {
          left.emplace_back(
            relative_diff,
            is_accessible(w, ws, m),
            is_uturn,
            ws
          );
        }
      }
    
      struct hub_geometry {
        point from_;
        point hub_;
        point to_;
      };
    
      hub_geometry get_hub_geometry(ways const& w, way_segment const& arrive,
                                    way_segment const& exit) {
        const auto& arrive_polyline = w.way_polylines_[arrive.way_idx_];
        const auto& exit_polyline = w.way_polylines_[exit.way_idx_];
    
        const point hub = arrive_polyline[arrive.osm_node_range_.to_];
    
        const auto arrive_prev_idx = static_cast<std::uint16_t>(
            arrive.osm_node_range_.to_ + (arrive.is_way_aligned() ? -1 : 1));
        const point from = arrive_polyline[arrive_prev_idx];
    
        const auto exit_next_idx = static_cast<std::uint16_t>(
            exit.osm_node_range_.from_ + (exit.is_way_aligned() ? 1 : -1));
        const point to = exit_polyline[exit_next_idx];
    
        return {from, hub, to};
      }
    
      void add_alternatives(ways const& w, node_idx_t const hub_node,
                            osm_node_idx_t const osm_hub_node,
                            way_segment const& arrive_hub_on,
                            way_segment const& exit_from_hub,
                            way_idx_t const exit_way_idx, double const exit_angle,
                            point const from_point, point const hub_point,
                            mode const m, std::vector<relative_way_segment>& left,
                            std::vector<relative_way_segment>& right) {
        const auto& alt_ways = w.r_->node_ways_[hub_node];
        for (const auto alt_way : alt_ways) {
          const auto& osm_nodes = w.way_osm_nodes_[alt_way];
          if (osm_nodes.front() == osm_nodes.back()) {
            // Ignore circular ways for now
            continue;
          }
    
          const auto it = std::ranges::find(osm_nodes, osm_hub_node);
          if (it == osm_nodes.end()) {
            continue;
          }
    
          const auto hub_node_idx =
              static_cast<unsigned long>(std::distance(osm_nodes.begin(), it));
    
          if (hub_node_idx > 0 &&
              (alt_way != exit_way_idx || exit_from_hub.is_way_aligned())) {
            way_segment alt_seg_rev = {
                .way_idx_ = alt_way,
                .osm_node_range_ = {
                    .from_ = static_cast<std::uint16_t>(hub_node_idx),
                    .to_ = static_cast<std::uint16_t>(hub_node_idx - 1)}};

            const bool is_uturn_alt = alt_way == arrive_hub_on.way_idx_ &&
                                      alt_seg_rev.is_way_aligned() ^ arrive_hub_on.is_way_aligned();

            emplace_relative_way_segment(w, exit_angle, from_point, hub_point,
                                         alt_seg_rev, m, is_uturn_alt, left, right);
          }
          if (hub_node_idx + 1 < osm_nodes.size() &&
              (alt_way != exit_way_idx || !exit_from_hub.is_way_aligned())) {
            way_segment alt_seg_forw = {
                .way_idx_ = alt_way,
                .osm_node_range_ = {
                    .from_ = static_cast<std::uint16_t>(hub_node_idx),
                    .to_ = static_cast<std::uint16_t>(hub_node_idx + 1)}};

            const bool is_uturn_alt = alt_way == arrive_hub_on.way_idx_ &&
                          alt_seg_forw.is_way_aligned() ^ arrive_hub_on.is_way_aligned();

            emplace_relative_way_segment(w, exit_angle, from_point, hub_point,
                                         alt_seg_forw, m, is_uturn_alt, left, right);
          }
        }
      }
    
    } // namespace
    
    traversed_node_hub traversed_node_hub::from(ways const& w, path::segment const& arrive_on, path::segment const& exit_on) {
      utl::verify(arrive_on.to_ == exit_on.from_,
        "Given segments to not have common hub: {} vs. {}", arrive_on.to_, exit_on.from_);

      traversed_node_hub node_hub;
      node_hub.hub_node_ = arrive_on.to_;

      osm_node_idx_t const osm_prev_hub_node = w.node_to_osm_[arrive_on.from_];
      osm_node_idx_t const osm_hub_node = w.node_to_osm_[node_hub.hub_node_];
      osm_node_idx_t const osm_next_hub_node = w.node_to_osm_[exit_on.to_];

      node_hub.arrive_hub_on_ = way_segment::from(w, osm_prev_hub_node, osm_hub_node, arrive_on.way_);
      node_hub.exit_from_hub_ = way_segment::from(w, osm_hub_node, osm_next_hub_node, exit_on.way_);

      auto const geo = get_hub_geometry(w, node_hub.arrive_hub_on_, node_hub.exit_from_hub_);
      node_hub.exit_angle_ = get_angle(geo.from_.as_latlng(), geo.hub_.as_latlng(), geo.to_.as_latlng());

      add_alternatives(w, node_hub.hub_node_, osm_hub_node, node_hub.arrive_hub_on_, node_hub.exit_from_hub_,
                       exit_on.way_, node_hub.exit_angle_, geo.from_, geo.hub_, arrive_on.mode_,
                       node_hub.alts_left_, node_hub.alts_right_);

      std::ranges::sort(node_hub.alts_right_, std::ranges::greater{}, &relative_way_segment::angle_);
      std::ranges::sort(node_hub.alts_left_, std::ranges::less{}, &relative_way_segment::angle_);

      return node_hub;
    }

    void traversed_node_hub::print(std::ostream& out, ways const& w) const {
      out << "hub_node=" << w.node_to_osm_[hub_node_]
          << ", arrive_on=" << w.way_osm_idx_[arrive_hub_on_.way_idx_]
          << ", exit_from=" << w.way_osm_idx_[exit_from_hub_.way_idx_]
          << ", exit_angle=" << exit_angle_;

      auto const print_alts = [&](std::string_view name,
                                  std::vector<relative_way_segment> const& alts) {
        out << ", " << name << "=[";
        for (auto i = 0U; i < alts.size(); ++i) {
          auto const& alt = alts[i];
          out << "{way=" << w.way_osm_idx_[alt.segment_.way_idx_]
              << ", angle=" << alt.angle_
              << ", accessible=" << (alt.is_accessible_ ? "true" : "false") << "}";
          if (i != alts.size() - 1) {
            out << ", ";
          }
        }
        out << "]";
      };

      print_alts("alts_left", alts_left_);
      print_alts("alts_right", alts_right_);
    }

    // TODO
    bool traversed_node_hub::is_exit_natural_choice(ways const& w) const {
      if (! ways_have_same_name(arrive_hub_on_.way_idx_, exit_from_hub_.way_idx_, w)) {
        return false;
      }

      return true;
    }

} // namespace osr