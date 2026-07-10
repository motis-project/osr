#include "osr/routing/instructions/meta_data.h"

#include "boost/json.hpp"

#include "osr/routing/instructions/instruction_util.h"
#include "utl/pipes/all.h"
#include "utl/pipes/transform.h"
#include "utl/pipes/vec.h"

namespace osr {

using hub_t = traversed_node_hub;
using osm_nodes_range_t = hub_t::way_osm_nodes_idx_range;
using way_segment_t = hub_t::way_segment;
using relative_way_segment_t = hub_t::relative_way_segment;

osm_nodes_range_t osm_nodes_range_t::flip() const {
  return {
      .from_ = to_,
      .to_ = from_,
  };
}

bool way_segment_t::is_way_aligned() const {
  return osm_node_range_.from_ <= osm_node_range_.to_;
}

distance_t get_min_exit_segment_distance_for_mode(ways const& w,
                                                  way_idx_t const way,
                                                  mode const m) {
  if (is_pedestrian(m)) {
    return 5;
  }
  const auto highway_type = w.way_instruction_properties_[way].get_highway();
  switch (highway_type) {
    case highway::motorway: return 100;
    case highway::trunk: return 80;
    case highway::primary:
    case highway::secondary: return 20;
    default: return 10;
  }
}

distance_t get_min_arrival_segment_distance_for_mode(ways const& w,
                                                     way_idx_t const way,
                                                     mode const m) {
  if (is_pedestrian(m)) {
    return 0;
  }
  const auto highway_type = w.way_instruction_properties_[way].get_highway();
  switch (highway_type) {
    case highway::motorway: return 30;
    case highway::trunk: return 20;
    case highway::primary:
    case highway::secondary: return 20;
    default: return 5;
  }
}

way_segment_t way_segment_t::from(const ways& w,
                                  way_idx_t const way,
                                  osm_node_idx_t const from,
                                  osm_node_idx_t const to,
                                  bool const is_way_aligned) {

  const auto& way_osm_nodes = w.way_osm_nodes_[way];

  std::uint16_t to_idx;
  std::uint16_t from_idx = to_idx =
      static_cast<std::uint16_t>(way_osm_nodes.size());

  const auto first_it =
      std::ranges::find(way_osm_nodes, is_way_aligned ? from : to);
  const auto second_it =
      std::find(first_it, way_osm_nodes.end(), is_way_aligned ? to : from);

  if (first_it != way_osm_nodes.end() && second_it != way_osm_nodes.end()) {
    from_idx = static_cast<std::uint16_t>(std::distance(
        way_osm_nodes.begin(), is_way_aligned ? first_it : second_it));
    to_idx = static_cast<std::uint16_t>(std::distance(
        way_osm_nodes.begin(), is_way_aligned ? second_it : first_it));
  }

  return {.way_idx_ = way,
          .osm_node_range_ = {
              .from_ = from_idx,
              .to_ = to_idx,
          }};
}

way_segment_t way_segment_t::flip_directions() const {
  return {.way_idx_ = way_idx_, .osm_node_range_ = osm_node_range_.flip()};
}

osm_node_idx_t way_segment_t::get_from_idx(ways const& w) const {
  return w.way_osm_nodes_[way_idx_][osm_node_range_.from_];
}

osm_node_idx_t way_segment_t::get_to_idx(ways const& w) const {
  return w.way_osm_nodes_[way_idx_][osm_node_range_.to_];
}

point way_segment_t::get_from_position(ways const& w) const {
  return w.way_polylines_[way_idx_][osm_node_range_.from_];
}

point way_segment_t::get_to_position(ways const& w) const {
  return w.way_polylines_[way_idx_][osm_node_range_.to_];
}


namespace {

bool is_accessible(ways const& w,
                   way_segment_t const& ws,
                   mode const m) {

  const auto w_props = w.r_->way_properties_[ws.way_idx_];
  switch (m) {
    case mode::kBike:
      if (!w_props.is_bike_accessible()) return false;
      break;
    case mode::kCar:
      if (!w_props.is_car_accessible()) return false;
      break;
    case mode::kFerry:
      if (!w_props.is_ferry_accessible()) return false;
      break;
    case mode::kRailway:
      if (!w_props.is_railway_accessible()) return false;
      break;
    case mode::kFoot:
    case mode::kWheelchair:
      if (!w_props.is_foot_accessible()) return false;
  }

  if (m == mode::kFoot || m == mode::kWheelchair) {
    return true;
  }

  const auto travel_dir =
      ws.is_way_aligned() ? direction::kForward : direction::kBackward;
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

void emplace_relative_way_segment(
    ways const& w,
    double const exit_angle,
    point const prev_hub,
    point const hub,
    way_segment_t const& ws,
    mode const m,
    bool const is_uturn,
    std::vector<relative_way_segment_t>& alternatives) {
  const auto& way_polyline = w.way_polylines_[ws.way_idx_];

  point const to = way_polyline[ws.osm_node_range_.to_];
  double const alt_angle =
      get_angle(prev_hub.as_latlng(), hub.as_latlng(), to.as_latlng());
  double const diff = alt_angle - exit_angle;

  bool const can_exit_hub = is_accessible(w, ws, m);
  bool const can_enter_hub = is_accessible(w, ws.flip_directions(), m);
  double const relative_diff = normalize(diff);
  alternatives.emplace_back(relative_diff, alt_angle, can_enter_hub,
                            can_exit_hub, is_uturn, ws);
}

struct hub_geometry {
  point from_;
  point hub_;
  point to_;
};

std::uint16_t get_next_osm_way_node_idx_or_last(ways const& w,
                                                way_idx_t const way_idx,
                                                std::uint16_t const osm_node_idx,
                                                direction const dir,
                                                distance_t const min_distance = 10) {
  const auto polyline = w.way_polylines_[way_idx];

  if (polyline.size() <= 1) {
    return 0;
  }

  if (dir == direction::kForward) {
    double accumulated_dist = 0.0;
    for (std::size_t i = osm_node_idx + 1; i < polyline.size(); ++i) {
      accumulated_dist += geo::distance(polyline[i - 1], polyline[i]);
      if (accumulated_dist >= min_distance) {
        return static_cast<std::uint16_t>(i);
      }
    }
    return static_cast<std::uint16_t>(polyline.size() - 1);
  } else {
    double accumulated_dist = 0.0;
    for (int i = static_cast<int>(osm_node_idx) - 1; i >= 0; --i) {
      accumulated_dist += geo::distance(polyline[static_cast<std::size_t>(i + 1)],
                                        polyline[static_cast<std::size_t>(i)]);
      if (accumulated_dist >= min_distance) {
        return static_cast<std::uint16_t>(i);
      }
    }
    return 0;
  }
}

hub_geometry get_hub_geometry(ways const& w,
                              way_segment_t const& arrive,
                              way_segment_t const& exit,
                              mode const m) {
  const auto& arrive_polyline = w.way_polylines_[arrive.way_idx_];
  const auto& exit_polyline = w.way_polylines_[exit.way_idx_];

  const point hub = arrive_polyline[arrive.osm_node_range_.to_];
  const auto arrive_prev_idx = get_next_osm_way_node_idx_or_last(w,
                                arrive.way_idx_, arrive.osm_node_range_.to_,
                                arrive.is_way_aligned() ?
                                  direction::kBackward : direction::kForward,
                                get_min_arrival_segment_distance_for_mode(w, arrive.way_idx_, m));
  utl::verify(arrive_prev_idx < arrive_polyline.size(), "out of bounds");
  const point from = arrive_polyline[arrive_prev_idx];
  const auto exit_next_idx = get_next_osm_way_node_idx_or_last(w,
                                exit.way_idx_, exit.osm_node_range_.from_,
                                exit.is_way_aligned() ?
                                  direction::kForward : direction::kBackward,
                                get_min_exit_segment_distance_for_mode(w, exit.way_idx_, m));
  utl::verify(exit_next_idx < exit_polyline.size(), "out of bounds");
  const point to = exit_polyline[exit_next_idx];

  return {from, hub, to};
}

void add_alternatives(ways const& w,
                      node_idx_t const hub_node,
                      osm_node_idx_t const osm_hub_node,
                      way_segment_t const& arrive_hub_on,
                      way_segment_t const& exit_from_hub,
                      way_idx_t const exit_way_idx,
                      double const exit_angle,
                      point const from_point,
                      point const hub_point,
                      mode const m,
                      std::vector<relative_way_segment_t>& alternatives) {
  const auto& alt_ways = w.r_->node_ways_[hub_node];
  for (const auto alt_way : alt_ways) {
    const auto& osm_nodes = w.way_osm_nodes_[alt_way];

    // Forward Search
    const auto forward_it = std::ranges::find(osm_nodes, osm_hub_node);
    if (forward_it == osm_nodes.end()) {
      continue;
    }

    const auto forward_hub_node_idx =
        static_cast<std::uint16_t>(std::distance(osm_nodes.begin(), forward_it));

    if (forward_hub_node_idx + 1U < osm_nodes.size() &&
        (alt_way != exit_way_idx || !exit_from_hub.is_way_aligned())) {
      way_segment_t alt_seg_forw = {
          .way_idx_ = alt_way,
          .osm_node_range_ = {
              .from_ = forward_hub_node_idx,
              .to_ = get_next_osm_way_node_idx_or_last(
                      w, alt_way, forward_hub_node_idx,
                      direction::kForward, get_min_exit_segment_distance_for_mode(w, alt_way, m))}};

      const bool is_uturn_alt =
          alt_way == arrive_hub_on.way_idx_ &&
          alt_seg_forw.is_way_aligned() ^ arrive_hub_on.is_way_aligned();

      emplace_relative_way_segment(w, exit_angle, from_point, hub_point,
                                   alt_seg_forw, m, is_uturn_alt, alternatives);
    }

    // Backward search
    const auto backward_it = std::find(osm_nodes.rbegin(), osm_nodes.rend(), osm_hub_node);
    if (backward_it == osm_nodes.rend()) {
      continue;
    }

    const auto backward_hub_node_idx = static_cast<std::uint16_t>(
        std::distance(osm_nodes.begin(), backward_it.base()) - 1);

    if (backward_hub_node_idx > 0 &&
        (alt_way != exit_way_idx || exit_from_hub.is_way_aligned())) {
      way_segment_t alt_seg_rev = {
          .way_idx_ = alt_way,
          .osm_node_range_ = {
              .from_ = backward_hub_node_idx,
              .to_ = get_next_osm_way_node_idx_or_last(
                      w, alt_way, backward_hub_node_idx,
                      direction::kBackward, get_min_exit_segment_distance_for_mode(w, alt_way, m))}};

      const bool is_uturn_alt =
          alt_way == arrive_hub_on.way_idx_ &&
          alt_seg_rev.is_way_aligned() ^ arrive_hub_on.is_way_aligned();

      emplace_relative_way_segment(w, exit_angle, from_point, hub_point,
                                   alt_seg_rev, m, is_uturn_alt, alternatives);
    }

  }
}

}  // namespace

traversed_node_hub traversed_node_hub::from(ways const& w,
                                            node_idx_t const prev_node,
                                            way_idx_t const arrive_way,
                                            node_idx_t const hub_node,
                                            way_idx_t const depart_way,
                                            node_idx_t const next_node,
                                            bool const is_arrive_way_aligned,
                                            bool const is_exit_way_aligned,
                                            mode const m) {
  traversed_node_hub node_hub;
  node_hub.hub_node_ = hub_node;

  osm_node_idx_t const osm_hub_node = w.node_to_osm_[node_hub.hub_node_];

  osm_node_idx_t const osm_prev_hub_node = w.node_to_osm_[prev_node];
  osm_node_idx_t const osm_next_hub_node = w.node_to_osm_[next_node];



  node_hub.arrive_hub_on_ = way_segment::from(
      w, arrive_way, osm_prev_hub_node, osm_hub_node, is_arrive_way_aligned);
  const auto exit_from_hub = way_segment::from(
      w, depart_way, osm_hub_node, osm_next_hub_node, is_exit_way_aligned);

  auto const geo =
      get_hub_geometry(w, node_hub.arrive_hub_on_, exit_from_hub, m);
  const auto exit_angle = get_angle(geo.from_.as_latlng(), geo.hub_.as_latlng(),
                                   geo.to_.as_latlng());

  add_alternatives(w, node_hub.hub_node_, osm_hub_node, node_hub.arrive_hub_on_,
                   exit_from_hub, depart_way, exit_angle, geo.from_, geo.hub_,
                   m, node_hub.alternatives_);

  const auto u_turn_seg =
      std::ranges::find_if(node_hub.alternatives_, [](const auto& rel_way_seg) {
        return rel_way_seg.is_opposite_of_arrive_;
      });

  if (u_turn_seg == node_hub.alternatives_.end()) {
    utl::fail("U turn segment not found");
  }

  if (u_turn_seg != node_hub.alternatives_.begin()) {
    std::iter_swap(node_hub.alternatives_.begin(), u_turn_seg);
  }

  std::ranges::sort(node_hub.alternatives_ | std::views::drop(1),
                    std::ranges::greater{},
                    &relative_way_segment::angle_with_arrive_);

  relative_way_segment exit_segment = {
    .angle_with_exit_ = 0.0,
    .angle_with_arrive_ = exit_angle,
    .can_enter_hub_ = is_accessible(w, exit_from_hub.flip_directions(), m),
    .can_exit_hub_ = true,
    .is_opposite_of_arrive_ = false,
    .segment_ = exit_from_hub
  };

  auto iter = std::ranges::upper_bound(
      node_hub.alternatives_ | std::views::drop(1),
      exit_segment.angle_with_arrive_, std::ranges::greater{},
      &relative_way_segment::angle_with_arrive_);

  auto const inserted_iter = node_hub.alternatives_.insert(iter, exit_segment);
  node_hub.exit_from_hub_idx_ = static_cast<unsigned>(
      std::distance(node_hub.alternatives_.begin(), inserted_iter));

  return node_hub;
}

traversed_node_hub::relative_way_segment traversed_node_hub::get_exit() const {
  return alternatives_[exit_from_hub_idx_];
}

unsigned long traversed_node_hub::number_of_possible_turns(
    bool const consider_uturn) const {
  constexpr auto is_possible = [](relative_way_segment_t const& rel_seg) {
    return rel_seg.can_exit_hub_;
  };
  // drop because we omit the u-turn
  const auto count = std::ranges::count_if(
      alternatives_ | std::views::drop(consider_uturn ? 0 : 1), is_possible);
  return static_cast<unsigned>(count);
}

unsigned long traversed_node_hub::number_of_visible_turns(
    bool const consider_uturn) const {
  return static_cast<unsigned>(alternatives_.size() - (consider_uturn ? 0 : 1));
}

double traversed_node_hub::angle_with_exit_from(
    ways const& w, std::size_t const alt_idx) const {
  utl::verify(alt_idx < alternatives_.size(), "Alternative out of bounds");
  if (alt_idx == exit_from_hub_idx_) {
    return 0.0;
  }

  const auto& from = alternatives_[alt_idx];
  const auto& exit = get_exit();

  // This must be get_to_idx because the direction
  // of alternative way segment at a hub is always from
  // hub
  const auto from_node_p = from.segment_.get_to_position(w);
  const auto hub_node_p = exit.segment_.get_from_position(w);
  const auto exit_node_p = exit.segment_.get_to_position(w);
  return get_angle(from_node_p.as_latlng(), hub_node_p.as_latlng(),
                   exit_node_p.as_latlng());
}

void traversed_node_hub::print(std::ostream& out, ways const& w) const {
  out << "hub_node=" << w.node_to_osm_[hub_node_]
      << ", arrive_on=" << w.way_osm_idx_[arrive_hub_on_.way_idx_];

  auto const print_alts = [&](std::string_view name,
                              std::vector<relative_way_segment> const& alts) {
    out << ", " << name << "=[";
    for (auto i = 0U; i < alts.size(); ++i) {
      auto const& alt = alts[i];
      out << "{way=" << w.way_osm_idx_[alt.segment_.way_idx_]
          << ", angle_with_exit=" << alt.angle_with_exit_
          << ", angle_with_arrive=" << alt.angle_with_arrive_
          << ", can_enter_hub=" << (alt.can_enter_hub_ ? "true" : "false")
          << ", can_exit_hub=" << (alt.can_exit_hub_ ? "true" : "false")
          << ", is_exit_taken=" << (i == exit_from_hub_idx_ ? "true" : "false")
          << "}";
      if (i != alts.size() - 1) {
        out << ", ";
      }
    }
    out << "]";
  };

  print_alts("alternatives", alternatives_);
}

// TODO
bool traversed_node_hub::is_exit_natural_choice(ways const& w) const {
  if (!ways_have_same_name(arrive_hub_on_.way_idx_,
                           get_exit().segment_.way_idx_, w)) {
    return false;
  }

  return true;
}

bool traversed_node_hub::is_end_of_turn_lane(ways const& w, bool left, bool const dedicated) const {
  constexpr auto is_dedicated_turn_lane = [](way_instruction_properties const& props,
                                         direction const dir,
                                         bool const l) -> bool {
    return (l ? props.has_left_turn_lane(dir)  && !props.has_right_turn_lane(dir)
              : props.has_right_turn_lane(dir) && !props.has_left_turn_lane(dir))
            && !props.has_through_lane(dir);
  };

  constexpr auto is_turn_lane = [](way_instruction_properties const& props,
                                       direction const dir,
                                       bool const l) -> bool {
    return l ? props.has_left_turn_lane(dir)
              : props.has_right_turn_lane(dir);

  };

  const auto applies = dedicated ? is_dedicated_turn_lane : is_turn_lane;

  direction const arrive_dir = arrive_hub_on_.is_way_aligned() ?
                    direction::kForward :
                    direction::kBackward;
  const auto arrive_props =
    w.way_instruction_properties_[arrive_hub_on_.way_idx_];
  const auto is_arrive_dedicated_turn_lane =
    applies(arrive_props, arrive_dir, left);
  if (!is_arrive_dedicated_turn_lane) {
    return false;
  }

  const auto& exit = get_exit();
  const auto exit_props = w.way_instruction_properties_[exit.segment_.way_idx_];
  direction const exit_dir = exit.segment_.is_way_aligned() ?
                  direction::kForward :
                  direction::kBackward;
  return !applies(exit_props,exit_dir , left);
}

bool traversed_node_hub::does_way_names_change(ways const& w) const {
  const auto arrive_way = arrive_hub_on_.way_idx_;
  const auto exit_way = get_exit().segment_.way_idx_;
  return ! ways_have_same_name(arrive_way, exit_way, w);
}

std::string traversed_node_hub::to_json(ways const& w) const {
  const auto way_segment_to_json = [&w](way_segment const& way_seg) {
    if (way_seg.way_idx_ == way_idx_t::invalid()) {
      return boost::json::object{{
                                     "way_id",
                                     0U,
                                 },
                                 {"node_range", boost::json::array{0, 0}}};
    }
    const auto& way_osm_nodes = w.way_osm_nodes_[way_seg.way_idx_];
    return boost::json::object{
        {"way_id", to_idx(w.way_osm_idx_[way_seg.way_idx_])},
        {"node_range",
         boost::json::array{
             to_idx(way_osm_nodes[way_seg.osm_node_range_.from_]),
             to_idx(way_osm_nodes[way_seg.osm_node_range_.to_])}}};
  };

  const auto relative_way_segment_to_json =
      [&way_segment_to_json](relative_way_segment const& rel_way_seg) {
        return boost::json::object{
            {"segment", way_segment_to_json(rel_way_seg.segment_)},
            {"angle_with_exit", rel_way_seg.angle_with_exit_},
            {"angle_with_arrive", rel_way_seg.angle_with_arrive_},
            {"can_enter_hub", rel_way_seg.can_enter_hub_},
            {"can_exit_hub", rel_way_seg.can_exit_hub_},
            {"is_opposite_of_arrive", rel_way_seg.is_opposite_of_arrive_}};
      };

  const auto rel_segments_to_json_array =
      [&relative_way_segment_to_json](
          std::vector<relative_way_segment> const& rel_segments) {
        return utl::all(rel_segments) |
               utl::transform([&](relative_way_segment const& rel_way_seg) {
                 return relative_way_segment_to_json(rel_way_seg);
               }) |
               utl::emplace_back_to<boost::json::array>();
      };


  return boost::json::serialize(boost::json::object{
    {"arrive_hub_on", way_segment_to_json(arrive_hub_on_)},
    {"hub_node", hub_node_ == node_idx_t::invalid() ? 0U : to_idx(w.node_to_osm_[hub_node_])},
    {"alternatives", rel_segments_to_json_array(alternatives_)},
    {"exit_from_hub_idx", exit_from_hub_idx_}
  });
}

bool relative_way_segment_t::can_enter_hub() const {
  return can_enter_hub_;
}

bool relative_way_segment_t::can_exit_hub() const {
  return can_exit_hub_;
}

bool relative_way_segment_t::is_opposite_of_arrive() const {
  return is_opposite_of_arrive_;
}

}  // namespace osr