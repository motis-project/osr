#include "osr/routing/instructions/instruction_util.h"

#include "osr/routing/instructions/module.h"

namespace osr {

bool is_from_motorway(ways const& w, traversed_node_hub const& hub) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[hub.arrive_hub_on_.way_idx_];

  return is_motorway_or_trunk(w, hub.arrive_hub_on_.way_idx_) &&
         !segment_instruction_properties.is_link();
}

bool is_from_ramp(ways const& w, traversed_node_hub const& hub) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[hub.arrive_hub_on_.way_idx_];

  return is_motorway_or_trunk(w, hub.arrive_hub_on_.way_idx_) &&
         segment_instruction_properties.is_link();
}

bool is_onto_motorway(ways const& w, traversed_node_hub const& hub) {
  const auto& exit_from_hub = hub.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_hub.segment_.way_idx_];

  return is_motorway_or_trunk(w, exit_from_hub.segment_.way_idx_) &&
         !segment_instruction_properties.is_link();
}

bool is_onto_ramp(ways const& w, traversed_node_hub const& hub) {
  const auto& exit_from_hub = hub.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_hub.segment_.way_idx_];

  return is_motorway_or_trunk(w, exit_from_hub.segment_.way_idx_) &&
         segment_instruction_properties.is_link();
}


bool is_last_segment(segment_contexts_window const& window) {
  return !window.has(1);
}

bool ways_have_same_name(way_idx_t const w1, way_idx_t const w2, ways const& w) {
  const string_idx_t s1 = w.way_names_[w1];
  if (s1 == string_idx_t::invalid()) {
    return false;
  }

  const string_idx_t s2 = w.way_names_[w2];
  if (s2 == string_idx_t::invalid()) {
    return false;
  }

  return s1 == s2;
}

bool ways_have_same_highway(way_idx_t const w1,
                            way_idx_t const w2,
                            ways const& w) {
  return w.way_instruction_properties_[w1].get_highway() ==
         w.way_instruction_properties_[w2].get_highway();
}

bool is_roundabout_way(ways const& w, way_idx_t const way) {
  if (way == way_idx_t::invalid()) {
    return false;
  }
  const auto junction = w.way_instruction_properties_[way].get_junction();
  return junction == junction::roundabout ||
         junction == junction::circular;
}

bool is_link(ways const& w, way_idx_t const way) {
  return w.way_instruction_properties_[way].is_link();
}

bool is_motorway_or_trunk(ways const& w, way_idx_t const way) {
  const auto h = w.way_instruction_properties_[way].get_highway();
  return h == highway::motorway || h == highway::trunk;
}

bool turn_onto_through_street(ways const& w, traversed_node_hub const& hub) {

  const auto& exit = hub.get_exit();
  for (std::size_t alt_idx = 1U; alt_idx < hub.alternatives_.size(); ++alt_idx) {
    if (alt_idx == hub.exit_from_hub_idx_) {
      continue;
    }

    const auto& alt = hub.alternatives_[alt_idx];

    const bool is_nearly_straight = std::abs(alt.angle_with_exit_) > 160.0;
    const bool same_name =
        ways_have_same_name(alt.segment_.way_idx_,
                            exit.segment_.way_idx_, w);
    const bool same_highway =
        ways_have_same_highway(alt.segment_.way_idx_,
                               exit.segment_.way_idx_, w);

    if (is_nearly_straight && same_name && same_highway) {
      return true;
    }
  }

  return false;
}

bool is_more_important_by(highway const h1,
                          highway const h2,
                          unsigned const diff) {
  return h1 + diff <= h2;
}

std::pair<instruction_action, instruction_action> get_instruction_for_fork(
    ways const& w, traversed_node_hub const& hub, size_t const left, size_t const right) {
  utl::verify(left < right, "Alternatives are ordered in circular order");

  const auto& alt_left = hub.alternatives_[left];
  const auto& alt_right = hub.alternatives_[right];

  const auto alt_left_direction =
      get_relative_direction(alt_left.angle_with_arrive_);
  const auto alt_right_direction =
    get_relative_direction(alt_right.angle_with_arrive_);

  const auto& left_props =
    w.way_instruction_properties_[alt_left.segment_.way_idx_];
  const auto& right_props =
  w.way_instruction_properties_[alt_right.segment_.way_idx_];

  const auto left_highway = left_props.get_highway();
  const auto right_highway = right_props.get_highway();

  if (alt_left_direction == relative_direction::kStraight &&
      alt_right_direction != relative_direction::kStraight) {

    if (is_more_important_by(right_highway, left_highway, 1)) {
      return {instruction_action::kStayLeft, instruction_action::kStayRight};
    }

    return {instruction_action::kNone, instruction_action::kStayRight};
  } else if (alt_right_direction == relative_direction::kStraight &&
             alt_left_direction != relative_direction::kStraight) {

    if (is_more_important_by(left_highway, right_highway, 1)) {
      return {instruction_action::kStayLeft, instruction_action::kStayRight};
    }

    return {instruction_action::kStayLeft, instruction_action::kNone};
  }

  return {instruction_action::kStayLeft, instruction_action::kStayRight};
}

std::tuple<instruction_action, instruction_action, instruction_action>
get_instruction_for_fork(ways const& w,
                         traversed_node_hub const& hub,
                         size_t const left,
                         size_t const center,
                         size_t const right) {
  utl::verify(left < center && center < right,
              "Alternatives are ordered in circular order");

  const auto& alt_left = hub.alternatives_[left];
  const auto& alt_center = hub.alternatives_[center];
  const auto& alt_right = hub.alternatives_[right];

  if (alt_left.can_exit_hub() &&
      alt_center.can_exit_hub() &&
      alt_right.can_exit_hub()) {

    const auto alt_left_direction =
      get_relative_direction(alt_left.angle_with_arrive_);
    if (alt_left_direction == relative_direction::kStraight) {
      return {
        instruction_action::kStayStraight,
        instruction_action::kStaySlightRight,
        instruction_action::kStayRight
      };
    }

    const auto alt_right_direction =
      get_relative_direction(alt_right.angle_with_arrive_);
    if (alt_right_direction == relative_direction::kStraight) {
      return {
        instruction_action::kStayLeft,
        instruction_action::kStaySlightLeft,
        instruction_action::kStayStraight
      };
    }

    // center is somewhat straight
    return {
      instruction_action::kStayLeft,
      instruction_action::kStayStraight,
      instruction_action::kStayRight
    };
  }

  if (alt_left.can_exit_hub()) {
    if (alt_right.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, hub, left, right);
      return {
        fork_instructions.first,
        instruction_action::kNone,
        fork_instructions.second};
    }
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, hub, left, center);
      return {
        fork_instructions.first,
        fork_instructions.second,
        instruction_action::kNone
      };
    }

    return {
      instruction_action::kStayLeft,
      instruction_action::kNone,
      instruction_action::kNone
    };
  }

  if (alt_right.can_exit_hub()) {
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
        get_instruction_for_fork(w, hub, center, right);

      return {
        instruction_action::kNone,
        fork_instructions.first,
        fork_instructions.second
      };
    }

    return {
      instruction_action::kNone,
      instruction_action::kNone,
      instruction_action::kStayRight
    };
  }


  if (alt_center.can_exit_hub()) {
    return {
      instruction_action::kNone,
      instruction_action::kStayStraight,
      instruction_action::kNone
    };
  }

  return {
    instruction_action::kNone,
    instruction_action::kNone,
    instruction_action::kNone
  };
}

} // namespace osr