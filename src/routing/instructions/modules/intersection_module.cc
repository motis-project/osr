#include "osr/routing/instructions/modules/intersection_module.h"

#include <cmath>
#include <optional>

#include "osr/routing/instructions/directions.h"
#include "osr/routing/instructions/instruction_util.h"

namespace osr {

using hub_t = traversed_node_hub;
using osm_nodes_range_t = hub_t::way_osm_nodes_idx_range;
using way_segment_t = hub_t::way_segment;
using relative_way_segment_t = hub_t::relative_way_segment;

bool of_slight_type(relative_direction const dir) {
  switch (dir) {
    case relative_direction::kStraight:
    case relative_direction::kSlightRight:
    case relative_direction::kSlightLeft:
      return true;
    default:
      return false;
  }
}

bool is_real_turn(relative_direction const dir) {
  return dir != relative_direction::kStraight &&
         dir != relative_direction::kSlightRight &&
         dir != relative_direction::kSlightLeft &&
         dir != relative_direction::kInvalid;
}

instruction_action turn_action_from(relative_direction dir) {
  // Map relative direction to instruction action
  switch (dir) {
    case relative_direction::kStraight:
      return instruction_action::kContinue;
    case relative_direction::kSlightRight:
      return instruction_action::kTurnSlightRight;
    case relative_direction::kRight:
      return instruction_action::kTurnRight;
    case relative_direction::kSharpRight:
      return instruction_action::kTurnSharpRight;
    case relative_direction::kTurnAround:
      return instruction_action::kTurnAround;
    case relative_direction::kSharpLeft:
      return instruction_action::kTurnSharpLeft;
    case relative_direction::kLeft:
      return instruction_action::kTurnLeft;
    case relative_direction::kSlightLeft:
      return instruction_action::kTurnSlightLeft;
    case relative_direction::kInvalid:
      return instruction_action::kNone;
  }
}

void set_segment_instruction(path::segment& seg,
                             instruction_action const action) {
  seg.instruction_annotation_ = action;
}

bool is_simple_u_turn(hub_t const& hub) {
  const auto& arrive_on = hub.arrive_hub_on_;
  const auto& exit_on = hub.exit_from_hub_;

  return arrive_on.way_idx_ == exit_on.way_idx_ &&
         (arrive_on.is_way_aligned() ^ exit_on.is_way_aligned());
}

unsigned long number_of_possible_turns(hub_t const& hub) {
  constexpr auto is_possible = [](relative_way_segment_t const& rel_seg) {
    return rel_seg.can_exit_hub_ && !rel_seg.is_opposite_of_arrive_;
  };
  const auto count_left = std::ranges::count_if(hub.alts_left_, is_possible);
  const auto count_right = std::ranges::count_if(hub.alts_right_, is_possible);
  return static_cast<unsigned>(count_left + count_right + 1); // +1 because of the actual exit
}

unsigned long number_of_visible_turns(hub_t const& hub) {
  // This is simply #alternatives + 1, and eventually -1 if the
  // u-turn alternative is possible
  // TODO This count can be combined with number_of_possible_turns
  constexpr auto is_visible = [](relative_way_segment_t const& rel_seg) {
    return !rel_seg.is_opposite_of_arrive_;
  };
  const auto count_left = std::ranges::count_if(hub.alts_left_, is_visible);
  const auto count_right = std::ranges::count_if(hub.alts_right_, is_visible);
  return static_cast<unsigned>(count_left + count_right + 1);
}

std::optional<relative_way_segment_t> get_other_continue(
    hub_t const& hub, double const cutoff_angle) {
  std::optional<relative_way_segment_t> best;

  auto const consider = [&](relative_way_segment_t const& alt) {
    if (!alt.can_exit_hub_ || alt.is_opposite_of_arrive_) {
      return;
    }
    double const abs_angle = std::abs(alt.angle_with_arrive_);
    if (abs_angle > std::abs(cutoff_angle)) {
      return;
    }

    if (!best.has_value() || abs_angle < std::abs(best->angle_with_arrive_)) {
      best = alt;
    }
  };

  for (auto const& alt : hub.alts_left_) {
    consider(alt);
  }
  for (auto const& alt : hub.alts_right_) {
    consider(alt);
  }

  return best;
}

bool outgoing_edges_are_slower_by_factor(ways const& w,
                                         hub_t const& hub,
                                         double const factor) {
  auto const chosen_speed =
      static_cast<double>(w.r_->way_properties_[hub.exit_from_hub_.way_idx_]
                              .max_speed_km_per_h());
  if (chosen_speed <= 0.0) {
    return false;
  }

  auto const is_relevant = [](relative_way_segment_t const& alt) {
    return alt.can_exit_hub_ && !alt.is_opposite_of_arrive_;
  };

  for (auto const& alt : hub.alts_left_) {
    if (!is_relevant(alt)) {
      continue;
    }
    auto const alt_speed = static_cast<double>(
        w.r_->way_properties_[alt.segment_.way_idx_].max_speed_km_per_h());
    if (alt_speed >= chosen_speed / factor) {
      return false;
    }
  }

  for (auto const& alt : hub.alts_right_) {
    if (!is_relevant(alt)) {
      continue;
    }
    auto const alt_speed = static_cast<double>(
        w.r_->way_properties_[alt.segment_.way_idx_].max_speed_km_per_h());
    if (alt_speed >= chosen_speed / factor) {
      return false;
    }
  }

  return true;
}

bool same_road_signature(ways const& w, way_idx_t const a, way_idx_t const b) {
  if (a == way_idx_t::invalid() || b == way_idx_t::invalid()) {
    return false;
  }

  if (!ways_have_same_name(a, b, w)) {
    return false;
  }

  auto const pa = w.way_instruction_properties_[a];
  auto const pb = w.way_instruction_properties_[b];
  return pa.get_highway() == pb.get_highway() &&
         pa.is_link() == pb.is_link();
}

bool is_major_road(way_instruction_properties const props) {
  auto const h = props.get_highway();
  return h == motorway || h == trunk || h == primary || h == secondary ||
         h == tertiary;
}

bool intersection_module::process(ways const& w,
                                  path&,
                                  segment_contexts_window const& window) {
  if (const auto next_seg_exists = window.has(1); ! next_seg_exists) {
    // this is the last segment.
    // -> Responsibility of the destination module
    return false;
  }

  const auto& segment_context = *window.focus();
  const auto& next_segment_context = *window[1];

  auto& segment = *segment_context.src_ptr_;
  const auto& next_segment = *next_segment_context.src_ptr_;

  if (segment.to_ != next_segment.from_) {
    return false;
  }

  if (segment.to_ == node_idx_t::invalid() ||
      next_segment.from_ == node_idx_t::invalid() ||
      segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid() ||
      ! segment_context.traversed_node_hub_.has_value()) {
    // Too little information to make an
    // educated decision
    return false;
  }

  const auto& common_node_hub = segment_context.traversed_node_hub_.value();

  if (is_simple_u_turn(common_node_hub)) {
    set_segment_instruction(segment, instruction_action::kTurnAround);
    return true;
  }

  const auto num_possible_turns = number_of_possible_turns(common_node_hub);
  const auto num_visible_turns = number_of_visible_turns(common_node_hub);

  const relative_direction next_seg_rel_dir = segment_context.relative_direction_;
  const instruction_action possible_turn_type = turn_action_from(next_seg_rel_dir);
  const bool same_name = ways_have_same_name(segment.way_, next_segment.way_, w);

  if (num_possible_turns <= 1) {
    // GH ignore-equivalent: keep instruction unset.
    if (!is_real_turn(next_seg_rel_dir) ||
        (num_visible_turns <= 1 && of_slight_type(next_seg_rel_dir))) {
      set_segment_instruction(segment, instruction_action::kNone);
      return true;
    }

    set_segment_instruction(segment, possible_turn_type);
    return true;
  }

  if (is_real_turn(next_seg_rel_dir)) {
    if (same_name && outgoing_edges_are_slower_by_factor(w, common_node_hub, 2.0)) {
      set_segment_instruction(segment, instruction_action::kNone);
      return true;
    }

    set_segment_instruction(segment, possible_turn_type);
    return true;
  }

  constexpr auto continue_abs_cutoff_angle = 60.;
  auto const other_continue =
      get_other_continue(common_node_hub, continue_abs_cutoff_angle);
  auto const outgoing_edges_are_slower =
      outgoing_edges_are_slower_by_factor(w, common_node_hub, 1.0);
  auto const leaving_current_street = !same_name;

  if (other_continue.has_value()) {
    auto const exit_props = w.way_instruction_properties_[next_segment.way_];
    auto const arrive_props = w.way_instruction_properties_[segment.way_];
    auto const other_props =
        w.way_instruction_properties_[other_continue->segment_.way_idx_];
    auto const link = exit_props.is_link();
    auto const prev_link = arrive_props.is_link();
    auto const other_link = other_props.is_link();

    if (is_major_road(exit_props)) {
      if ((exit_props.get_highway() == arrive_props.get_highway() && link == prev_link) &&
          (other_props.get_highway() != arrive_props.get_highway() ||
           other_link != prev_link)) {
        set_segment_instruction(segment, instruction_action::kNone);
        return true;
      }
    }

    auto const delta = common_node_hub.exit_angle_;
    auto const other_delta = other_continue->angle_with_arrive_;

    if (std::abs(delta) < 6 && std::abs(other_delta) > 10 && same_name) {
      set_segment_instruction(segment, instruction_action::kContinue);
      return true;
    }

    if (!same_name || !outgoing_edges_are_slower) {
      set_segment_instruction(segment, other_delta < delta
                                           ? instruction_action::kStayLeft
                                           : instruction_action::kStayRight);
      return true;
    }
  }

  if (!outgoing_edges_are_slower &&
      (std::abs(common_node_hub.exit_angle_) > 60 || leaving_current_street)) {
    set_segment_instruction(segment, possible_turn_type);
    return true;
  }

  set_segment_instruction(segment, instruction_action::kNone);
  return true;
}

} // namespace osr
