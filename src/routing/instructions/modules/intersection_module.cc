#include "osr/routing/instructions/modules/intersection_module.h"

#include "osr/routing/instructions/directions.h"

namespace osr {

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

bool is_simple_u_turn(traversed_node_hub const& hub) {
  const auto& arrive_on = hub.arrive_hub_on_;
  const auto& exit_on = hub.exit_from_hub_;

  return arrive_on.way_idx_ == exit_on.way_idx_ &&
         (arrive_on.is_way_aligned() ^ exit_on.is_way_aligned());
}

unsigned long number_of_possible_turns(traversed_node_hub const& hub) {
  constexpr auto is_possible = [](relative_way_segment const& rel_seg) {
    return rel_seg.is_accessible_ && !rel_seg.is_opposite_of_arrive_;
  };
  const auto count_left = std::ranges::count_if(hub.alts_left_, is_possible);
  const auto count_right = std::ranges::count_if(hub.alts_right_, is_possible);
  return static_cast<unsigned>(count_left + count_right + 1); // +1 because of the actual exit
}

unsigned long number_of_visible_turns(traversed_node_hub const& hub) {
  // This is simply #alternatives + 1, and eventually -1 if the
  // u-turn alternative is possible
  // TODO This count can be combined with number_of_possible_turns
  constexpr auto is_visible = [](relative_way_segment const& rel_seg) {
    return !rel_seg.is_opposite_of_arrive_;
  };
  const auto count_left = std::ranges::count_if(hub.alts_left_, is_visible);
  const auto count_right = std::ranges::count_if(hub.alts_right_, is_visible);
  return static_cast<unsigned>(count_left + count_right + 1);
}

bool intersection_module::process(ways const&,
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
    // Not sure in which scenario
    // this actually applies
    set_segment_instruction(segment, instruction_action::kTurnAround);
  }

  const auto num_possible_turns = number_of_possible_turns(common_node_hub);
  const auto num_visible_turns = number_of_visible_turns(common_node_hub);

  const relative_direction next_seg_rel_dir = segment_context.relative_direction_;
  const instruction_action possible_turn_type = turn_action_from(next_seg_rel_dir);

  if (num_possible_turns <= 1) {
    // The next segment is the only possible way to go
    if (num_visible_turns <= 1 || of_slight_type(next_seg_rel_dir)) {
      // No other visible turn options or only slight direction change
      // -> no turn instruction
      set_segment_instruction(segment, instruction_action::kContinue);
      return true;
    }

    // At least one other visible turn option and direction change is more
    // than slight -> turn instruction
    set_segment_instruction(segment, possible_turn_type);
    return true;
  }

  // Number of possible turns > 1
  // -> There is at least one other viable option to turn into


  set_segment_instruction(segment, possible_turn_type); // Fallback for now
  return true;
}

} // namespace osr
