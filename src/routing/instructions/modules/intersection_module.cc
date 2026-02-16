#include "osr/routing/instructions/modules/intersection_module.h"

#include "osr/routing/instructions/directions.h"

namespace osr {

bool intersection_module::process(ways const& w,
                                  path&,
                                  segment_contexts_window const& window) {
  if (const auto next_seg_exists = window.has(1); ! next_seg_exists) {
    return false;
  }

  const auto& segment_context = *window.focus();
  const auto& next_segment_context = *window[1];

  auto& segment = *segment_context.src_ptr_;
  const auto& next_segment = *next_segment_context.src_ptr_;

  if (segment.to_ == node_idx_t::invalid() ||
      next_segment.to_ == node_idx_t::invalid() ||
      segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid()) {
    return false;
  }

  if (segment.to_ != next_segment.from_) {
    return false;
  }
  const auto common_node = segment.to_;
  const auto& incident_ways = w.r_->node_ways_[common_node];
  const auto n_incident_ways = incident_ways.size();
  if (n_incident_ways <= 1) {
    return false;
  }


  const auto way_name_idx_from = w.way_names_[segment.way_];
  const auto way_name_idx_to = w.way_names_[next_segment.way_];
  if (way_name_idx_from == way_name_idx_to) {
    auto const& way_to_nodes = w.r_->way_nodes_[next_segment.way_];
    auto const common_node_iter = std::find(way_to_nodes.begin(), way_to_nodes.end(), common_node);

    utl::verify(common_node_iter != way_to_nodes.end(),
      "Expected node_idx {} to be in way_idx {}\n", common_node, next_segment.way_);

    if (common_node_iter == way_to_nodes.begin() &&
        std::next(common_node_iter) == way_to_nodes.end()) {
      // First or Last node in next way -> Continue
      segment.instruction_annotation_ = instruction_action::kContinue;
      return true;
    }
  }

  // Map relative direction to instruction action
  switch (auto const rel_dir = segment_context.relative_direction_; rel_dir) {
    case relative_direction::kStraight:
      segment.instruction_annotation_ = instruction_action::kContinue;
      break;
    case relative_direction::kSlightRight:
      segment.instruction_annotation_ = instruction_action::kTurnSlightRight;
      break;
    case relative_direction::kRight:
      segment.instruction_annotation_ = instruction_action::kTurnRight;
      break;
    case relative_direction::kSharpRight:
      segment.instruction_annotation_ = instruction_action::kTurnSharpRight;
      break;
    case relative_direction::kTurnAround:
      segment.instruction_annotation_ = instruction_action::kTurnAround;
      break;
    case relative_direction::kSharpLeft:
      segment.instruction_annotation_ = instruction_action::kTurnSharpLeft;
      break;
    case relative_direction::kLeft:
      segment.instruction_annotation_ = instruction_action::kTurnLeft;
      break;
    case relative_direction::kSlightLeft:
      segment.instruction_annotation_ = instruction_action::kTurnSlightLeft;
      break;
    case relative_direction::kInvalid:
      segment.instruction_annotation_ = instruction_action::kNone;
      break;
  }
  
  return true;
}

} // namespace osr
