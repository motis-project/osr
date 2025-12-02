#include "osr/routing/instructions/instruction_annotator.h"

namespace osr {

void instruction_annotator::annotate(osr::path& path) {
  for (std::size_t seg_idx{path.segments_.size()}; seg_idx > 0; seg_idx--) {
    set_relative_direction(path, seg_idx);
  }

}

void instruction_annotator::set_relative_direction(path& path,
                                                   std::size_t towards_idx) {
  utl_verify(towards_idx <= path.segments_.size(),
             "Index {} too large for given path size {}",
             towards_idx, path.segments_.size()
  );
  auto& current_segment = path.segments_[towards_idx - 1];
  if (towards_idx == path.segments_.size()) {
    // current_segment is the last segment and has no relative direction
    // to the not available next segment
    current_segment.instruction_annotation_.next_seg_direction_ = relative_direction::kInvalid;
    return;
  }
  // towards_idx < path.segments_.size()
  utl_verify(current_segment.polyline_.size() > 1,
             "Polyline must have at least 2 elements but contains only {}",
             current_segment.polyline_.size());

  const auto& last_coord = current_segment.polyline_.back();
  const auto& second_to_last_coord = current_segment.polyline_[current_segment.polyline_.size() - 2];

  auto const& towards_segment = path.segments_[towards_idx];
  utl_verify(towards_segment.polyline_.size() > 1,
           "Polyline must have at least 2 elements but contains only {}",
           towards_segment.polyline_.size());

  auto target_coord = towards_segment.polyline_.front();
  if (target_coord == last_coord) {
    target_coord = towards_segment.polyline_[1];
  }

  double angle_in_deg = get_angle(second_to_last_coord, last_coord, target_coord);
  current_segment.instruction_annotation_.next_seg_direction_ = get_relative_direction(angle_in_deg);
}



} // namespace osr
