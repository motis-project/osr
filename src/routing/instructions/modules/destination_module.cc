#include "osr/routing/instructions/modules/destination_module.h"

#include "osr/routing/instructions/directions.h"

namespace osr {

bool destination_module::process(ways const&, path& path, std::size_t const segment_idx, instruction_meta_data const&) {
  if (segment_idx == path.segments_.size() - 1) {
    auto& segment = path.segments_[segment_idx];
    segment.instruction_annotation_ = instruction_action::kDestination;
    return true;
  }
  return false;
}

} // namespace osr
