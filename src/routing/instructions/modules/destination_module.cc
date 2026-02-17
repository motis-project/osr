#include "osr/routing/instructions/modules/destination_module.h"

#include "osr/routing/instructions/directions.h"

namespace osr {

bool destination_module::process(ways const&, path&, segment_contexts_window const& window) {
  const auto next_seg_exists = window.has(1);
  if (! next_seg_exists) {
    const auto& segment_context = *window.focus();
    segment_context.src_ptr_->instruction_annotation_ = instruction_action::kDestination;
  }
  return false;
}

} // namespace osr
