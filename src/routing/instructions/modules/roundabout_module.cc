#include "osr/routing/instructions/modules/roundabout_module.h"

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"

namespace osr {

bool roundabout_module::can_process(ways const& w,
                                    path&,
                                    segment_contexts_window const& window) {
  if (!window.has(1)) {
    return false;
  }

  auto const& segment = *window.focus()->src_ptr_;
  auto const& next_segment = *window[1]->src_ptr_;

  auto const current_is_roundabout = is_roundabout_way(w, segment.way_);
  auto const next_is_roundabout = is_roundabout_way(w, next_segment.way_);
  if (!current_is_roundabout && !next_is_roundabout) {
    return false;
  }

  return true;
}

void roundabout_module::process(ways const& w,
                                path&,
                                segment_contexts_window const& window) {
  trace("┃ ╔ ROUNDABOUT MODULE\n");
  auto& segment = *window.focus()->src_ptr_;
  auto const& next_segment = *window[1]->src_ptr_;

  auto const current_is_roundabout = is_roundabout_way(w, segment.way_);
  auto const next_is_roundabout = is_roundabout_way(w, next_segment.way_);

  if (!current_is_roundabout && next_is_roundabout) {
    trace("┃ ╚ entering roundabout -> annotation=kEnterRoundabout\n");
    segment.instruction_annotation_ = instruction_action::kEnterRoundabout;
    return;
  }

  if (current_is_roundabout && !next_is_roundabout) {
    trace("┃ ╚ exiting roundabout -> annotation=kExitRoundabout\n");
    segment.instruction_annotation_ = instruction_action::kExitRoundabout;
    return;
  }

  if (current_is_roundabout && next_is_roundabout) {
    segment.instruction_annotation_ = instruction_action::kNone;
    return;
  }

  utl::fail("Module not responsible");
}

} // namespace osr
