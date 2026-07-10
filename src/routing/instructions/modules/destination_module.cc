#include "osr/routing/instructions/modules/destination_module.h"

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"

namespace osr {

void destination_module::process(ways const& w,
                                 path& p,
                                 segment_contexts_window const& window) {
  trace("┃ ╔ DESTINATION MODULE\n");
  if (is_last_segment(window)) {
    trace("┃ ╚ inspecting last segment -> annotation=kDestination\n");
    const auto& segment_context = *window.focus();
    segment_context.src_ptr_->instruction_annotation_ =
        destination_instruction{};
    return;
  }

  trace("┃ ║ not responsible, passing to next module\n");
  instruction_module::process(w, p, window);
}

} // namespace osr
