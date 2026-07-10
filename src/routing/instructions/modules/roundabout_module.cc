#include "osr/routing/instructions/modules/roundabout_module.h"

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"

namespace osr {

void roundabout_module::process(ways const& w,
                                path& p,
                                segment_contexts_window const& window) {
  trace("┃ ╔ ROUNDABOUT MODULE\n");
  if (is_last_segment(window)) {
    trace("┃ ║ is last segment, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }
  auto& segment = *window.focus()->src_ptr_;
  auto const& next_segment = *window[1]->src_ptr_;

  auto const current_is_roundabout = is_roundabout_way(w, segment.way_);
  auto const next_is_roundabout = is_roundabout_way(w, next_segment.way_);

  if (!current_is_roundabout && next_is_roundabout) {
    trace("┃ ╚ entering roundabout -> annotation=kEnterRoundabout\n");
    segment.instruction_annotation_ = enter_roundabout_instruction{};
    return;
  }

  if (current_is_roundabout && !next_is_roundabout) {
    trace("┃ ╚ exiting roundabout -> annotation=kExitRoundabout\n");
    segment.instruction_annotation_ = exit_roundabout_instruction{};
    return;
  }

  if (current_is_roundabout && next_is_roundabout) {
    segment.instruction_annotation_ = none_instruction{};
    return;
  }

  trace("┃ ╚ not responsible, passing to next module\n");
  instruction_module::process(w, p, window);
}

void roundabout_module::post_process(ways const& w,
                                     path&,
                                     segment_contexts const& contexts) {

  bool in_roundabout = false;
  bool is_clockwise = false;
  std::uint16_t n_exits_encountered = 0U;
  for (const auto& context : contexts) {
    auto& current_segment = *context.src_ptr_;
    const auto& hub_opt = context.traversed_node_hub_;
    if (!hub_opt.has_value()) {
      continue;
    }

    if (std::holds_alternative<enter_roundabout_instruction>(
            current_segment.instruction_annotation_)) {
      in_roundabout = true;

      const auto& hub = hub_opt.value();
      const auto& exit = hub.get_exit();
      is_clockwise = exit.angle_with_arrive_ > 0;
      n_exits_encountered = 0U;
      continue;
    }

    if (std::holds_alternative<exit_roundabout_instruction>(
        current_segment.instruction_annotation_)) {
      if (!in_roundabout) {
        continue;
      }
      in_roundabout = false;
      std::uint16_t const exit = n_exits_encountered + 1U;
      current_segment.instruction_annotation_ =
          exit_roundabout_instruction{exit};
      continue;
    }

    if (in_roundabout && hub_opt.has_value()) {
      const auto& hub = hub_opt.value();
      bool has_non_roundabout_exit = false;
      for (auto const& alt : hub.get_exit_neighbors(is_clockwise)) {
        if (alt.can_exit_hub() &&
            !is_roundabout_way(w, alt.segment_.way_idx_)) {
          has_non_roundabout_exit = true;
          break;
        }
      }
      if (has_non_roundabout_exit) {
        ++n_exits_encountered;
      }
    }
  }
}

} // namespace osr
