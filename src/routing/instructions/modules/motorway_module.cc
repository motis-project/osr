#include "osr/routing/instructions/modules/motorway_module.h"

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"
#include "osr/routing/instructions/routing_instruction.h"

namespace osr {

bool handle_from_motorway(ways const& w,
                          traversed_node_hub const& hub,
                          path::segment& seg) {
  const auto num_visible_turns = hub.number_of_visible_turns(true);

  const auto get_continue_index = [&](traversed_node_hub const& h) {
    for (auto i = 1U; i < h.alternatives_.size(); ++i) {
      const auto& alt = h.alternatives_[i];

      if (!alt.can_exit_hub()) {
        continue;
      }

      const auto arrive_way = h.arrive_hub_on_.way_idx_;
      const auto alt_way = alt.segment_.way_idx_;

      const bool is_same_name = ways_have_same_name(arrive_way, alt_way, w);
      const bool is_same_highway =
          ways_have_same_highway(arrive_way, alt_way, w);

      if (is_same_name && is_same_highway && !is_link(w, alt_way)) {
        return static_cast<int>(i);
      }
    }

    return -1;
  };

  const auto get_most_likely_continue_index = [&](traversed_node_hub const& h) {
    double best = 180.0;
    auto best_index = -1;

    for (auto i = 1U; i < h.alternatives_.size(); ++i) {
      const auto& alt = h.alternatives_[i];
      const auto alt_way = alt.segment_.way_idx_;

      if (!alt.can_exit_hub() || is_link(w, alt_way) ||
          !is_motorway_or_trunk(w, alt_way)) {
        continue;
      }

      if (alt.angle_with_arrive_ < best) {
        best = alt.angle_with_arrive_;
        best_index = static_cast<int>(i);
      }
    }

    return best_index;
  };

  const auto get_best_continue_index = [&](traversed_node_hub const& h) {
    if (const auto continue_index = get_continue_index(h);
        continue_index >= 0) {
      return continue_index;
    }

    return get_most_likely_continue_index(h);
  };

  const auto best_continue_index = get_best_continue_index(hub);

  if (best_continue_index < 0) {
    const auto& alts = hub.alternatives_;

    if (num_visible_turns == 2U) {
      // Ramp at the end of the highway
      seg.instruction_annotation_ = none_instruction{};
      return true;
    } else if (num_visible_turns == 3U) {
      if (alts[1].can_exit_hub() && alts[2].can_exit_hub()) {
        // splitting Ramp at the end of highway
        const auto fork_instructions = get_instruction_for_fork(w, hub, 1, 2);
        seg.instruction_annotation_ = hub.exit_from_hub_idx_ == 1
                                          ? fork_instructions.first
                                          : fork_instructions.second;
      } else {
        seg.instruction_annotation_ = none_instruction{};
      }
      return true;
    } else if (num_visible_turns == 4U) {
      // triple  fork at the end of highway
      const auto fork_instruction = get_instruction_for_fork(w, hub, 1, 2, 3);
      seg.instruction_annotation_ = hub.exit_from_hub_idx_ == 1
                                  ? std::get<0>(fork_instruction)
                                    : hub.exit_from_hub_idx_ == 2 ?
                                      std::get<1>(fork_instruction) :
                                      std::get<2>(fork_instruction);
      return true;
    }

    return false;
  }

  const auto& exit = hub.get_exit();
  if (is_link(w, exit.segment_.way_idx_)) {
    if (hub.exit_from_hub_idx_ > static_cast<unsigned>(best_continue_index)) {
      seg.instruction_annotation_ = ramp_off_instruction{relative_direction::kRight};
    } else { // hub.exit_from_hub_idx_ < best_continue_index
      seg.instruction_annotation_ = ramp_off_instruction{relative_direction::kLeft};
    }
    return true;
  }

  const auto n_exiting_motorways = // >= 1 because continuation exists
    std::ranges::count_if(hub.alternatives_, [&](const auto& alt) {
      const auto way = alt.segment_.way_idx_;
      return alt.can_exit_hub() && is_motorway_or_trunk(w, way) &&
             !is_link(w, way);
    });

  if (n_exiting_motorways > 1 &&
      is_motorway_or_trunk(w, exit.segment_.way_idx_)) {

    std::vector<size_t> exiting_motorways;

    for (size_t i = 1U; i < hub.alternatives_.size(); ++i) {
      const auto& candidate = hub.alternatives_[i];
      const auto candidate_way = candidate.segment_.way_idx_;
      if (candidate.can_exit_hub() &&
          is_motorway_or_trunk(w, candidate_way) &&
          !is_link(w, candidate_way)) {
        exiting_motorways.push_back(i);
      }
    }

    // exit must be in exiting_motorways
    if (exiting_motorways.size() == 2U) {
      const auto fork_instructions = get_instruction_for_fork(
          w, hub, exiting_motorways[0], exiting_motorways[1]);

      if (exiting_motorways[0] == hub.exit_from_hub_idx_) {
        seg.instruction_annotation_ = fork_instructions.first;
      } else if (exiting_motorways[1] == hub.exit_from_hub_idx_) {
        seg.instruction_annotation_ = fork_instructions.second;
      }
    } else if (exiting_motorways.size() == 3U) {
      const auto [il, ic, ir] =
          get_instruction_for_fork(w, hub, exiting_motorways[0],
                                   exiting_motorways[1], exiting_motorways[2]);
      if (exiting_motorways[0] == hub.exit_from_hub_idx_) {
        seg.instruction_annotation_ = il;
      } else if (exiting_motorways[1] == hub.exit_from_hub_idx_) {
        seg.instruction_annotation_ = ic;
      } else if (exiting_motorways[2] == hub.exit_from_hub_idx_) {
        seg.instruction_annotation_ = ir;
      }
    }

    return true;
  }

  return false;
}

bool handle_from_ramp(ways const& w,
                      traversed_node_hub const& h,
                      path::segment& seg) {
  const auto num_possible_turns = h.number_of_possible_turns(true);
  const auto num_visible_turns = h.number_of_visible_turns(true);

  // ramp straight into a motorway/ramp
  if (num_visible_turns == 2U && num_possible_turns == 1U) {
    seg.instruction_annotation_ = none_instruction{};
    return true;
  } else if (num_visible_turns == 3U) {

    if (num_possible_turns == 1U && !is_onto_ramp(w, h)) {
      // merging onto a passing highway
      if (h.exit_from_hub_idx_ == 2U) {
        // exit is second clockwise -> Merge left
        seg.instruction_annotation_ = ramp_on_instruction{relative_direction::kLeft};
      } else {
        // merge right
        seg.instruction_annotation_ = ramp_on_instruction{relative_direction::kRight};
      }
      return true;
    } else if (num_possible_turns == 2U) {
      // num_possible_turns == 2
      // -> Ramp fork
      if (h.exit_from_hub_idx_ == 1U) {
        seg.instruction_annotation_ = stay_instruction{relative_direction::kLeft};
      } else {
        // h.exit_from_hub_idx_ == 2U
        seg.instruction_annotation_ = stay_instruction{relative_direction::kRight};
      }
      return true;
    }
  }

  return false;
}

void motorway_module::process(ways const& w,
                              path& p,
                              segment_contexts_window const& window) {
  trace("┃ ╔ MOTORWAY MODULE\n");
  if (is_last_segment(window)) {
    trace("┃ ║ is last segment, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& segment_context = *window.focus();
  if (!segment_context.traversed_node_hub_.has_value()) {
    trace("┃ ║ no hub, cannot annotate, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& hub = segment_context.traversed_node_hub_.value();
  bool annotated = false;
  if (is_from_motorway(w, hub) &&
      (is_onto_motorway(w, hub) || is_onto_ramp(w, hub))) {
    annotated = handle_from_motorway(w, hub, *segment_context.src_ptr_);
  }
  if (is_from_ramp(w, hub) &&
      (is_onto_motorway(w, hub) || is_onto_ramp(w, hub))) {
    annotated = handle_from_ramp(w, hub, *segment_context.src_ptr_);
  }

  if (annotated) {
    return;
  }

  trace("┃ ╚ not responsible, passing to next module\n");
  instruction_module::process(w, p, window);
}

}  // namespace osr
