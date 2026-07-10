#include "osr/routing/instructions/modules/pedestrian_module.h"

#include "osr/debug.h"
#include "osr/routing/instructions/instruction_util.h"

namespace osr {

void pedestrian_module::process(ways const& w,
                                path& p,
                                segment_contexts_window const& window) {
  trace("┃ ╔ PEDESTRIAN MODULE\n");
  if (is_last_segment(window)) {
    trace("┃ ║ is last segment, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& segment_context = *window.focus();
  const auto& next_segment_context = *window[1];
  const auto& next_segment = *next_segment_context.src_ptr_;

  if (!is_pedestrian(next_segment.mode_)) {
    trace("┃ ║ next segment is not foot or wheelchair, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto next_seg_way = get_way_from(next_segment_context);
  if (next_seg_way == way_idx_t::invalid()) {
    trace("┃ ║ way of next segment is invalid, passing to next module\n");
    instruction_module::process(w, p, window);
    return;
  }

  const auto& next_seg_instruction_props =
      w.way_instruction_properties_[next_seg_way];
  const auto& opt_hub = segment_context.traversed_node_hub_;

  if (next_seg_instruction_props.is_stairs()) {
    trace("┃ ║ next segment are stairs\n");
    if (opt_hub.has_value()) {
      const auto& hub = opt_hub.value();
      const auto& exit = hub.get_exit();

      bool const is_in_way_direction = exit.segment_.is_way_aligned();
      bool const is_stairs_up = is_in_way_direction ^ next_seg_instruction_props.is_stairs_down();

      segment_context.src_ptr_->instruction_annotation_ =
        stairs_instruction{
          .direction_ = is_stairs_up ? vertical_direction::kUp : vertical_direction::kDown,
          .relative_direction_ = get_relative_direction(exit.angle_with_arrive_),
        };
      trace("┃ ╚ annotation = {}\n", to_string(segment_context.src_ptr_->instruction_annotation_));
      return;
    }
  }

  if (opt_hub.has_value()) {
    trace("┃ ║ check if hub is crossing street\n");
    trace("┃ ║ trying to find crossed street\n");
    const auto opt_crossed_street = get_crossing_street(w, opt_hub.value());
    if (opt_crossed_street.has_value()) {
      trace("┃ ║ crossed street found, [{}]\n",
            w.strings_[opt_crossed_street.value()].view());
      segment_context.src_ptr_->instruction_annotation_ = crossing_instruction{
          .street_ = opt_crossed_street.value()};
      return;
    } else {
      trace("┃ ║ could not find crossed street\n");
    }
  }


  trace("┃ ║ not responsible, passing to next module\n");
  instruction_module::process(w, p, window);
}

void pedestrian_module::post_process(ways const& w,
                                     path& p,
                                     segment_contexts const&) {
  trace("┃ ╔ PEDESTRIAN MODULE POST PROCESSING\n");

  constexpr distance_t composite_crossing_cutoff = 25U;
  for (auto first_segment_it = p.segments_.begin();
       first_segment_it != p.segments_.end(); ++first_segment_it) {

    if (!std::holds_alternative<crossing_instruction>(
            first_segment_it->instruction_annotation_)) {
      continue;
    }
    const auto first_crossing = std::get<crossing_instruction>(
        first_segment_it->instruction_annotation_);
    const auto first_street = first_crossing.street_;
    if (first_street == string_idx_t::invalid()) {
      trace("┃ ║ found crossing but street name invalid, continue\n");
      continue;
    }
    trace("┃ ║ found crossing of {}\n",
          w.strings_[first_crossing.street_].view());

    trace("┃ ║ searching for second crossing\n");
    distance_t dist_to_first = 0U;
    for (auto second_segment_it = std::next(first_segment_it);
         second_segment_it != p.segments_.end(); ++second_segment_it) {
      dist_to_first += second_segment_it->dist_;
      if (dist_to_first > composite_crossing_cutoff) {
        trace("┃ ║ too far from first, stopping search\n");
        break;
      }

      if (!std::holds_alternative<crossing_instruction>(
        second_segment_it->instruction_annotation_)) {
        continue;
      }
      const auto second_crossing = std::get<crossing_instruction>(
          second_segment_it->instruction_annotation_);
      const auto second_street = second_crossing.street_;
      if (second_street == string_idx_t::invalid()) {
        trace(
            "┃ ║ found another crossing but street name invalid, stopping "
            "search\n");
        break;
      } else if (second_street != first_street) {
        trace(
            "┃ ║ found another crossing but street name different, stopping "
            "search\n");
        break;
      }

      trace("┃ ║ found another crossing of {}\n",
            w.strings_[second_street].view());
      trace("┃ ║ removing annotation, then continue search\n");
      second_segment_it->instruction_annotation_ = none_instruction{};
    }
  }

  constexpr distance_t composite_stairs_cutoff = 10U;
  for (auto first_segment_it = p.segments_.begin();
       first_segment_it != p.segments_.end(); ++first_segment_it) {
    if (!std::holds_alternative<stairs_instruction>(
        first_segment_it->instruction_annotation_)) {
      continue;
    }

    const auto first_stairs =
      std::get<stairs_instruction>(first_segment_it->instruction_annotation_);

    distance_t dist_to_first = 0U;
    for (auto second_segment_it = std::next(first_segment_it);
         second_segment_it != p.segments_.end(); ++second_segment_it) {
      dist_to_first += second_segment_it->dist_;
      if (dist_to_first > composite_stairs_cutoff) {
        trace("┃ ║ too far from first stairs, stopping search\n");
        break;
      }

      if (std::holds_alternative<none_instruction>(
        second_segment_it->instruction_annotation_)) {
        continue;
      }

      if (!std::holds_alternative<stairs_instruction>(
        second_segment_it->instruction_annotation_)) {
        break;
      }

      const auto second_stairs = std::get<stairs_instruction>(
          second_segment_it->instruction_annotation_);
      if (second_stairs.direction_ != first_stairs.direction_) {
        trace(
            "┃ ║ stairs have different vertical directions -> stopping search\n");
        break;
      }

      if (second_stairs.relative_direction_ != relative_direction::kStraight) {
        trace(
            "┃ ║ second stairs has direction modifier -> stopping search\n");
        break;
      }

      trace("┃ ║ found another straight stairs\n");
      trace("┃ ║ removing annotation, then continue search\n");
      second_segment_it->instruction_annotation_ = none_instruction{};
    }
  }
}

} // namespace osr
