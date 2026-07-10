#include "osr/routing/instructions/instruction_annotator.h"

#include "osr/debug.h"
#include "osr/routing/path.h"
#include "utl/verify.h"

#include "osr/routing/instructions/directions.h"
#include "osr/routing/instructions/instruction_util.h"
#include "osr/routing/instructions/modules/destination_module.h"
#include "osr/routing/instructions/modules/intersection_module.h"
#include "osr/routing/instructions/modules/motorway_module.h"
#include "osr/routing/instructions/modules/pedestrian_module.h"
#include "osr/routing/instructions/modules/roundabout_module.h"
#include "osr/util/sliding_window.h"

namespace osr {

instruction_annotator::instruction_annotator(ways const& w)
  : ways_(w)
  {
    init_chain();
  }

void instruction_annotator::init_chain() {
  auto destination_module_ptr = std::make_shared<destination_module>();
  auto roundabout_module_ptr = std::make_shared<roundabout_module>();
  auto motorway_module_ptr = std::make_shared<motorway_module>();
  auto pedestrian_module_ptr = std::make_shared<pedestrian_module>();
  auto intersection_module_ptr = std::make_shared<intersection_module>();

  destination_module_ptr
    ->set_next(roundabout_module_ptr)
    ->set_next(motorway_module_ptr)
    ->set_next(pedestrian_module_ptr)
    // Fallback
    ->set_next(intersection_module_ptr);
  head_module_ = destination_module_ptr;
  modules_.emplace_back(std::move(destination_module_ptr));
  modules_.emplace_back(std::move(roundabout_module_ptr));
  modules_.emplace_back(std::move(motorway_module_ptr));
  modules_.emplace_back(std::move(pedestrian_module_ptr));
  modules_.emplace_back(std::move(intersection_module_ptr));
}

bool can_be_annotated(segment_contexts_window const& window) {
  if (is_last_segment(window)) {
    return true;
  }

  const auto& segment_context = *window.focus();
  const auto& next_segment_context = *window[1];
  const auto& segment = *segment_context.src_ptr_;
  const auto& next_segment = *next_segment_context.src_ptr_;

  if (segment.to_ == node_idx_t::invalid() ||
      next_segment.from_ == node_idx_t::invalid() ||
      segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid() ||
      !segment_context.traversed_node_hub_.has_value()) {
    return false;
      }

  if (segment.to_ != next_segment.from_) {
    return false;
  }

  return true;
}

void instruction_annotator::annotate(path& path) {
  std::vector<segment_context> segment_contexts;
  preprocess(path, segment_contexts);

  const auto context_windows = sliding_window<1>(segment_contexts);
  for (const auto cw : context_windows) {
    if (!can_be_annotated(cw)) {
      continue;
    }
    trace("Matching context window against modules:\n");
    head_module_->process(ways_, path, cw);
  }

  for (const auto& m : modules_) {
    m->post_process(ways_, path, segment_contexts);
  }
}

void instruction_annotator::preprocess(path& path,
                                       std::vector<segment_context>& meta) {

  auto& segments = path.segments_;
  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    auto const& segment = segments[i];
    auto const& next_segment = segments[i + 1];

    meta.emplace_back(&segments[i],
                      get_traversed_node_hub(segment, next_segment, ways_));
  }

  if (segments.size() > 1) {
    // Sentinel for the last segment
    meta.emplace_back(&segments.back(),
                      std::nullopt);
  }
}

} // namespace osr
