#include "osr/routing/instructions/instruction_annotator.h"

#include "osr/routing/path.h"
#include "utl/verify.h"

#include "osr/util/sliding_window.h"
#include "osr/routing/instructions/directions.h"
#include "osr/routing/instructions/modules/intersection_module.h"
#include "osr/routing/instructions/modules/destination_module.h"

namespace osr {

instruction_annotator::instruction_annotator(ways const& w)
  : ways_(w)
  {
    add_module(std::make_unique<destination_module>());
    add_module(std::make_unique<intersection_module>());
  }

void instruction_annotator::add_module(std::unique_ptr<instruction_module> m) {
  modules_.emplace_back(std::move(m));
}

void instruction_annotator::annotate(path& path) {
  std::vector<segment_context> segment_contexts;
  preprocess(path, segment_contexts);

  const auto context_windows = sliding_window<1>(segment_contexts);

  for (const auto cw : context_windows) {
    for (const auto& m : modules_) {
      if (m->process(ways_, path, cw)) {
        break;
      }
    }
  }
}

relative_direction get_relative_direction(path::segment const& from,
                                          path::segment const& to) {
  utl_verify(from.polyline_.size() > 1,
             "Polyline must have at least 2 elements but contains only {}",
             from.polyline_.size());
  utl_verify(to.polyline_.size() > 1,
             "Polyline must have at least 2 elements but contains only {}",
             to.polyline_.size());

  const auto& last_coord = from.polyline_.back();
  const auto& second_to_last_coord = from.polyline_[from.polyline_.size() - 2];

  auto target_coord = to.polyline_.front();
  if (target_coord == last_coord) {
    target_coord = to.polyline_[1];
  }

  const double angle_in_deg = get_angle(second_to_last_coord, last_coord, target_coord);
  return get_relative_direction(angle_in_deg);
}

std::optional<traversed_node_hub> get_traversed_node_hub(path::segment const& segment,
                                                         path::segment const& next_segment,
                                                         ways const& w) {
  if (segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid()) {
    return std::nullopt;
  }

  if (segment.from_ == node_idx_t::invalid() || segment.to_ == node_idx_t::invalid() ||
      next_segment.from_ == node_idx_t::invalid() || next_segment.to_ == node_idx_t::invalid()) {
    return std::nullopt;
  }

  return traversed_node_hub::from(w, segment, next_segment);
}


void instruction_annotator::preprocess(path& path,
                                       std::vector<segment_context>& meta) {

  auto& segments = path.segments_;
  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    auto const& segment = segments[i];
    auto const& next_segment = segments[i + 1];

    meta.emplace_back(&segments[i],
                      get_traversed_node_hub(segment, next_segment, ways_),
                      get_relative_direction(segment, next_segment));
  }

  if (segments.size() > 1) {
    // Sentinel for the last segment
    meta.emplace_back(&segments.back(),
                      std::nullopt,
                      relative_direction::kInvalid);
  }
}

} // namespace osr
