#include "osr/routing/instructions/instruction_annotator.h"

#include "utl/verify.h"

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
  instruction_meta_data meta;
  preprocess(path, meta);

  for (std::size_t i = 0; i < path.segments_.size(); ++i) {
    if (i == 0U || (path.segments_.size() >= 2 && i == path.segments_.size() - 2U)) {
      // The first and last segment are off-road.
      // Necessary information for annotation is therefore missing.
      continue;
    }
    for (const auto& m : modules_) {
      if (m->process(ways_, path, i, meta)) {
        break;
      }
    }
  }
}


void instruction_annotator::preprocess(path const& path, instruction_meta_data& meta) {
  meta.relative_directions_.resize(path.segments_.size(), relative_direction::kInvalid);
  meta.traversed_node_hubs_.resize(path.segments_.size(), std::nullopt);

  for (std::size_t i = 0; i + 1 < path.segments_.size(); ++i) {
    auto const& segment = path.segments_[i];
    auto const& next_segment = path.segments_[i + 1];

    utl_verify(segment.polyline_.size() > 1,
               "Polyline must have at least 2 elements but contains only {}",
               segment.polyline_.size());
    utl_verify(next_segment.polyline_.size() > 1,
               "Polyline must have at least 2 elements but contains only {}",
               next_segment.polyline_.size());

    const auto& last_coord = segment.polyline_.back();
    const auto& second_to_last_coord = segment.polyline_[segment.polyline_.size() - 2];

    auto target_coord = next_segment.polyline_.front();
    if (target_coord == last_coord) {
      target_coord = next_segment.polyline_[1];
    }

    double angle_in_deg = get_angle(second_to_last_coord, last_coord, target_coord);
    meta.relative_directions_[i] = get_relative_direction(angle_in_deg);

    if (segment.way_ == way_idx_t::invalid() ||
        next_segment.way_ == way_idx_t::invalid()) {
      continue;
    }

    if (segment.from_ == node_idx_t::invalid() ||
        segment.to_ == node_idx_t::invalid()) {
      continue;
    }

    meta.traversed_node_hubs_[i] = traversed_node_hub::from(ways_, segment, next_segment);
  }
}

} // namespace osr
