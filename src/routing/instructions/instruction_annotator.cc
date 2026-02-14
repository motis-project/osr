#include "osr/routing/instructions/instruction_annotator.h"

#include "osr/routing/path.h"
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

void set_relative_direction(path::segment const& from,
                            path::segment const& to,
                            std::vector<relative_direction>& relative_directions,
                            size_t idx) {
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
  relative_directions[idx] = get_relative_direction(angle_in_deg);
}

void set_traversed_node_hub(path::segment const& segment,
                            path::segment const& next_segment,
                            ways const& w,
                            std::vector<std::optional<traversed_node_hub>>& hubs,
                            const size_t idx) {
  if (segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid()) {
    return;
  }

  if (segment.from_ == node_idx_t::invalid() || segment.to_ == node_idx_t::invalid() ||
      next_segment.from_ == node_idx_t::invalid() || next_segment.to_ == node_idx_t::invalid()) {
    return;
  }

  hubs[idx] = traversed_node_hub::from(w, segment, next_segment);
}


void instruction_annotator::preprocess(path const& path,
                                       instruction_meta_data& meta) {
  meta.relative_directions_.resize(path.segments_.size(), relative_direction::kInvalid);
  meta.traversed_node_hubs_.resize(path.segments_.size(), std::nullopt);

  for (std::size_t i = 0; i + 1 < path.segments_.size(); ++i) {
    auto const& segment = path.segments_[i];
    auto const& next_segment = path.segments_[i + 1];

    set_relative_direction(segment, next_segment, meta.relative_directions_, i);
    set_traversed_node_hub(segment, next_segment, ways_, meta.traversed_node_hubs_, i);
  }
}

} // namespace osr
