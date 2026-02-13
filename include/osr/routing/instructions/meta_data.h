#pragma once

#include <vector>

#include "osr/routing/instructions/directions.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct way_osm_nodes_idx_range {
  std::uint16_t from_;
  std::uint16_t to_; // inclusive
};

struct way_segment {

  bool is_way_aligned() const;

  static way_segment from(const ways& w, osm_node_idx_t from, osm_node_idx_t to, way_idx_t way);

  way_idx_t way_idx_;
  way_osm_nodes_idx_range osm_node_range_;
};

struct relative_way_segment {
  double angle_;
  bool is_accessible_;
  way_segment segment_;
};

struct traversed_node_hub {

  static traversed_node_hub from(ways const& w, path::segment const& arrive_on, path::segment const& exit_on);

  way_segment arrive_hub_on_;
  node_idx_t hub_node_;

  double exit_angle_;
  way_segment exit_from_hub_;

  std::vector<relative_way_segment> alts_left_{};
  std::vector<relative_way_segment> alts_right_{};
};

struct instruction_meta_data {
  std::vector<std::optional<traversed_node_hub>> traversed_node_hubs_;
  std::vector<relative_direction> relative_directions_;
};

} // namespace osr
