#pragma once

#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/types.h"

namespace osr {

struct path {
  struct segment {
    geo::polyline polyline_;
    level_t from_level_{kNoLevel};
    level_t to_level_{kNoLevel};
    node_idx_t from_{node_idx_t::invalid()};
    node_idx_t to_{node_idx_t::invalid()};
    way_idx_t way_{way_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    distance_t dist_{0};
    elevation_storage::elevation elevation_{};
    mode mode_{mode::kFoot};
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  elevation_storage::elevation elevation_{};
  std::vector<segment> segments_{};
  bool uses_elevator_{false};
  node_idx_t track_node_{node_idx_t::invalid()};
};

}  // namespace osr
