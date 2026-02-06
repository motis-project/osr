#pragma once

#include <vector>

#include "geo/latlng.h"

#include "osr/types.h"

namespace osr {

struct additional_edge {
  friend bool operator==(additional_edge, additional_edge) = default;

  node_idx_t node_{};  // to
  distance_t distance_{};

  way_idx_t underlying_way_{way_idx_t::invalid()};
  bool reverse_{false};
  std::vector<geo::latlng> polyline_{};
};

}  // namespace osr
