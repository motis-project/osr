#pragma once

#include "osr/types.h"

namespace osr {

struct additional_edge {
  friend bool operator==(additional_edge, additional_edge) = default;

  node_idx_t node_{};
  distance_t distance_{};
};

}  // namespace osr
