#pragma once

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

void for_each_parking_edge(ways::routing const&,
                           node_idx_t,
                           std::function<void(parking_edge_idx_t)> const&);

}  // namespace osr
