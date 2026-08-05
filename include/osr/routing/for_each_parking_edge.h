#pragma once

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

void for_each_parking_edge(ways::routing const&,
                           node_idx_t,
                           std::function<void(parking_edge_idx_t)> const&);

vec<point> parking_edge_offset_polyline(ways const&,
                                        ways::routing::parking_edge const&,
                                        bool is_from,
                                        bool is_left);

vec<point> parking_edge_offset_polyline(ways const&,
                                        ways::routing::parking_edge const&,
                                        bool is_from,
                                        node_idx_t);

}  // namespace osr
