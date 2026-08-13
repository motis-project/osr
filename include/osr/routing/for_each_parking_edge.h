#pragma once

#include "geo/polyline.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

void for_each_parking_edge(ways::routing const&,
                           node_idx_t,
                           std::function<void(parking_edge_idx_t)> const&);

vec<point> parking_edge_connection_polyline(ways::routing::parking_edge const&);

vec<point> parking_edge_offset_polyline(
    ways const&, ways::routing::parking_edge::offset const&, bool is_left);

geo::polyline parking_edge_polyline(ways const&,
                                    ways::routing::parking_edge const&,
                                    node_idx_t from,
                                    node_idx_t to);

}  // namespace osr
