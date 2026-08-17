#pragma once

#include <optional>

#include "geo/polyline.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

void for_each_connection(ways::routing const&,
                         node_idx_t,
                         std::function<void(connection_idx_t)> const&);

ways::routing::additional_connection const& get_additional_connection(
    ways::routing const&, way_idx_t);
// std::optional<ways::routing::additional_connection const>
// get_additional_connection(ways::routing const&, way_idx_t);

way_idx_t add_additional_connection(
    ways::routing&,
    ways::routing::additional_connection::offset&& from,
    ways::routing::additional_connection::offset&& to,
    vec<point>&& connection,
    bool const is_parking);

geo::polyline get_additional_connection_polyline(
    ways const&,
    ways::routing::additional_connection const&,
    node_idx_t from,
    node_idx_t to);

vec<point> get_additional_connection_points(
    ways::routing::additional_connection const&);

vec<point> get_additional_connection_offset_points(
    ways const&,
    ways::routing::additional_connection::offset const&,
    bool is_left);

}  // namespace osr
