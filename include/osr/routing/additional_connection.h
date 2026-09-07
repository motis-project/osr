#pragma once

#include "geo/polyline.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct lookup;
struct way_candidate;

struct additional_connection_offset {
  ways::routing::additional_connection::offset const& offset_;
  node_idx_t node_;
  std::uint16_t dist_;
  direction dir_;
};

void for_each_connection(ways::routing const&,
                         node_idx_t,
                         std::function<void(connection_idx_t)> const&);

bool is_additional_connection(ways::routing const&, way_idx_t);

ways::routing::additional_connection const& get_additional_connection(
    ways::routing const&, way_idx_t);

way_idx_t add_additional_connection(
    ways::routing&,
    ways::routing::additional_connection::offset&& from,
    ways::routing::additional_connection::offset&& to,
    vec<point>&& connection,
    bool const is_parking);

geo::polyline get_additional_connection_polyline(
    lookup const&,
    ways::routing::additional_connection const&,
    node_idx_t from,
    node_idx_t to);

geo::polyline get_additional_connection_points(
    ways::routing::additional_connection const&);

geo::polyline get_additional_connection_offset_points(
    lookup const&,
    ways::routing::additional_connection::offset const&,
    point const&,
    bool is_left);

ways::routing::additional_connection::offset to_offset(way_candidate const&);

void for_each_addional_connection(
    ways::routing const&,
    node_idx_t,
    direction,
    std::function<void(ways::routing::additional_connection const&,
                       way_idx_t,
                       node_idx_t,
                       additional_connection_offset const& from,
                       additional_connection_offset const& to)> const&);

}  // namespace osr
