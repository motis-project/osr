#pragma once

#include "geo/polyline.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

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
    ways const&,
    ways::routing::additional_connection const&,
    node_idx_t from,
    node_idx_t to);

vec<point> get_additional_connection_points(
    ways::routing::additional_connection const&);

vec<point> get_additional_connection_offset_points(
    ways const&,
    ways::routing::additional_connection::offset const&,
    point const&,
    bool is_left);

ways::routing::additional_connection::offset to_offset(ways const&,
                                                       way_candidate const&,
                                                       point const&);

void for_each_addional_connection(
    ways::routing const&,
    node_idx_t,
    direction,
    std::function<void(ways::routing::additional_connection const&,
                       way_idx_t,
                       node_idx_t,
                       direction,
                       additional_connection_offset from,
                       additional_connection_offset to)> const&);

}  // namespace osr
