#pragma once

#include <type_traits>

#include "geo/polyline.h"

#include "osr/lookup.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename t>
struct is_parking_type : std::false_type {};

template <bool IsWheelchair>
struct is_parking_type<car_parking<IsWheelchair, true>> : std::true_type {};

template <typename T>
constexpr bool is_parking() {
  return is_parking_type<T>::value;
}

static_assert(is_parking<car_parking<false, true>>());
static_assert(is_parking<car_parking<true, true>>());
static_assert(!is_parking<car>());
static_assert(!is_parking<car_parking<true, false>>());

void connect_parking_ways(ways&,
                          lookup const&,
                          vec_map<way_idx_t, way_extra_properties> const&,
                          unsigned n_components);

bool is_parking_way(ways::routing const&, way_idx_t);

geo::polyline parking_way_polyline(
    ways const&, way_idx_t, direction, node_idx_t, node_idx_t);

}  // namespace osr
