#pragma once

#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

void connect_parking_ways(ways const&,
                          lookup const&,
                          vec_map<way_idx_t, way_extra_properties> const&,
                          unsigned n_components);

}  // namespace osr
