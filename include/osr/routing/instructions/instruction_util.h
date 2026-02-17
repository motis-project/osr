#pragma once

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

bool ways_have_same_name(way_idx_t w1, way_idx_t w2, ways const& w);

} // namespace osr
