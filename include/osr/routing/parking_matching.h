#pragma once

#include "osr/lookup.h"
#include "osr/ways.h"

namespace osr {

void connect_parking_ways(ways const&, lookup const&, unsigned n_components);

}  // namespace osr
