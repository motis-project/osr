#pragma once

#include "osr/routing/dijkstra_bidir.h"
#include "osr/routing/profile.h"

namespace osr {

template <Profile P, bool EarlyTermination = false>
struct cch : dijkstra_bidir<P, EarlyTermination> {};

}  // namespace osr
