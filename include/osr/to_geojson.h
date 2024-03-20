#pragma once

#include <string>

namespace osr {

struct ways;

std::string to_geojson(ways const&);

}  // namespace osr