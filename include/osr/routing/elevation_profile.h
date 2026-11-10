#pragma once

#include <vector>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/routing/path.h"

namespace osr {

struct elevation_profile {
  elevation_profile(ways const&, std::span<path::segment const>);

  preprocessing::elevation::elevation_meters_t median() const;

  std::vector<point> points_{};
  std::vector<preprocessing::elevation::elevation_meters_t> elevation_{};
  preprocessing::elevation::elevation_meters_t baseline_ =
      preprocessing::elevation::elevation_meters_t::invalid();
  preprocessing::elevation::elevation_meters_t min_{0}, max_{0};
};

}  // namespace osr