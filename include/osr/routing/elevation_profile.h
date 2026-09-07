#pragma once

#include <limits>
#include <vector>

#include "osr/point.h"
#include "osr/routing/path.h"
#include "osr/types.h"

namespace osr {

struct height_profile {
  height_profile(ways const&, path const&, unsigned steps);

  elevation_absolute_t median() const;

  std::vector<point> points_{};
  std::vector<elevation_difference_t> elevation_{};
  elevation_absolute_t baseline_;
  elevation_absolute_t min_{std::numeric_limits<elevation_absolute_t>::max()},
      max_{std::numeric_limits<elevation_absolute_t>::min()};
  double resolution_;
};

}  // namespace osr