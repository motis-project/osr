#pragma once

#include <vector>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/routing/path.h"
#include "osr/types.h"

namespace osr {

struct elevation_profile {
  elevation_profile(ways const&,
                    std::span<path::segment const>,
                    double resolution);

  elevation_absolute_t median() const;

  std::vector<point> points_{};
  std::vector<elevation_absolute_t> elevation_{};
  elevation_absolute_t baseline_;
  elevation_absolute_t min_{0}, max_{0};
  double resolution_;
};

}  // namespace osr