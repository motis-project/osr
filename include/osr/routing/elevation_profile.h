#pragma once

#include <limits>
#include <vector>

#include "geo/latlng.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/routing/path.h"

namespace osr {

struct height_profile {
  using value_t = osr::preprocessing::elevation::elevation_meters_t;
  using loc_t = geo::latlng;

  height_profile(path&,
                 preprocessing::elevation::provider const&,
                 double resolution);

  value_t median();

  std::vector<loc_t> points_{};
  std::vector<value_t> elevation_{};
  value_t up_{0}, down_{0}, min_{std::numeric_limits<value_t>::max()},
      max_{std::numeric_limits<value_t>::min()};
  double resolution_;
};

}  // namespace osr