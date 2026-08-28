#pragma once

#include <vector>

#include "geo/latlng.h"
#include "osr/point.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/routing/path.h"
#include "osr/types.h"

namespace osr {

struct height_profile {
  using value_t = osr::preprocessing::elevation::elevation_meters_t;
  using loc_t = geo::latlng;

  height_profile(path&,
                 preprocessing::elevation::provider const&,
                 unsigned n_samples);

  value_t median() { return 0; }

  value_t min() { return 0; }

  value_t max() { return 0; }

  value_t up_elevation() { return 0; }

  value_t down_elevation() { return 0; }

  std::vector<loc_t> points_{};
  std::vector<value_t> elevation_{};
  value_t up_{0}, down_{0};
};

}  // namespace osr