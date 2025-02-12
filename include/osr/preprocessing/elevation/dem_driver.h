#pragma once

#include <vector>

#include "cista/containers/rtree.h"

#include "geo/latlng.h"

#include "osr/preprocessing/elevation/dem_tile.h"
#include "osr/preprocessing/elevation/resolution.h"
#include "osr/preprocessing/elevation/shared.h"

namespace osr::preprocessing::elevation {

struct dem_driver {
  dem_driver() = default;
  bool add_tile(std::filesystem::path const&);
  elevation_meters_t get(geo::latlng const&) const;
  tile_idx_t tile_idx(geo::latlng const&) const;
  resolution max_resolution() const;
  std::size_t n_tiles() const;

  cista::raw::rtree<std::size_t> rtree_{};
  std::vector<dem_tile> tiles_{};
};

}  // namespace osr::preprocessing::elevation
