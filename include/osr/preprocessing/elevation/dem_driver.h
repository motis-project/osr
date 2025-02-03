#pragma once

#include <vector>

#include "cista/containers/rtree.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/dem_tile.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

struct dem_driver {
  dem_driver() = default;
  bool add_tile(fs::path const&);
  elevation_meters_t get(point const&) const;
  tile_idx_t tile_idx(point const&) const;
  step_size get_step_size() const;
  std::size_t n_tiles() const;

  cista::raw::rtree<std::size_t> rtree_{};
  std::vector<dem_tile> tiles_{};
};

}  // namespace osr::preprocessing::elevation
