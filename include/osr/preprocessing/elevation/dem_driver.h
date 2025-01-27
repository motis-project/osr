#pragma once

#include <vector>

#include "cista/containers/rtree.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/dem_tile.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

struct dem_driver {
  dem_driver() = default;
  bool add_tile(fs::path const&);
  ::osr::elevation_t get(::osr::point const&) const;
  step_size get_step_size() const;
  elevation_tile_idx_t get_tile_idx(point const&) const;
  elevation_tile_idx_t get_sub_tile_idx(point const&) const;
  std::size_t n_tiles() const;

  cista::raw::rtree<std::size_t> rtree_{};
  std::vector<dem_tile> tiles_{};
};

}  // namespace osr::preprocessing::elevation
