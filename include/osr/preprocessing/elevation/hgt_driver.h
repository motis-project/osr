#pragma once

#include <optional>
#include <variant>
#include <vector>

#include "cista/containers/rtree.h"

#include "geo/latlng.h"

#include "osr/preprocessing/elevation/hgt_tile.h"
#include "osr/preprocessing/elevation/resolution.h"
#include "osr/preprocessing/elevation/shared.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

static_assert(IsTile<hgt_tile<3601>>);
static_assert(IsTile<hgt_tile<1201>>);

struct hgt_driver {
  using hgt_tile_t = std::variant<hgt_tile<3601>, hgt_tile<1201>>;

  hgt_driver() = default;
  bool add_tile(fs::path const&);
  elevation_meters_t get(geo::latlng const&) const;
  tile_idx_t tile_idx(geo::latlng const&) const;
  resolution max_resolution() const;
  std::size_t n_tiles() const;
  static std::optional<hgt_tile_t> open(fs::path const&);

  cista::raw::rtree<std::size_t> rtree_;
  std::vector<hgt_tile_t> tiles_;
};

}  // namespace osr::preprocessing::elevation
