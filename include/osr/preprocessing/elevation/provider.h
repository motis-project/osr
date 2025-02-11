#pragma once

#include <filesystem>

#include "geo/latlng.h"

#include "osr/preprocessing/elevation/resolution.h"
#include "osr/preprocessing/elevation/shared.h"

namespace osr::preprocessing::elevation {

struct provider {
  provider(std::filesystem::path const&);
  ~provider();
  provider(provider const&) = delete;
  provider& operator=(provider const&) = delete;
  provider(provider&&) = delete;
  provider& operator=(provider&&) = delete;

  elevation_meters_t get(geo::latlng const&) const;
  tile_idx_t tile_idx(geo::latlng const&) const;
  std::size_t driver_count() const;
  resolution max_resolution() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
