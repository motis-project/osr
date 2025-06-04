#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "geo/box.h"
#include "geo/latlng.h"

#include "osr/preprocessing/elevation/resolution.h"
#include "osr/preprocessing/elevation/shared.h"

namespace osr::preprocessing::elevation {

enum class pixel_type : std::uint8_t { int16, float32 };

union pixel_value {
  std::int16_t int16_;
  float float32_;
};

struct dem_tile {
  explicit dem_tile(std::filesystem::path const&);
  ~dem_tile();
  dem_tile(dem_tile&& grid) noexcept;
  dem_tile(dem_tile const&) = delete;
  dem_tile& operator=(dem_tile const&) = delete;
  dem_tile& operator=(dem_tile&&) = delete;

  elevation_meters_t get(geo::latlng const&) const;
  tile_idx_t tile_idx(geo::latlng const&) const;
  geo::box get_box() const;

  pixel_value get_raw(geo::latlng const&) const;
  pixel_type get_pixel_type() const;
  resolution max_resolution() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
