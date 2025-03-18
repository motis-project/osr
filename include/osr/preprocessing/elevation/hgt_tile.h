#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "geo/box.h"
#include "geo/latlng.h"

#include "osr/preprocessing/elevation/resolution.h"
#include "osr/preprocessing/elevation/shared.h"

namespace osr::preprocessing::elevation {

template <std::size_t RasterSize>
struct hgt_tile {
  constexpr static auto const kBytesPerPixel = std::size_t{2U};

  explicit hgt_tile(std::string const& path,
                    std::int8_t const lat,
                    std::int16_t const lng);
  ~hgt_tile();
  hgt_tile(hgt_tile&& grid) noexcept;
  hgt_tile(hgt_tile const&) = delete;
  hgt_tile& operator=(hgt_tile const&) = delete;
  hgt_tile& operator=(hgt_tile&&) = delete;

  elevation_meters_t get(geo::latlng const&) const;
  tile_idx_t tile_idx(geo::latlng const&) const;

  resolution max_resolution() const;

  geo::box get_box() const;

  static constexpr std::size_t file_size() {
    return RasterSize * RasterSize * kBytesPerPixel;
  }

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
