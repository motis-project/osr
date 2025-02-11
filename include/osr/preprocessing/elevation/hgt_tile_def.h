#pragma once

#include "osr/preprocessing/elevation/hgt_tile.h"

#include <cassert>
#include <algorithm>
#include <bit>
#include <limits>

#include "cista/mmap.h"

// SRTM HGT File Format
//
// https://lpdaac.usgs.gov/documents/179/SRTM_User_Guide_V3.pdf
//
// Usage (see User Guide 2.1.4 SRTM Topography Data Format):
//
// The names of individual data tiles refer to the latitude and longitude of the
// south west (lower left) corner of the tile. For example, N37W105 has its
// lower left corner at 37°N latitude and 105° West (W) longitude and covers
// (slightly more than) the area 37-38°N and 104 -105°W. To be more exact, the
// file name coordinates refer to the geometric center of the lower- left pixel,
// and all edge pixels of the tile are centered on whole-degree lines of
// latitude and / or longitude. The unit of elevation is meters as referenced to
// the WGS84/EGM96 geoid (NGA, 1997;Lemoine, 1998)
//
// Height files have the extension. HGT, and the DEM is provided as two-byte
// (16-bit) binary signed integer raster data. Two-byte signed integers can
// range from - 32,767 to 32,767m and can encompass the range of the Earth’s
// elevations. Header or trailer bytes are not embedded in the file. The data
// are stored in row major order, meaning all the data for the northernmost row,
// row 1, are followed by all the data for row 2, and so on .
//

namespace osr::preprocessing::elevation {

// Void value is used in version 1.0 and 2.1 only
constexpr auto const kVoidValue = std::int16_t{-32768};

template <std::size_t RasterSize>
struct hgt_tile<RasterSize>::hgt_tile<RasterSize>::impl {

  constexpr static auto kStepWidth = double{1. / (RasterSize - 1U)};
  constexpr static auto kCenterOffset = kStepWidth / 2.;

  impl(std::string const& filename,
       std::int8_t const lat,
       std::int16_t const lng)
      : file_{cista::mmap{filename.data(), cista::mmap::protection::READ}},
        sw_lat_(lat),
        sw_lng_(lng) {}

  template <std::size_t UpperBound>
  std::size_t get_offset(geo::latlng const& pos) const {
    auto const box = get_box();
    if (box.contains(pos)) {
      // Column: Left to right
      auto const column = std::clamp(
          static_cast<std::size_t>(
              std::floor(((pos.lng_ - box.min_.lng_) * (RasterSize - 1U)) /
                         RasterSize * UpperBound)),
          std::size_t{0U}, UpperBound - 1U);
      // Row: Top to bottom
      auto const row = std::clamp(
          static_cast<std::size_t>(
              std::floor(((box.max_.lat_ - pos.lat_) * (RasterSize - 1U)) /
                         RasterSize * UpperBound)),
          std::size_t{0U}, (UpperBound - 1U));
      // Data in row major order
      return UpperBound * row + column;
    }
    return std::numeric_limits<std::size_t>::max();
  }

  elevation_meters_t get(geo::latlng const& pos) const {
    auto const offset = get_offset<RasterSize>(pos);
    if (offset == std::numeric_limits<std::size_t>::max()) {
      return elevation_meters_t::invalid();
    }
    return get(kBytesPerPixel * offset);
  }

  elevation_meters_t get(std::size_t const offset) const {
    assert(offset < kBytesPerPixel * RasterSize * RasterSize);
    auto const byte_ptr = file_.data() + offset;
    auto const raw_value = *reinterpret_cast<std::int16_t const*>(byte_ptr);
    // Byte is stored in big-endian
    auto const meters = std::endian::native == std::endian::big
                            ? raw_value
                            : std::byteswap(raw_value);
    return (meters == kVoidValue) ? elevation_meters_t::invalid()
                                  : elevation_meters_t{meters};
  }

  tile_idx_t tile_idx(geo::latlng const& pos) const {
    constexpr auto const kSegments = 1 << (tile_idx_t::kSubTileIdxSize / 2);
    auto const offset = get_offset<kSegments>(pos);
    return (offset == std::numeric_limits<std::size_t>::max())
               ? tile_idx_t::invalid()
               : tile_idx_t::from_sub_tile(
                     static_cast<tile_idx_t::data_t>(offset));
  }

  geo::box get_box() const {
    return {
        {
            sw_lat_ - kCenterOffset,
            sw_lng_ - kCenterOffset,
        },
        {
            sw_lat_ + 1. + kCenterOffset,
            sw_lng_ + 1. + kCenterOffset,
        },
    };
  }

  constexpr resolution max_resolution() const {
    return {.x_ = kStepWidth, .y_ = kStepWidth};
  }

  cista::mmap file_{};
  // south west coordinate
  std::int8_t sw_lat_;
  std::int16_t sw_lng_;
};

template <std::size_t RasterSize>
hgt_tile<RasterSize>::hgt_tile(std::string const& filename,
                               std::int8_t const lat,
                               std::int16_t const lng)
    : impl_{std::make_unique<impl>(filename, lat, lng)} {}

template <std::size_t RasterSize>
hgt_tile<RasterSize>::~hgt_tile() = default;

template <std::size_t RasterSize>
hgt_tile<RasterSize>::hgt_tile(hgt_tile&& grid) noexcept = default;

template <std::size_t RasterSize>
elevation_meters_t hgt_tile<RasterSize>::get(geo::latlng const& pos) const {
  return impl_->get(pos);
}

template <std::size_t RasterSize>
tile_idx_t hgt_tile<RasterSize>::tile_idx(geo::latlng const& pos) const {
  return impl_->tile_idx(pos);
}

template <std::size_t RasterSize>
resolution hgt_tile<RasterSize>::max_resolution() const {
  return impl_->max_resolution();
}

template <size_t RasterSize>
geo::box hgt_tile<RasterSize>::get_box() const {
  return impl_->get_box();
}

}  // namespace osr::preprocessing::elevation
