#pragma once

#include "osr/preprocessing/elevation/hgt_tile.h"

#include <cstdint>
#include <algorithm>
#include <bit>
#include <limits>

#include "cista/mmap.h"

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/shared.h"

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
constexpr auto const kVoidValue = -32786;

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
  std::size_t get_offset(::osr::point const& p) const {
    auto const lat = p.lat();
    auto const lng = p.lng();
    auto const box = get_coord_box();
    if (box.min_lng_ <= lng && lng < box.max_lng_ &&  //
        box.min_lat_ <= lat && lat < box.max_lat_) {
      auto const column =
          std::clamp(static_cast<std::size_t>(
                         std::floor((lng - box.min_lng_) * (RasterSize - 1U) *
                                    (UpperBound / RasterSize))),
                     std::size_t{0U}, UpperBound - 1U);
      auto const row = std::clamp(static_cast<std::size_t>(std::floor(
                                      (box.max_lat_ - lat) * (RasterSize - 1U) *
                                      (UpperBound / RasterSize))),
                                  std::size_t{0U}, (UpperBound - 1U));
      return UpperBound * row + column;
    }
    return std::numeric_limits<std::size_t>::max();
  }

  elevation_t get(::osr::point const& p) const {
    auto const offset = get_offset<RasterSize>(p);
    if (offset == std::numeric_limits<std::size_t>::max()) {
      return ::osr::NO_ELEVATION_DATA;
    }
    auto const value = get(kBytesPerPixel * offset);
    return (value == kVoidValue) ? ::osr::NO_ELEVATION_DATA : value;
  }

  elevation_t get(std::size_t const offset) const {
    assert(offset < kBytesPerPixel * RasterSize * RasterSize);
    auto const byte_ptr = file_.data() + offset;
    auto const raw_value = *reinterpret_cast<std::int16_t const*>(byte_ptr);
    // Byte is stored in big-endian
    return std::endian::native == std::endian::big ? raw_value
                                                   : std::byteswap(raw_value);
  }

  tile_idx_t tile_idx(::osr::point const& p) const {
    constexpr auto const kUpper = 1 << (tile_idx_t::kSubTileIdxSize / 2);
    auto const offset = get_offset<kUpper>(p);
    return (offset == std::numeric_limits<std::size_t>::max())
               ? tile_idx_t::invalid()
               : tile_idx_t::from_sub_tile(offset);
  }

  coord_box get_coord_box() const {
    return {
        .min_lat_ = static_cast<float>(sw_lat_ - kCenterOffset),
        .min_lng_ = static_cast<float>(sw_lng_ - kCenterOffset),
        .max_lat_ = static_cast<float>(sw_lat_ + 1.F + kCenterOffset),
        .max_lng_ = static_cast<float>(sw_lng_ + 1.F + kCenterOffset),
    };
  }

  sub_tile_idx_t get_sub_tile_idx(::osr::point const& p) const {
    constexpr auto const kUpper =
        1 << (std::numeric_limits<sub_tile_idx_t>::digits / 2);
    auto const offset = get_offset<kUpper>(p);
    return (offset == std::numeric_limits<std::size_t>::max())
               ? 0U
               : static_cast<sub_tile_idx_t>(offset);
  }

  constexpr step_size get_step_size() const {
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
::osr::elevation_t hgt_tile<RasterSize>::get(::osr::point const& p) const {
  return impl_->get(p);
}

template <std::size_t RasterSize>
tile_idx_t hgt_tile<RasterSize>::tile_idx(::osr::point const& p) const {
  return impl_->tile_idx(p);
}

template <std::size_t RasterSize>
step_size hgt_tile<RasterSize>::get_step_size() const {
  return impl_->get_step_size();
}

template <size_t RasterSize>
coord_box hgt_tile<RasterSize>::get_coord_box() const {
  return impl_->get_coord_box();
}

template <size_t RasterSize>
sub_tile_idx_t hgt_tile<RasterSize>::get_sub_tile_idx(
    ::osr::point const& point) const {
  return impl_->get_sub_tile_idx(point);
}

}  // namespace osr::preprocessing::elevation
