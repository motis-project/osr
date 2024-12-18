#pragma once

#include "osr/preprocessing/elevation/hgt.h"

#include <cstdint>
#include <bit>

#include "cista/mmap.h"

#include "osr/elevation_storage.h"

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

namespace osr::preprocessing::elevation {

// Void value is used in version 1.0 and 2.1 only
constexpr auto const kVoidValue = -32786;

template <std::size_t RasterSize>
struct hgt<RasterSize>::hgt<RasterSize>::impl {

  constexpr static auto kStepWidth = double{1. / (RasterSize - 1U)};
  constexpr static auto kCenterOffset = kStepWidth / 2.;

  impl(std::string const& filename,
       std::int8_t const lat,
       std::int16_t const lng)
      : file_{cista::mmap{filename.data(), cista::mmap::protection::READ}},
        sw_lat_(lat),
        sw_lng_(lng) {}

  elevation_t get(::osr::point const& p) {
    auto const lat = p.lat();
    auto const lng = p.lng();
    if (sw_lng_ - kCenterOffset <= lng && lng < sw_lng_ + 1 + kCenterOffset &&
        sw_lat_ - kCenterOffset <= lat && lat < sw_lat_ + 1 + kCenterOffset) {
      auto const column = std::clamp(
          static_cast<std::size_t>(
              std::floor((lng - kCenterOffset - sw_lng_) * (RasterSize - 1U))),
          std::size_t{0U}, RasterSize - 1U);
      auto const row = std::clamp(
          static_cast<std::size_t>(
              std::floor((lat - kCenterOffset - sw_lat_) * (RasterSize - 1U))),
          std::size_t{0U}, RasterSize - 1);
      auto const offset = kBytesPerPixel * (RasterSize * row + column);
      auto const v_native = get(offset);
      // Byte is stored in big-endian
      auto const v = (std::endian::native == std::endian::big)
                         ? v_native
                         : std::byteswap(v_native);
      return (v == kVoidValue) ? ::osr::NO_ELEVATION_DATA : v;
    }
    return ::osr::NO_ELEVATION_DATA;
  }

  elevation_t get(std::size_t const offset) {
    assert(offset < RasterSize * RasterSize);
    auto const byte_ptr = file_.data() + offset;
    return *reinterpret_cast<std::int16_t const*>(byte_ptr);
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
hgt<RasterSize>::hgt(std::string const& filename,
                     std::int8_t const lat,
                     std::int16_t const lng)
    : impl_{std::make_unique<impl>(filename, lat, lng)} {}

template <std::size_t RasterSize>
hgt<RasterSize>::~hgt() = default;

template <std::size_t RasterSize>
hgt<RasterSize>::hgt(hgt&& grid) noexcept = default;

template <std::size_t RasterSize>
::osr::elevation_t hgt<RasterSize>::get(::osr::point const& p) const {
  return impl_->get(p);
}

template <std::size_t RasterSize>
step_size hgt<RasterSize>::get_step_size() const {
  return impl_->get_step_size();
}

template <std::size_t RasterSize>
std::int8_t hgt<RasterSize>::lat() const {
  return impl_->sw_lat_;
}

template <std::size_t RasterSize>
std::int16_t hgt<RasterSize>::lng() const {
  return impl_->sw_lng_;
}

}  // namespace osr::preprocessing::elevation
