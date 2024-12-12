#pragma once

#include "osr/preprocessing/elevation/hgt.h"

#include <cstdint>
#include <bit>
#include <filesystem>
#include <sstream>

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

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

// Void value is used in version 1.0 and 2.1 only
constexpr auto const kVoidValue = -32786;

template <std::size_t RasterSize>
struct hgt<RasterSize>::hgt::impl {
  constexpr static auto kStepWidth = double{1. / (RasterSize - 1U)};
  constexpr static auto kPixelSize = 2U;

  impl(std::string const& filename,
       std::int8_t const lat,
       std::int16_t const lng)
      : filename_{filename},
        sw_lat_(lat),
        sw_lng_(lng),
        blx_{lng - kStepWidth / 2},
        bly_{lat - kStepWidth / 2},
        urx_{lng + 1 + kStepWidth / 2},
        ury_{lat + 1 + kStepWidth / 2} {}

  elevation_t get(::osr::point const& p) {
    auto const lat = p.lat();
    auto const lng = p.lng();
    if (blx_ <= lng && lng < urx_ && bly_ <= lat && lat < ury_) {
      auto const column =
          static_cast<std::size_t>(std::floor((lng - blx_) / kStepWidth));
      auto const row =
          static_cast<std::size_t>(std::floor((lat - bly_) / kStepWidth));
      auto const offset = kPixelSize * (RasterSize * row + column);
      auto const v_native = get(offset);
      // Byte stored in big-endian
      auto const v = (std::endian::native == std::endian::big)
                         ? v_native
                         : std::byteswap(v_native);
      return v == kVoidValue ? ::osr::NO_ELEVATION_DATA : v;
    }
    return ::osr::NO_ELEVATION_DATA;
  }

  elevation_t get(std::size_t const offset) {
    assert(0 <= offset && offset < RasterSize * RasterSize);
    {
      auto const l = std::lock_guard{m_};
      if (!file_) {
        std::cout << "Using HGT file " << filename_ << "\n";
        file_ = std::make_optional(
            cista::mmap{filename_.data(), cista::mmap::protection::READ});
      }
    }
    auto const byte_ptr = file_->data() + offset;
    return *reinterpret_cast<std::int16_t const*>(byte_ptr);
  }

  constexpr step_size get_step_size() const {
    return {.x_ = kStepWidth, .y_ = kStepWidth};
  }

  std::string filename_;
  std::optional<cista::mmap> file_{};
  std::mutex m_{};
  // south west
  std::int8_t sw_lat_;
  std::int16_t sw_lng_;
  // bottom left point
  double blx_;
  double bly_;
  // upper right point
  double urx_;
  double ury_;
};

inline point get_bottom_left(fs::path const& path) {
  auto lat_dir = char{};
  auto lng_dir = char{};
  auto lat = int{};
  auto lng = int{};
  auto s = std::stringstream{path.filename().string()};
  s >> lat_dir >> lat >> lng_dir >> lng;

  utl::verify(lat_dir == 'S' || lat_dir == 'N', "Invalid direction '{}'",
              lat_dir);
  utl::verify(lng_dir == 'W' || lng_dir == 'E', "Invalid direction '{}'",
              lng_dir);
  utl::verify(-180 <= lng && lng < 180, "Invalid longitude '{}'", lng);
  utl::verify(-90 <= lat && lat < 90, "Invalid latitude '{}'", lat);

  return point::from_latlng(lat_dir == 'N' ? lat : -lat,
                            lng_dir == 'E' ? lng : -lng);
}

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
