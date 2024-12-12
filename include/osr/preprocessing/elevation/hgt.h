#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

template <std::size_t RasterSize>
struct hgt {
  explicit hgt(std::string const& filename,
               std::int8_t const lat,
               std::int16_t const lng);
  ~hgt();
  hgt(hgt&& grid) noexcept;
  hgt(hgt const&) = delete;
  hgt& operator=(hgt const&) = delete;
  hgt& operator=(hgt&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;

  step_size get_step_size() const;

  std::int8_t lat() const;
  std::int16_t lng() const;

  static constexpr std::size_t file_size() {
    constexpr auto const kBytesPerPixel = std::size_t{2U};
    return RasterSize * RasterSize * kBytesPerPixel;
  }

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

extern template struct hgt<3601U>;
extern template struct hgt<1201U>;

}  // namespace osr::preprocessing::elevation
