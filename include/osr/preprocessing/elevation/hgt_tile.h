#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

template <std::size_t RasterSize>
struct hgt_tile {
  constexpr static auto const kBytesPerPixel = std::size_t{2U};

  explicit hgt_tile(std::string const& filename,
                    std::int8_t const lat,
                    std::int16_t const lng);
  ~hgt_tile();
  hgt_tile(hgt_tile&& grid) noexcept;
  hgt_tile(hgt_tile const&) = delete;
  hgt_tile& operator=(hgt_tile const&) = delete;
  hgt_tile& operator=(hgt_tile&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;

  step_size get_step_size() const;

  coord_box get_coord_box() const;
  sub_tile_idx_t get_sub_tile_idx(::osr::point const&) const;

  static constexpr std::size_t file_size() {
    return RasterSize * RasterSize * kBytesPerPixel;
  }

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

extern template struct hgt_tile<3601U>;
extern template struct hgt_tile<1201U>;

}  // namespace osr::preprocessing::elevation
