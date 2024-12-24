#pragma once

#include <optional>
#include <variant>
#include <vector>

#include "osr/point.h"
#include "osr/preprocessing/elevation/hgt.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

static_assert(IsRasterDriver<hgt<3601>>);
static_assert(IsRasterDriver<hgt<1201>>);

struct hgt_raster {
  using hgt_tile = std::variant<hgt<3601>, hgt<1201>>;

  explicit hgt_raster(std::vector<hgt_tile>&&);
  ::osr::elevation_t get(::osr::point const&) const;
  step_size get_step_size() const;
  static std::optional<hgt_tile> open(fs::path const&);

private:
  std::size_t get_tile_offset(int lat, int lng) const;
  std::vector<std::optional<hgt_tile>> tiles_;
  std::int8_t sw_lat_{0};
  std::int16_t sw_lng_{0};
  std::int8_t ne_lat_{0};
  std::int16_t ne_lng_{0};
  std::int16_t width_;
};

}  // namespace osr::preprocessing::elevation
