#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

enum class pixel_type : std::uint8_t { int16, float32 };

union pixel_value {
  std::int16_t int16_;
  float float32_;
};

struct dem_tile {
  explicit dem_tile(std::string const& filename);
  ~dem_tile();
  dem_tile(dem_tile&& grid) noexcept;
  dem_tile(dem_tile const&) = delete;
  dem_tile& operator=(dem_tile const&) = delete;
  dem_tile& operator=(dem_tile&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;
  coord_box get_coord_box() const;

  pixel_value get_raw(::osr::point const&) const;
  pixel_type get_pixel_type() const;
  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
