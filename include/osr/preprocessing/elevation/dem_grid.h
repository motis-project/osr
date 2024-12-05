#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "osr/point.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

enum class pixel_type : std::uint8_t { int16, float32 };

union pixel_value {
  std::int16_t int16_;
  float float32_;
};

struct dem_grid {
  explicit dem_grid(std::string const& filename);
  ~dem_grid();
  dem_grid(dem_grid&& grid) noexcept;
  dem_grid(dem_grid const&) = delete;
  dem_grid& operator=(dem_grid const&) = delete;
  dem_grid& operator=(dem_grid&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;

  pixel_value get_raw(::osr::point const&) const;
  pixel_type get_pixel_type() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
