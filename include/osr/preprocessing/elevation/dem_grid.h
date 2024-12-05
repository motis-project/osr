#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "osr/elevation.h"
#include "osr/preprocessing/elevation/location.h"

namespace osr::preprocessing::elevation {

enum class pixel_type : uint8_t { int16, float32 };

union pixel_value {
  int16_t int16_;
  float float32_;
};

struct dem_grid {
  explicit dem_grid(std::string const& filename);
  ~dem_grid();
  dem_grid(dem_grid&& grid) noexcept;
  dem_grid(dem_grid const&) = delete;
  dem_grid& operator=(dem_grid const&) = delete;
  dem_grid& operator=(dem_grid&& grid) = delete;

  elevation_t get(location const& loc) const;

  pixel_value get_raw(location const& loc) const;
  pixel_type get_pixel_type() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
