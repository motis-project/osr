#pragma once

#include <filesystem>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

struct dem_source {
  dem_source(std::filesystem::path const&);
  ~dem_source();
  dem_source(dem_source const&) = delete;
  dem_source& operator=(dem_source const&) = delete;
  dem_source(dem_source&&) = delete;
  dem_source& operator=(dem_source&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;
  std::size_t size() const;
  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
