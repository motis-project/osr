#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

#include "osr/preprocessing/elevation/elevation.h"
#include "osr/preprocessing/elevation/location.h"

namespace osr::preprocessing::elevation {

struct dem_source {
  dem_source(std::filesystem::path const&);
  ~dem_source();
  dem_source(dem_source const&) = delete;
  dem_source& operator=(dem_source const&) = delete;
  dem_source(dem_source&&) = delete;
  dem_source& operator=(dem_source&&) = delete;

  elevation_t get(location const& loc) const;
  std::size_t size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
