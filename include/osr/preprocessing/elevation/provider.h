#pragma once

#include <filesystem>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"

namespace osr::preprocessing::elevation {

struct provider {
  provider(std::filesystem::path const&);
  ~provider();
  provider(provider const&) = delete;
  provider& operator=(provider const&) = delete;
  provider(provider&&) = delete;
  provider& operator=(provider&&) = delete;

  elevation_meters_t get(point const&) const;
  tile_idx_t tile_idx(point const&) const;
  std::size_t driver_count() const;
  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
