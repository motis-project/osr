#pragma once

#include <filesystem>

#include "osr/point.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

struct provider {
  provider(std::filesystem::path const&);
  ~provider();
  provider(provider const&) = delete;
  provider& operator=(provider const&) = delete;
  provider(provider&&) = delete;
  provider& operator=(provider&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;
  tile_idx_t tile_idx(::osr::point const&) const;
  std::size_t driver_count() const;
  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
