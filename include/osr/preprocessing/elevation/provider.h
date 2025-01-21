#pragma once

#include <filesystem>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

struct provider {
  struct point_idx {
    bool operator<(point_idx const& other) const {
      return ((driver_idx_ < other.driver_idx_) ||
              ((driver_idx_ == other.driver_idx_) &&
               (tile_idx_ < other.tile_idx_)));
    }
    bool operator==(point_idx const& other) const {
      return (driver_idx_ == other.driver_idx_) &&
             (tile_idx_ == other.tile_idx_);
    }
    elevation_driver_idx_t driver_idx_;
    elevation_tile_idx_t tile_idx_;
  };

  provider(std::filesystem::path const&);
  ~provider();
  provider(provider const&) = delete;
  provider& operator=(provider const&) = delete;
  provider(provider&&) = delete;
  provider& operator=(provider&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;
  std::size_t driver_count() const;
  step_size get_step_size() const;
  point_idx get_point_idx(::osr::point const&) const;
  unsigned int get_bucket_count() const;
  elevation_bucket_idx_t get_bucket_idx(::osr::point const&) const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
