#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"
#include "cista/strong.h"

#include "osr/preprocessing/elevation/shared.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace preprocessing::elevation {
struct provider;
}

struct elevation_storage {
  elevation_storage(std::filesystem::path const&,
                    cista::mmap::protection const mode);
  static std::unique_ptr<elevation_storage> try_open(
      std::filesystem::path const&);
  void set_elevations(
      ways const&,
      preprocessing::elevation::provider const&,
      preprocessing::elevation::elevation_meters_t min_elevation,
      double max_incline);
  std::vector<preprocessing::elevation::elevation_meters_t> get_elevations(
      way_idx_t const way, std::uint16_t const segment) const;

  // (way_idx, internal-idx) -> single elevation sample
  // mm_vecvec<way_idx_t, encoding> elevations_;
  using data_vec_t = mm_vec32<preprocessing::elevation::elevation_meters_t>;
  data_vec_t elevations_;

  struct offset_t {
    cista::base_t<way_idx_t> const idx_ = 0;
    std::uint8_t const n_ = 0;

    bool valid() const { return n_ != 0; }
  };

  // (way_idx, segment_idx) -> internal-idx, number of samples
  mm_vecvec<way_idx_t, offset_t> offsets_;
};

std::vector<preprocessing::elevation::elevation_meters_t> get_elevations(
    elevation_storage const*, way_idx_t const way, std::uint16_t const segment);

}  // namespace osr