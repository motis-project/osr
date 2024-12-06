#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "utl/progress_tracker.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

constexpr elevation_t NO_ELEVATION_DATA = -32767;

namespace preprocessing::elevation {
struct dem_source;
}

struct elevation_storage {
  elevation_storage(std::filesystem::path const&,
                    cista::mmap::protection const mode);
  static std::unique_ptr<elevation_storage> try_open(
      std::filesystem::path const&);
  void set_elevations(ways&,
                      preprocessing::elevation::dem_source const&,
                      std::shared_ptr<utl::progress_tracker>&);
  std::pair<elevation_t, elevation_t> get_elevations(
      way_idx_t const way,
      std::uint16_t const from,
      std::uint16_t const to) const;

  mm_vecvec<way_idx_t, int> elevation_up_;
  mm_vecvec<way_idx_t, int> elevation_down_;
};

std::pair<elevation_t, elevation_t> get_elevations(elevation_storage const*,
                                                   way_idx_t const way,
                                                   std::uint16_t const from,
                                                   std::uint16_t const to);

}  // namespace osr