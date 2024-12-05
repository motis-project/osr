#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "utl/progress_tracker.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

using elevation_t = int16_t;

constexpr elevation_t NO_ELEVATION_DATA = -32767;

namespace preprocessing::elevation {
struct dem_source;
}

struct elevation {
  elevation(std::filesystem::path const& p, cista::mmap::protection const mode);
  void set_elevations(ways& w,
                      preprocessing::elevation::dem_source const& dem,
                      std::shared_ptr<utl::progress_tracker>& pt);
  mm_vecvec<way_idx_t, int> elevation_up_m_;
  mm_vecvec<way_idx_t, int> elevation_down_m_;
};

std::pair<elevation_t, elevation_t> get_elevations(ways::routing const& r,
                                                   way_idx_t const way,
                                                   std::uint16_t const from,
                                                   std::uint16_t const to);

}  // namespace osr