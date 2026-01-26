#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/types.h"

namespace osr {

struct matched_route {
  path path_{};
  std::vector<std::size_t> segment_offsets_{};
  unsigned n_routed_{};
  unsigned n_beelined_{};
};

matched_route map_match(
    ways const&,
    lookup const&,
    search_profile,
    profile_parameters const&,
    std::vector<location> const&,
    cost_t max_segment_cost,
    bitvec<node_idx_t> const* blocked = nullptr,
    elevation_storage const* = nullptr,
    std::function<std::optional<std::filesystem::path>(matched_route const&)>
        debug_path_fn = nullptr);

}  // namespace osr
