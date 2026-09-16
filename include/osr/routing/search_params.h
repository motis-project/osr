#pragma once

#include <optional>

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <typename ProfileParameters>
struct search_params {
  ways const& w() const { return *w_; }
  ways::routing const& r() const { return *w_->r_; }
  sharing_data const* sharing() const { return sharing_; }

  ProfileParameters profile_{};
  ways const* w_{nullptr};

  cost_t max_{0U};  // caller clamps to max(kMinCostSettled, max)
  direction dir_{direction::kForward};
  std::optional<routing_time_t> start_time_{};

  bitvec<node_idx_t> const* blocked_{nullptr};
  sharing_data const* sharing_{nullptr};  // only read during search,
                                          // reconstruct uses one_to_many_state
  elevation_storage const* elevations_{nullptr};

  location start_loc_{};
  location end_loc_{};
};

}  // namespace osr
