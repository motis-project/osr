#pragma once

#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/types.h"

namespace osr {

// Accessibility of a path (precomputed footpaths): path kinds used, steepest
// tagged incline, narrowest tagged width. Ways without an incline / width tag
// set the unknown bits (steps and elevators have no incline). Values are
// stored in the resolution of footpath restriction records (incline 0.2 %,
// width 5 cm), so labels that only differ below it do not multiply.
struct path_accessibility {
  static constexpr auto const kSteps = std::uint8_t{1U << 0U};
  static constexpr auto const kElevator = std::uint8_t{1U << 2U};
  static constexpr auto const kRamp = std::uint8_t{1U << 3U};
  static constexpr auto const kNoWidth = std::uint16_t{0xFFFFU};

  friend constexpr bool operator==(path_accessibility const&,
                                   path_accessibility const&) = default;

  constexpr bool has_width() const { return min_width_cm_ != kNoWidth; }

  // a is at least as good as b for every user
  friend constexpr bool is_less_restricted(path_accessibility const& a,
                                           path_accessibility const& b) {
    return (a.flags_ & ~b.flags_) == 0U && a.max_incline_ <= b.max_incline_ &&
           a.min_width_cm_ >= b.min_width_cm_ &&
           (!a.has_unknown_incline_ || b.has_unknown_incline_) &&
           (!a.has_unknown_width_ || b.has_unknown_width_);
  }

  std::uint8_t flags_{0U};
  bool has_unknown_incline_{false};
  bool has_unknown_width_{false};
  std::uint16_t max_incline_{0U};  // 0.1 %, even (0.2 % steps, rounded up)
  std::uint16_t min_width_cm_{kNoWidth};  // 5 cm steps, rounded down
};

struct path {
  struct segment {
    geo::polyline polyline_;
    level_t from_level_{kNoLevel};
    level_t to_level_{kNoLevel};
    node_idx_t from_{node_idx_t::invalid()};
    node_idx_t to_{node_idx_t::invalid()};
    way_idx_t way_{way_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    duration_t duration_{0};
    distance_t dist_{0};
    elevation_storage::elevation elevation_{};
    mode mode_{mode::kFoot};
  };

  cost_t cost_{kInfeasible};
  duration_t duration_{0};
  double dist_{0.0};
  elevation_storage::elevation elevation_{};
  std::vector<segment> segments_{};
  bool uses_elevator_{false};
  node_idx_t track_node_{node_idx_t::invalid()};
};

}  // namespace osr
