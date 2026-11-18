#pragma once

#include <algorithm>

#include "osr/routing/path.h"
#include "osr/ways.h"

#ifdef _MSC_VER
#define OSR_NO_UNIQUE_ADDRESS [[no_unique_address]] [[msvc::no_unique_address]]
#else
#define OSR_NO_UNIQUE_ADDRESS [[no_unique_address]]
#endif

namespace osr {

struct elevator_tracking {
  void write(path& p) const { p.uses_elevator_ = uses_elevator_; }
  void track(elevator_tracking const& l,
             ways::routing const& r,
             way_idx_t,
             node_idx_t const n,
             bool) {
    uses_elevator_ = l.uses_elevator_ || r.node_properties_[n].is_elevator();
  }

  bool uses_elevator_{false};
};

// Record of a path extended by way (and its target node n).
inline path_accessibility extend(path_accessibility a,
                                 ways::routing const& r,
                                 way_idx_t const way,
                                 node_idx_t const n) {
  if (n != node_idx_t::invalid() && r.node_properties_[n].is_elevator()) {
    a.flags_ |= path_accessibility::kElevator;
  }
  if (way == way_idx_t::invalid()) {
    return a;
  }
  auto const p = r.way_properties_[way];
  if (p.is_steps()) {
    a.flags_ |= path_accessibility::kSteps;
  }
  if (p.is_elevator()) {
    a.flags_ |= path_accessibility::kElevator;
  }
  if (p.is_ramp()) {
    a.flags_ |= path_accessibility::kRamp;
  }
  auto const* x = r.get_accessibility(way);
  if (x != nullptr && x->has_incline()) {
    auto const incline =
        static_cast<std::uint16_t>((x->abs_incline() + 1U) / 2U * 2U);
    a.max_incline_ = std::max(a.max_incline_, incline);
  } else if (!p.is_steps() && !p.is_elevator()) {
    a.has_unknown_incline_ = true;
  }
  if (x != nullptr && x->has_width()) {
    auto const width = static_cast<std::uint16_t>(x->width_cm_ / 5U * 5U);
    a.min_width_cm_ = std::min(a.min_width_cm_, width);
  } else {
    a.has_unknown_width_ = true;
  }
  return a;
}

struct track_node_tracking {
  void write(path& p) const { p.track_node_ = track_node_; }
  void track(track_node_tracking const& l,
             ways::routing const&,
             way_idx_t,
             node_idx_t const n,
             bool const track) {
    track_node_ = track ? n : l.track_node_;
  }

  node_idx_t track_node_{node_idx_t::invalid()};
};

struct noop_tracking {
  void write(path&) const {}
  void track(
      noop_tracking const&, ways::routing const&, way_idx_t, node_idx_t, bool) {
  }
};

}  // namespace osr
