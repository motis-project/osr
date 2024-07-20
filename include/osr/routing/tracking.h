#pragma once

#include "osr/routing/route.h"
#include "osr/ways.h"

namespace osr {

struct elevator_tracking {
  void write(path& p) const { p.uses_elevator_ = uses_elevator_; }
  void track(ways::routing const& r, way_idx_t, node_idx_t const n) {
    uses_elevator_ |= r.node_properties_[n].is_elevator();
  }

  bool uses_elevator_{false};
};

struct noop_tracking {
  void write(path&) const {}
  void track(ways::routing const&, way_idx_t, node_idx_t) {}
};

}  // namespace osr