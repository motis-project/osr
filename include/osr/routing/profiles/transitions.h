#pragma once

#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace osr::transitions {

// callable object to check if a node is a parking
  struct is_parking {
    static constexpr bool operator()(osr::ways::routing const& w,
                    node_idx_t n_,
                    bitvec<node_idx_t> const*
    ) {
      return w.node_properties_[n_].is_parking();
    }
  };

}  // namespace osr