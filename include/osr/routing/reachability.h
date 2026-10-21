#pragma once

#include <cassert>

#include "utl/helpers/algorithm.h"

#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

// The components contain every way `P` can use, but not the additional edges
// of sharing data.
template <Profile P>
class_components const* reachability_components(ways const& w,
                                                sharing_data const* sharing) {
  return P::kReachabilityClass.has_value() && sharing == nullptr
             ? &w.r_->get_class_components(*P::kReachabilityClass)
             : nullptr;
}

inline bool is_connected(class_components const& c,
                         way_idx_t const a,
                         match_view_t const& m) {
  return utl::any_of(m.way_,
                     [&](way_idx_t const b) { return c.is_connected(a, b); });
}

inline bool is_connected(class_components const& c,
                         match_view_t const& a,
                         match_view_t const& b) {
  return utl::any_of(a.way_,
                     [&](way_idx_t const x) { return is_connected(c, x, b); });
}

// Returns false if no candidates of `from` and `to` are connected.
template <Profile P>
bool restrict_to_connected(ways const& w,
                           sharing_data const* sharing,
                           match_view_t& from,
                           match_view_t& to,
                           match_result& storage) {
  assert(storage.empty());
  auto const* components = reachability_components<P>(w, sharing);
  if (components == nullptr) {
    return true;
  }
  if (!is_connected(*components, from, to)) {
    return false;
  }
  storage.append(from, [&](way_idx_t const x) {
    return is_connected(*components, x, to);
  });
  storage.append(to, [&](way_idx_t const x) {
    return is_connected(*components, x, from);
  });
  from = storage[match_idx_t{0U}];
  to = storage[match_idx_t{1U}];
  return true;
}

}  // namespace osr
