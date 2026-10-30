#pragma once

#include <span>

#include "utl/helpers/algorithm.h"

#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <Profile P>
class_components const* reachability_components(ways const& w,
                                                sharing_data const* sharing) {
  return P::kReachabilityClass.has_value() && sharing == nullptr
             ? &w.r_->get_class_components(*P::kReachabilityClass)
             : nullptr;
}

inline bool is_connected(class_components const& c,
                         way_idx_t const a,
                         std::span<way_idx_t const> const other_ways) {
  return utl::any_of(other_ways,
                     [&](way_idx_t const b) { return c.is_connected(a, b); });
}

inline bool is_connected(class_components const& c,
                         match_view_t const& a,
                         match_view_t const& b) {
  return utl::any_of(
      a.way_, [&](way_idx_t const x) { return is_connected(c, x, b.way_); });
}

template <Profile P>
match_result connected_candidates(ways const& w,
                                  sharing_data const* sharing,
                                  match_view_t const& m,
                                  std::span<way_idx_t const> const other_ways) {
  auto const* components = reachability_components<P>(w, sharing);
  auto out = match_result{};
  out.append(m, [&](way_idx_t const x) {
    return components == nullptr || is_connected(*components, x, other_ways);
  });
  return out;
}

}  // namespace osr
