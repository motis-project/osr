#pragma once

#include <cstdint>
#include <algorithm>
#include <ranges>
#include <span>
#include <vector>

#include "utl/erase_duplicates.h"
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

inline std::uint64_t component_key(class_components const& components,
                                   ways const& w,
                                   way_idx_t const way,
                                   match_result::nodes const& n) {
  return components.key(way, candidate_edge(w, way, n));
}

// Component keys of all candidates of `m` (one match or all matches), sorted,
// without duplicates.
template <typename Match>
std::vector<std::uint64_t> component_keys(class_components const& components,
                                          ways const& w,
                                          Match const& m) {
  auto keys = std::vector<std::uint64_t>(m.way_.size());
  std::ranges::transform(
      m.way_, m.nodes_, begin(keys),
      [&](way_idx_t const way, match_result::nodes const& n) {
        return component_key(components, w, way, n);
      });
  utl::erase_duplicates(keys);
  return keys;
}

inline bool shares_component(
    class_components const& components,
    ways const& w,
    match_view_t const& m,
    std::size_t const j,
    std::span<std::uint64_t const> const other_component_keys) {
  return std::ranges::binary_search(
      other_component_keys,
      component_key(components, w, m.way_[j], m.nodes_[j]));
}

inline bool shares_component(
    class_components const& components,
    ways const& w,
    match_view_t const& m,
    std::span<std::uint64_t const> const other_component_keys) {
  return utl::any_of(
      std::views::iota(std::size_t{0U}, m.size()), [&](std::size_t const j) {
        return shares_component(components, w, m, j, other_component_keys);
      });
}

template <Profile P, typename Match>
match_result connected_candidates(ways const& w,
                                  sharing_data const* sharing,
                                  match_view_t const& m,
                                  Match const& other) {
  auto const* components = reachability_components<P>(w, sharing);
  auto out = match_result{};
  if (components == nullptr) {
    out.append(m, [](std::size_t) { return true; });
    return out;
  }
  auto const other_component_keys = component_keys(*components, w, other);
  out.append(m, [&](std::size_t const j) {
    return shares_component(*components, w, m, j, other_component_keys);
  });
  return out;
}

}  // namespace osr
