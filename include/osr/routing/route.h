#pragma once

#include <string_view>
#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
// #include "osr/lookup.h"
#include "osr/match_view.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/parameters.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <IsProfile Profile>
struct dijkstra;

template <IsProfile Profile>
struct bidirectional;

struct sharing_data;

struct lookup;

using match_view_t = std::span<way_candidate const>;

template <IsProfile Profile>
bidirectional<Profile>& get_bidirectional();

template <IsProfile Profile>
dijkstra<Profile>& get_dijkstra();

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

std::optional<path> route(profile_parameters const&,
  ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra
                          );

std::vector<std::optional<path>> route(
    profile_parameters const&,
  ways const&,
    lookup const&,
    search_profile const,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

std::optional<path> route(profile_parameters const&,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra);

}  // namespace osr
