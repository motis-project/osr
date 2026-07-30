#pragma once

#include <string_view>
#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/mode.h"
#include "osr/routing/parameters.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <Profile, bool EarlyTermination>
struct dijkstra;

template <Profile, bool EarlyTermination>
struct astar;

template <Profile>
struct bidirectional;

struct sharing_data;

struct route_endpoint_options {
  bool exact_return_at_from_{};
  std::vector<bool> exact_return_at_to_{};

  bool exact_return_at_to(std::size_t const i) const {
    return i < exact_return_at_to_.size() && exact_return_at_to_[i];
  }
};

template <Profile P>
bidirectional<P>& get_bidirectional();

template <Profile P>
dijkstra<P, false>& get_dijkstra();

template <Profile P>
astar<P, false>& get_astar();

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
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt,
    route_endpoint_options const& = {});

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
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt,
                          route_endpoint_options const& = {});

std::optional<path> route_bidirectional(
    profile_parameters const&,
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
    elevation_storage const* = nullptr);

std::optional<path> route_dijkstra(
    profile_parameters const&,
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
    std::optional<routing_time_t> = std::nullopt);

std::optional<path> route_astar(profile_parameters const&,
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
                                std::optional<routing_time_t> = std::nullopt);

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile const,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    cost_t const max,
    direction const,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt);

std::optional<path> route(profile_parameters const&,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t const& from_match,
                          match_view_t const& to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt);

}  // namespace osr
