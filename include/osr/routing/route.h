#pragma once

#include <chrono>
#include <memory>
#include <optional>
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

struct one_to_many_state {
  virtual ~one_to_many_state() = default;
  virtual std::vector<std::optional<path>> const& results() const = 0;
  virtual std::optional<path> reconstruct(ways const&,
                                          lookup const&,
                                          std::size_t dest_idx,
                                          sharing_data const*) = 0;
};

std::unique_ptr<one_to_many_state> route_one_to_many(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    std::chrono::seconds max_duration,
    direction,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt);

// One Pareto-optimal path of route_pareto.
struct pareto_path {
  duration_t duration() const { return walk_ + fixed_; }

  duration_t walk_;  // walking time
  duration_t fixed_;  // elevator rides (not scaled by walking speed)
  path_accessibility accessibility_;
};

// Multi-criteria one-to-many search (kFoot / kWheelchair): labels (walking
// time, fixed time, accessibility record), one Pareto bag per node. Per
// destination: all Pareto-optimal paths (both times <=, record at most as
// restricted).
std::vector<std::vector<pareto_path>> route_pareto(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    match_view_t const& from_match,
    match_result const& to_match,
    std::chrono::seconds max_duration,
    direction,
    bitvec<node_idx_t> const* blocked = nullptr,
    elevation_storage const* = nullptr);

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    std::chrono::seconds max_duration,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt);

std::optional<path> route(profile_parameters const&,
                          ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          std::chrono::seconds max_duration,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt);

std::optional<path> route_bidirectional(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    location const& to,
    std::chrono::seconds max_duration,
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
    std::chrono::seconds max_duration,
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
                                std::chrono::seconds max_duration,
                                direction,
                                double max_match_distance,
                                bitvec<node_idx_t> const* blocked = nullptr,
                                sharing_data const* sharing = nullptr,
                                elevation_storage const* = nullptr,
                                std::optional<routing_time_t> = std::nullopt);

std::optional<path> route(profile_parameters const&,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t const& from_match,
                          match_view_t const& to_match,
                          std::chrono::seconds max_duration,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra,
                          std::optional<routing_time_t> = std::nullopt);

}  // namespace osr
