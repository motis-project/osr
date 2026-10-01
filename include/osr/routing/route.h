#pragma once

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

  // Sharing profiles: where the ride is on the path to destination
  // `dest_idx`, as tracked durations from the search start: the first and the
  // last rental label, the foot labels around them, and the destination's
  // node (the path without the final matching). These are the times
  // reconstruct() lays out in the segments, so a caller can place the ride on
  // the timeline the path is shown with (GTFS-Flex pickup and drop-off
  // windows apply to the ride, not to the walks around it). Walks the
  // predecessor chain only, no geometry. nullopt if the path uses no vehicle
  // (or the profile has none).
  struct rental_duration_info {
    duration_t min_;  // rental label closest to the search start
    duration_t max_;  // rental label farthest from the search start
    duration_t before_min_;  // non-rental label preceding min_ (search order)
    duration_t after_max_;  // non-rental label following max_ (search order)
    duration_t dest_node_;  // destination node
    duration_t dest_match_;  // final matching piece as reconstruct() shows it
  };
  virtual std::optional<rental_duration_info> rental_durations(
      std::size_t /* dest_idx */) const {
    return std::nullopt;
  }
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
    cost_t max,
    direction,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct =
        [](path const&) { return false; },
    std::optional<routing_time_t> = std::nullopt);

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
    std::optional<routing_time_t> = std::nullopt);

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
                          std::optional<routing_time_t> = std::nullopt);

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
