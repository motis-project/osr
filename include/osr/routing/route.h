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

  // Sharing profiles: where the ride is on the path to destination
  // `dest_idx`, as tracked durations from the search start: the first and the
  // last rental label, the non-rental labels around them, the destination's
  // node and the final matching piece. These are the times reconstruct() lays
  // out in the segments, so a caller can place the ride on the timeline the
  // path is shown with (GTFS-Flex pickup and drop-off windows apply to the
  // ride, not to the walks around it). In search order the ride spans
  // [before_min_, max_]: like the segments' modes, the switch to the vehicle
  // counts as riding, the return as walking. If the vehicle is returned at the
  // destination coordinate, the matching piece at that end is ridden too and
  // the ride reaches the end of the path (max_ = dest_node_ + dest_match_ when
  // the destination node is the rental node; before_min_ = 0 when the search
  // root is, in a backward search). Walks the predecessor chain only, no
  // geometry. nullopt if the path uses no vehicle (or the profile has none).
  struct rental_duration_info {
    duration_t min_;  // rental label closest to the search start
    duration_t max_;  // rental label farthest from the search start, or the
                      // path end if the final matching piece is ridden
    duration_t before_min_;  // non-rental label preceding min_ (search
                             // order), 0 if there is none
    duration_t after_max_;  // non-rental label following max_ (search order),
                            // the path end if there is none
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
    std::chrono::seconds max_duration,
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
