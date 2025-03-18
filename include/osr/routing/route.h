#pragma once

#include <string_view>
#include <vector>

#include "geo/polyline.h"

#include "bidirectional.h"
#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/mode.h"
#include "osr/routing/profile.h"
#include "osr/routing/algorithms.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <typename Profile>
struct dijkstra;

struct sharing_data;

struct path {
  struct segment {
    geo::polyline polyline_;
    level_t from_level_{kNoLevel};
    level_t to_level_{kNoLevel};
    node_idx_t from_{node_idx_t::invalid()};
    node_idx_t to_{node_idx_t::invalid()};
    way_idx_t way_{way_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    distance_t dist_{0};
    elevation_storage::elevation elevation_{};
    mode mode_{mode::kFoot};
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  elevation_storage::elevation elevation_{};
  std::vector<segment> segments_{};
  bool uses_elevator_{false};
};

template <typename Profile>
bidirectional<Profile>& get_bidirectional();

template <typename Profile>
dijkstra<Profile>& get_dijkstra();

struct one_to_many_result {
  one_to_many_result(std::chrono::milliseconds&& lookup_time,
                     std::vector<std::optional<path>>&& paths)
      : lookup_time_{lookup_time}, paths_{std::move(paths)} {}

  one_to_many_result(std::vector<std::optional<path>>&& paths)
      : paths_{std::move(paths)} {}

  std::chrono::milliseconds lookup_time_{};
  std::vector<std::optional<path>> paths_;
};

one_to_many_result route(
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

std::optional<path> route_dijkstra(ways const&,
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

std::optional<path> route_bidirectional(ways const&,
                                   lookup const&,
                                   search_profile,
                                   location const& from,
                                   location const& to,
                                   cost_t max,
                                   direction,
                                   double max_match_distance,
                                   bitvec<node_idx_t> const* blocked = nullptr,
                                   sharing_data const* sharing = nullptr);

std::optional<path> route_dijkstra(ways const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr);

std::vector<std::optional<path>> route(
    ways const&,
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

std::optional<path> route(ways const&,
                          lookup const&,
                          search_profile,
                          routing_algorithm,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr);
}  // namespace osr
