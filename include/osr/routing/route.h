#pragma once

#include <string_view>
#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/mode.h"
#include "osr/routing/profile.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <typename Profile>
struct dijkstra;

template <typename Profile>
struct bidirectional;

struct sharing_data;

struct routing_parameters {
  float speed_{1.2F};
};

constexpr auto const kRoutingParameters = routing_parameters{};
 
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
  node_idx_t track_node_{node_idx_t::invalid()};
};

template <typename Profile>
bidirectional<Profile>& get_bidirectional();

template <typename Profile>
dijkstra<Profile>& get_dijkstra();

std::vector<std::optional<path>> route(
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

std::optional<path> route(ways const&,
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
                          routing_parameters = kRoutingParameters
                          );

std::vector<std::optional<path>> route(
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

std::optional<path> route(ways const& w,
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
