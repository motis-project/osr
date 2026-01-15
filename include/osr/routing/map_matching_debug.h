#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "boost/json.hpp"

#include "geo/latlng.h"

#include "osr/lookup.h"
#include "osr/routing/map_matching.h"
#include "osr/routing/map_matching_data.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct debug_node_ref {
  std::size_t node_array_idx_{};
  way_pos_t way_pos_{};
  direction dir_{};
};

struct debug_match_point_segment {
  bool valid_{false};
  std::size_t node_array_idx_{};
  double dist_to_node_{};
  cost_t cost_{};
};

struct debug_route_step {
  std::size_t node_array_idx_{};
  way_pos_t way_pos_{};
  direction dir_{};
  cost_t cost_{};
  std::optional<std::size_t> pred_node_array_idx_{};
  std::optional<std::size_t> way_array_idx_{};
};

struct debug_route_result {
  bool reached_{false};
  cost_t cost_{kInfeasible};
  std::vector<debug_route_step> path_{};
  std::vector<geo::latlng> geometry_{};
};

struct debug_start_label {
  cost_t match_penalty_{};
  cost_t prev_segment_cost_{};
  cost_t cost_offset_{};
  cost_t total_start_cost_{};
};

struct debug_match {
  std::size_t match_idx_{};
  std::size_t way_array_idx_{};
  geo::latlng projected_point_{};
  double dist_to_way_{};
  unsigned way_segment_idx_{};
  bool oneway_{};
  std::size_t additional_node_array_idx_{};
  std::optional<debug_node_ref> fwd_node_{};
  std::optional<debug_node_ref> bwd_node_{};
  debug_match_point_segment fwd_out_{}, fwd_in_{}, bwd_out_{}, bwd_in_{};
  std::optional<debug_start_label> start_label_{};
  debug_route_result fwd_result_{}, bwd_result_{};
  std::optional<std::size_t> beeline_from_match_idx_{};
  std::optional<distance_t> beeline_dist_{};
};

struct debug_beeline {
  std::string reason_{};
  std::optional<std::size_t> from_match_idx_{};
  std::optional<std::size_t> to_match_idx_{};
  geo::latlng from_point_{};
  geo::latlng to_point_{};
  distance_t distance_{};
  cost_t cost_{};
};

struct debug_node_label {
  std::string label_str_{};
  cost_t cost_{};
  std::optional<std::size_t> pred_node_array_idx_{};
};

struct debug_route_segment {
  std::size_t segment_idx_{};
  std::size_t from_point_idx_{};
  std::size_t to_point_idx_{};
  bool all_beelined_{false};
  cost_t min_cost_{};
  cost_t max_cost_{};
  cost_t dijkstra_cost_limit_{};
  bool max_reached_in_dijkstra_{false};
  std::vector<debug_match> start_matches_{};
  std::vector<debug_match> dest_matches_{};
  std::optional<debug_beeline> beeline_{};
  hash_map<std::size_t, std::vector<debug_node_label>> node_labels_{};
  std::vector<std::size_t> additional_edge_ways_{};
};

template <Profile P>
void write_map_match_debug(
    ways const& w,
    lookup const& l,
    typename P::parameters const& params,
    std::vector<location> const& points,
    std::vector<point_data<P>> const& pds,
    std::vector<segment_data<P>> const& segments,
    matched_route const& result,
    std::function<geo::latlng(node_idx_t)> const& get_node_pos,
    std::filesystem::path const& debug_path);

}  // namespace osr
