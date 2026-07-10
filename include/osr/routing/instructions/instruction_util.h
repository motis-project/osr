#pragma once

#include "osr/routing/instructions/module.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

bool is_from_motorway(ways const& w, traversed_node_hub const& hub);

bool is_from_ramp(ways const& w, traversed_node_hub const& hub);

bool is_onto_motorway(ways const& w, traversed_node_hub const& hub);

bool is_onto_ramp(ways const& w, traversed_node_hub const& hub);

bool is_last_segment(segment_contexts_window const& window);

bool ways_have_same_name(way_idx_t w1, way_idx_t w2, ways const& w);

bool ways_have_same_highway(way_idx_t w1, way_idx_t w2, ways const& w);

bool is_roundabout_way(ways const& w, way_idx_t way);

bool is_link(ways const& w, way_idx_t way);

bool is_motorway_or_trunk(ways const& w, way_idx_t way);

bool turn_onto_through_street(ways const& w, traversed_node_hub const& hub);

bool is_more_important_by(highway h1,
                          highway h2,
                          unsigned diff);

bool is_segment_mode(path::segment const& segment, mode m);

way_idx_t get_way_from(segment_context const& context);

std::pair<routing_instruction, routing_instruction> get_instruction_for_fork(
    ways const& w, traversed_node_hub const& hub, size_t left, size_t right);

std::tuple<routing_instruction, routing_instruction, routing_instruction>
get_instruction_for_fork(ways const& w, traversed_node_hub const& hub,
                         size_t left, size_t center, size_t right);

std::optional<traversed_node_hub> get_traversed_node_hub(
    path::segment const& segment, path::segment const& next_segment,
    ways const& w);

std::optional<string_idx_t> get_crossing_street(ways const& w,
                                                traversed_node_hub const& h);

bool is_merge_on_through_street(ways const& w,
                                traversed_node_hub const& h);

bool is_pedestrian(mode const m);

} // namespace osr
