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

std::pair<instruction_action, instruction_action> get_instruction_for_fork(
    ways const& w, traversed_node_hub const& hub, size_t left, size_t right);

std::tuple<instruction_action, instruction_action, instruction_action>
get_instruction_for_fork(ways const& w, traversed_node_hub const& hub,
                         size_t left, size_t center, size_t right);

} // namespace osr
