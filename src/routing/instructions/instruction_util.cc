#include "osr/routing/instructions/instruction_util.h"

#include "osr/routing/instructions/module.h"

namespace osr {

bool is_from_motorway(ways const& w, traversed_node_hub const& hub) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[hub.arrive_hub_on_.way_idx_];

  return is_motorway_or_trunk(w, hub.arrive_hub_on_.way_idx_) &&
         !segment_instruction_properties.is_link();
}

bool is_from_ramp(ways const& w, traversed_node_hub const& hub) {
  const auto& segment_instruction_properties =
      w.way_instruction_properties_[hub.arrive_hub_on_.way_idx_];

  return is_motorway_or_trunk(w, hub.arrive_hub_on_.way_idx_) &&
         segment_instruction_properties.is_link();
}

bool is_onto_motorway(ways const& w, traversed_node_hub const& hub) {
  const auto& exit_from_hub = hub.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_hub.segment_.way_idx_];

  return is_motorway_or_trunk(w, exit_from_hub.segment_.way_idx_) &&
         !segment_instruction_properties.is_link();
}

bool is_onto_ramp(ways const& w, traversed_node_hub const& hub) {
  const auto& exit_from_hub = hub.get_exit();

  const auto& segment_instruction_properties =
      w.way_instruction_properties_[exit_from_hub.segment_.way_idx_];

  return is_motorway_or_trunk(w, exit_from_hub.segment_.way_idx_) &&
         segment_instruction_properties.is_link();
}


bool is_last_segment(segment_contexts_window const& window) {
  return !window.has(1);
}

bool ways_have_same_name(way_idx_t const w1, way_idx_t const w2, ways const& w) {
  const string_idx_t s1 = w.way_names_[w1];
  if (s1 == string_idx_t::invalid()) {
    return false;
  }

  const string_idx_t s2 = w.way_names_[w2];
  if (s2 == string_idx_t::invalid()) {
    return false;
  }

  return s1 == s2;
}

bool ways_have_same_highway(way_idx_t const w1,
                            way_idx_t const w2,
                            ways const& w) {
  return w.way_instruction_properties_[w1].get_highway() ==
         w.way_instruction_properties_[w2].get_highway();
}

bool is_roundabout_way(ways const& w, way_idx_t const way) {
  if (way == way_idx_t::invalid()) {
    return false;
  }
  const auto junction = w.way_instruction_properties_[way].get_junction();
  return junction == junction::roundabout ||
         junction == junction::circular;
}

bool is_link(ways const& w, way_idx_t const way) {
  return w.way_instruction_properties_[way].is_link();
}

bool is_motorway_or_trunk(ways const& w, way_idx_t const way) {
  const auto h = w.way_instruction_properties_[way].get_highway();
  return h == highway::motorway || h == highway::trunk;
}

bool turn_onto_through_street(ways const& w, traversed_node_hub const& hub) {

  const auto& exit = hub.get_exit();
  for (std::size_t alt_idx = 1U; alt_idx < hub.alternatives_.size(); ++alt_idx) {
    if (alt_idx == hub.exit_from_hub_idx_) {
      continue;
    }

    const auto& alt = hub.alternatives_[alt_idx];

    const bool is_nearly_straight = std::abs(alt.angle_with_exit_) > 160.0;
    const bool same_name =
        ways_have_same_name(alt.segment_.way_idx_,
                            exit.segment_.way_idx_, w);
    const bool same_highway =
        ways_have_same_highway(alt.segment_.way_idx_,
                               exit.segment_.way_idx_, w);

    if (is_nearly_straight && same_name && same_highway) {
      return true;
    }
  }

  return false;
}

bool is_more_important_by(highway const h1,
                          highway const h2,
                          unsigned const diff) {
  return h1 + diff <= h2;
}

bool is_segment_mode(path::segment const& segment, mode const m) {
  return segment.mode_ == m;
}

way_idx_t get_way_from(segment_context const& context) {
  return context.src_ptr_->way_;
}

std::pair<routing_instruction, routing_instruction> get_instruction_for_fork(
    ways const& w, traversed_node_hub const& hub, size_t const left, size_t const right) {
  utl::verify(left < right, "Alternatives are ordered in circular order");

  const auto& alt_left = hub.alternatives_[left];
  const auto& alt_right = hub.alternatives_[right];

  const auto alt_left_direction =
      get_relative_direction(alt_left.angle_with_arrive_);
  const auto alt_right_direction =
    get_relative_direction(alt_right.angle_with_arrive_);

  const auto& left_props =
    w.way_instruction_properties_[alt_left.segment_.way_idx_];
  const auto& right_props =
  w.way_instruction_properties_[alt_right.segment_.way_idx_];

  const auto left_highway = left_props.get_highway();
  const auto right_highway = right_props.get_highway();

  if (alt_left_direction == relative_direction::kStraight &&
      alt_right_direction != relative_direction::kStraight) {

    if (is_more_important_by(right_highway, left_highway, 1)) {
      return {stay_instruction{relative_direction::kLeft}, stay_instruction{relative_direction::kRight}};
    }

    return {none_instruction{}, stay_instruction{relative_direction::kRight}};
  } else if (alt_right_direction == relative_direction::kStraight &&
             alt_left_direction != relative_direction::kStraight) {

    if (is_more_important_by(left_highway, right_highway, 1)) {
      return {stay_instruction{relative_direction::kLeft}, stay_instruction{relative_direction::kRight}};
    }

    return {stay_instruction{relative_direction::kLeft}, none_instruction{}};
  }

  return {stay_instruction{relative_direction::kLeft}, stay_instruction{relative_direction::kRight}};
}

std::tuple<routing_instruction, routing_instruction, routing_instruction>
get_instruction_for_fork(ways const& w,
                         traversed_node_hub const& hub,
                         size_t const left,
                         size_t const center,
                         size_t const right) {
  utl::verify(left < center && center < right,
              "Alternatives are ordered in circular order");

  const auto& alt_left = hub.alternatives_[left];
  const auto& alt_center = hub.alternatives_[center];
  const auto& alt_right = hub.alternatives_[right];

  if (alt_left.can_exit_hub() &&
      alt_center.can_exit_hub() &&
      alt_right.can_exit_hub()) {

    const auto alt_left_direction =
      get_relative_direction(alt_left.angle_with_arrive_);
    if (alt_left_direction == relative_direction::kStraight) {
      return {
        stay_instruction{relative_direction::kStraight},
        stay_instruction{relative_direction::kSlightRight},
        stay_instruction{relative_direction::kRight}
      };
    }

    const auto alt_right_direction =
      get_relative_direction(alt_right.angle_with_arrive_);
    if (alt_right_direction == relative_direction::kStraight) {
      return {
        stay_instruction{relative_direction::kLeft},
        stay_instruction{relative_direction::kSlightLeft},
        stay_instruction{relative_direction::kStraight}
      };
    }

    // center is somewhat straight
    return {
      stay_instruction{relative_direction::kLeft},
      stay_instruction{relative_direction::kStraight},
      stay_instruction{relative_direction::kRight}
    };
  }

  if (alt_left.can_exit_hub()) {
    if (alt_right.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, hub, left, right);
      return {
        fork_instructions.first,
        none_instruction{},
        fork_instructions.second};
    }
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
          get_instruction_for_fork(w, hub, left, center);
      return {
        fork_instructions.first,
        fork_instructions.second,
        none_instruction{}
      };
    }

    return {
      stay_instruction{relative_direction::kLeft},
      none_instruction{},
      none_instruction{}
    };
  }

  if (alt_right.can_exit_hub()) {
    if (alt_center.can_exit_hub()) {
      const auto fork_instructions =
        get_instruction_for_fork(w, hub, center, right);

      return {
        none_instruction{},
        fork_instructions.first,
        fork_instructions.second
      };
    }

    return {
      none_instruction{},
      none_instruction{},
      stay_instruction{relative_direction::kRight}
    };
  }


  if (alt_center.can_exit_hub()) {
    return {
      none_instruction{},
      stay_instruction{relative_direction::kStraight},
      none_instruction{}
    };
  }

  return {
    none_instruction{},
    none_instruction{},
    none_instruction{}
  };
}

std::optional<traversed_node_hub> get_traversed_node_hub(
    path::segment const& segment, path::segment const& next_segment,
    ways const& w) {
  if (segment.way_ == way_idx_t::invalid() ||
      next_segment.way_ == way_idx_t::invalid()) {
    return std::nullopt;
  }

  if (segment.from_ == node_idx_t::invalid() || segment.to_ == node_idx_t::invalid() ||
      next_segment.from_ == node_idx_t::invalid() || next_segment.to_ == node_idx_t::invalid()) {
    return std::nullopt;
  }

  return traversed_node_hub::from(w, segment.from_, segment.way_, segment.to_,
                                  next_segment.way_, next_segment.to_,
                                  !segment.is_reverse_, !next_segment.is_reverse_,
                                  segment.mode_);
}

unsigned long number_of_possible_turns_at(traversed_node_hub const& hub,
                                          bool const left) {
  constexpr auto is_possible =
      [](traversed_node_hub::relative_way_segment const& rel_seg) {
        return rel_seg.can_exit_hub_;
      };

  const auto count =
      std::ranges::count_if(hub.get_exit_neighbors(left), is_possible);
  return static_cast<unsigned>(count);
}

std::optional<string_idx_t> get_crossing_street(ways const& w,
                                                traversed_node_hub const& h) {
  const auto& exit_segment = h.get_exit();

  const auto from_way_idx = h.arrive_hub_on_.way_idx_;
  const auto to_way_idx = exit_segment.segment_.way_idx_;

  const auto& from_way_instruction_props =
      w.way_instruction_properties_[from_way_idx];
  const auto& to_way_instruction_props =
      w.way_instruction_properties_[to_way_idx];

  if (from_way_instruction_props.get_highway() != highway::footish ||
      to_way_instruction_props.get_highway() != highway::footish) {
    return std::nullopt;
  }

  const auto left_alts = h.get_exit_neighbors(true);
  const auto right_alts = h.get_exit_neighbors(false);

  string_idx_t crossed_street = string_idx_t::invalid();
  for (const auto& left_alt : left_alts) {
    const auto& left_way_instruction_props =
        w.way_instruction_properties_[left_alt.segment_.way_idx_];

    if (!is_more_important_by(left_way_instruction_props.get_highway(), footish,
                              1)) {
      continue;
    }

    for (const auto& right_alt : right_alts) {
      const auto& right_way_instruction_props =
          w.way_instruction_properties_[right_alt.segment_.way_idx_];

      if (!is_more_important_by(right_way_instruction_props.get_highway(),
                                footish, 1)) {
        continue;
      }

      if (!ways_have_same_name(left_alt.segment_.way_idx_,
                               right_alt.segment_.way_idx_, w)) {
        continue;
      }

      const auto street_found = w.way_names_[left_alt.segment_.way_idx_];
      if (crossed_street == string_idx_t::invalid()) {
        crossed_street = street_found;
      } else if (crossed_street != string_idx_t::invalid() &&
                 crossed_street != street_found) {
        // crossing multiple streets, cannot give definitive answer
        return std::nullopt;
      }
    }
  }
  if (crossed_street != string_idx_t::invalid()) {
    return std::optional{crossed_street};
  }

  return std::nullopt;
}

bool is_merge_on_through_street(ways const& w,
                                traversed_node_hub const& h) {
  const auto coming_from_way = h.arrive_hub_on_.way_idx_;

  const auto exit = h.get_exit();
  const auto exit_way = exit.segment_.way_idx_;
  if (ways_have_same_name(coming_from_way, exit_way, w)) {
    return false;
  }

  const bool is_left_merge = exit.angle_with_arrive_ > 0;

  const auto get_opt_exit_predecessor = [&] -> std::optional<traversed_node_hub::relative_way_segment> {

    const auto neighbor_idx_from = is_left_merge ? h.exit_from_hub_idx_ + 1 : 1;
    const auto neighbor_idx_to = is_left_merge ? h.alternatives_.size() : h.exit_from_hub_idx_;
    for (auto neighbor_idx = neighbor_idx_from; neighbor_idx < neighbor_idx_to; neighbor_idx++) {
      const auto neighbor = h.alternatives_[neighbor_idx];
      const auto neighbor_way = neighbor.segment_.way_idx_;
      const auto angle_with_exit = h.angle_with_exit_from(w, neighbor_idx);
      if (ways_have_same_name(neighbor_way, exit_way, w) && std::abs(angle_with_exit) < 20.0) {
        return std::make_optional(h.alternatives_[neighbor_idx]);
      }
    }

    return std::nullopt;
  };
  const auto opt_exit_predecessor = get_opt_exit_predecessor();
  if (opt_exit_predecessor.has_value()) {
    return true;
  }
  return false;
}

bool is_pedestrian(mode const m) {
  return m == mode::kFoot || m == mode::kWheelchair;
}

} // namespace osr