#pragma once

#include <vector>

#include "osr/routing/instructions/directions.h"
#include "osr/routing/path.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct traversed_node_hub {

  struct way_osm_nodes_idx_range {

    way_osm_nodes_idx_range flip() const;

    std::uint16_t from_;
    std::uint16_t to_; // inclusive
  };

  struct way_segment {

    bool is_way_aligned() const;

    static way_segment from(const ways& w, way_idx_t way, osm_node_idx_t from,
                            osm_node_idx_t to);

    way_segment flip_directions() const;

    node_idx_t get_from_idx(ways const& w) const;
    node_idx_t get_to_idx(ways const& w) const;

    way_idx_t way_idx_;
    way_osm_nodes_idx_range osm_node_range_;
  };

  struct relative_way_segment {

    bool can_enter_hub() const;

    bool can_exit_hub() const;

    bool is_opposite_of_arrive() const;

    double angle_with_exit_;
    double angle_with_arrive_;

    bool can_enter_hub_;
    bool can_exit_hub_;
    bool is_opposite_of_arrive_;

    way_segment segment_;
  };

  static traversed_node_hub from(ways const& w, node_idx_t prev_node,
                                 way_idx_t arrive_way, node_idx_t hub_node,
                                 way_idx_t depart_way, node_idx_t next_node,
                                 mode m);

  unsigned long number_of_possible_turns(bool consider_uturn) const;

  unsigned long number_of_visible_turns(bool consider_uturn) const;

  double angle_with_exit_from(ways const& w, std::size_t alt_idx) const;



  void print(std::ostream& out, ways const& w) const;

  relative_way_segment get_exit() const;

  auto get_exit_neighbors(bool left) const {
    if (left) {
      // From index 1 (omit u-turn) to exit_from_hub_idx_ (exclusive)
      return std::ranges::subrange(
          alternatives_.begin() + 1,
          alternatives_.begin() + static_cast<long>(exit_from_hub_idx_));
    } else {
      // From index exit_from_hub_idx_ + 1 to the end
      return std::ranges::subrange(
          alternatives_.begin() + static_cast<long>(exit_from_hub_idx_) + 1,
          alternatives_.end());
    }
  }

  bool is_exit_natural_choice(ways const& w) const;

  std::string to_json(ways const& w) const;

  way_segment arrive_hub_on_;
  node_idx_t hub_node_;
  std::vector<relative_way_segment> alternatives_{};
  size_t exit_from_hub_idx_;
};

struct segment_context {
  path::segment* src_ptr_;
  std::optional<traversed_node_hub> traversed_node_hub_;
  relative_direction relative_direction_;
};

} // namespace osr
