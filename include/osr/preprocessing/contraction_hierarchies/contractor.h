#pragma once
#include "osr/routing/profiles/car.h"
#include "osr/routing/dijkstra.h"
#include "osr/types.h"
#include "storage.h"

namespace osr::ch {

struct neighbor_data {
  node_idx_t node_idx;
  std::uint32_t cost;
  way_idx_t way;
  distance_t distance;
  car::node node;
};

struct contractor {
  std::unordered_map<node_idx_t, std::vector<neighbor_data>> outgoing_neighbors_;
  std::unordered_map<node_idx_t, std::vector<neighbor_data>> incoming_neighbors_;

  static bool has_uturn(way_idx_t from,
                          direction from_dir,
                          way_idx_t to,
                          direction to_dir,
                          shortcut_storage* shortcut_storage) {
    way_and_dir from_way_and_dir =
        shortcut_storage->resolve_last_way_and_dir(from, from_dir);
    way_and_dir to_way_and_dir =
        shortcut_storage->resolve_first_way_and_dir(to, to_dir);
    return (from_way_and_dir.way == to_way_and_dir.way &&
        from_way_and_dir.dir == opposite(to_way_and_dir.dir));
  }

  struct bypass_path {
    std::vector<car::node> nodes;
    vec<ShortcutSegment> segments;
    cost_t total_cost{0};
    distance_t total_distance{0};

    bool is_distinct_from(const bypass_path& other) const {
      if (nodes.empty() || other.nodes.empty()) {
        return false;
      }
      return !(nodes.front() == other.nodes.front() &&
        nodes.back() == other.nodes.back());
    }
  };
  std::vector<bypass_path> find_restriction_bypasses(ways const& w,
                                                  bitvec<node_idx_t>* blocked,
                                                  node_idx_t node,
                                                  node_idx_t from_node,
                                                  way_idx_t from_way,
                                                  direction from_dir,
                                                  shortcut_storage const* shortcut_storage);
  std::vector<way_pos_t> get_restriction_to_ways(ways const& w,
                                           node_idx_t node,
                                           way_idx_t from_way,
                                           shortcut_storage const* shortcut_storage) {
    std::vector<way_pos_t> to_ways;

    if (!w.r_->node_is_restricted_[node]) {
      return to_ways;
    }
    auto const from_way_pos = w.r_->get_way_pos(node, from_way);
    auto const& node_restrictions = w.r_->node_restrictions_[node];

    for (auto const& restriction : node_restrictions) {
      if (restriction.from_ == from_way_pos && !shortcut_storage->is_shortcut(w.r_->node_ways_[node][restriction.to_])) {
        to_ways.push_back(restriction.to_);
      }
    }
    return to_ways;
  }
  bool is_restriction_subset_at_node(ways const& w, node_idx_t node, way_pos_t way1_pos, way_pos_t way2_pos) {
    if (!w.r_->node_is_restricted_[node]) {
      return true;
    }
    auto const& node_restrictions = w.r_->node_restrictions_[node];
    for (auto const& restr : node_restrictions) {
      if (restr.from_ == way2_pos) {
        bool found_matching = false;
        for (auto const& way1_restr : node_restrictions) {
          if (way1_restr.from_ == way1_pos && way1_restr.to_ == restr.to_) {
            found_matching = true;
            break;
          }
        }
        if (!found_matching) {
          return false;
        }
      }
    }
    return true;
  }

  static cost_t get_possible_shortcut_cost(neighbor_data from,
                                           neighbor_data to,
                                           shortcut_storage* shortcut_storage) {
    if (has_uturn(from.way, from.node.dir_, to.way, to.node.dir_,
                  shortcut_storage)) {
      return from.cost + to.cost + car::kUturnPenalty;
                  }
    return from.cost + to.cost;
  }

  void calculate_neighbors(ways const& w);

  void contract_node(ways& w,
                     ways const& w_without,
                     bitvec<node_idx_t>* blocked,
                     shortcut_storage* shortcut_storage,
                     node_idx_t node);
};

} // namespace osr::ch
