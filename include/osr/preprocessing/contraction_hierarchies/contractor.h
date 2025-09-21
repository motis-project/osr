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
  std::vector<std::vector<neighbor_data>> outgoing_neighbors_;
  std::vector<std::vector<neighbor_data>> incoming_neighbors_;

  static bool has_uturn(way_idx_t const from,
                          direction const from_dir,
                          way_idx_t const to,
                          direction const to_dir,
                          shortcut_storage const* shortcut_storage) {
    auto const from_way_and_dir =
        shortcut_storage->resolve_last_way_and_dir(from, from_dir);
    auto const to_way_and_dir =
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
  static void add_loop(
    ways& w,
    shortcut_storage* shortcuts,
    node_idx_t node,
                       const bypass_path& bypass
  );
  /*bool has_shortcut(node_idx_t from, node_idx_t to, way_idx_t from_way, way_idx_t to_way, direction from_dir, direction to_dir, shortcut_storage* shortcut_storage) {
    auto const& neighbors = outgoing_neighbors_[to_idx(from)];
    return std::ranges::any_of(neighbors, [to, from_way, to_way, from_dir, to_dir, shortcut_storage](auto const& neighbor) {
      if (neighbor.node_idx == to) {
        if (shortcut_storage->is_shortcut(neighbor.way)) {
          auto const* sh = shortcut_storage->get_shortcut(neighbor.way);
          auto const from_resolved = shortcut_storage->resolve_first_way_and_dir(sh->upward_way, sh->downward_dir);
          auto const to_resolved = shortcut_storage->resolve_last_way_and_dir(sh->downward_way, sh->downward_dir);
          return from_resolved.way == from_way && to_resolved.way == to_way && from_dir == from_resolved.dir && to_dir == to_resolved.dir;
        }
      }
      return false;
    });
  }*/

  std::vector<bypass_path> find_restriction_bypasses(ways const& w,
                                                  node_idx_t node,
                                                  node_idx_t from_node,
                                                  way_idx_t from_way,
                                                  direction from_dir,
                                                  shortcut_storage const* shortcut_storage) const;
  static std::vector<way_pos_t> get_restriction_to_ways(
      ways const& w,
      node_idx_t const node,
      way_idx_t const from_way,
      shortcut_storage const* shortcut_storage) {
    std::vector<way_pos_t> to_ways;

    if (!w.r_->node_is_restricted_[node]) {
      return to_ways;
    }
    auto const from_way_pos = w.r_->get_way_pos(node, from_way);

    for (auto const& node_restrictions = w.r_->node_restrictions_[node];
         const auto& [from_, to_] : node_restrictions) {
      if (from_ == from_way_pos && !shortcut_storage->is_shortcut(w.r_->node_ways_[node][to_])) {
        to_ways.push_back(to_);
      }
    }
    return to_ways;
  }
  static bool is_restriction_subset_at_node(ways const& w, node_idx_t const node, way_pos_t const way1_pos, way_pos_t const way2_pos) {
    if (!w.r_->node_is_restricted_[node]) {
      return true;
    }
    for (auto const& node_restrictions = w.r_->node_restrictions_[node];
         const auto& [from_, to_] : node_restrictions) {
      if (from_ == way2_pos) {
        bool found_matching = false;
        for (const auto& [way1_from_, way1_to_] : node_restrictions) {
          if (way1_from_ == way1_pos && way1_to_ == to_) {
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

  static cost_t get_possible_shortcut_cost(neighbor_data const& from,
                                           neighbor_data const& to,
                                           shortcut_storage const* shortcut_storage) {
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
