#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <algorithm>

#include "osr/types.h"
#include "osr/ways.h"
#include "osr/elevation_storage.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"

namespace osr {

template <typename Profile>
struct ch_preprocessor {
  using node = typename Profile::node;
  using label = typename Profile::label;
  using dijkstra_t = dijkstra<Profile>;
  using parameters_t = typename Profile::parameters;

  struct shortcut {
    node from_;
    node to_;
    cost_t cost_;
    node via_;
  };

  void run(ways const& w,
           ways::routing const& r,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    calculate_node_priorities(w, r, blocked, sharing, elevations);
    contract_all_nodes(w, r, blocked, sharing, elevations);
  }

  void calculate_node_priorities(ways const& w,
                                 ways::routing const& r,
                                 bitvec<node_idx_t> const* blocked,
                                 sharing_data const* sharing,
                                 elevation_storage const* elevations) {
    for (auto const& osm_node_idx : w.node_to_osm_) {
      auto const n = get_node(r, w.get_node_idx(osm_node_idx));
      auto priority = calculate_priority(n, w, r, blocked, sharing, elevations);
      node_priorities_[n] = priority;
    }
  }

  int calculate_priority(node const& n,
                         ways const& w,
                         ways::routing const& r,
                         bitvec<node_idx_t> const* blocked,
                         sharing_data const* sharing,
                         elevation_storage const* elevations) {
    auto in_neighbors =
        get_neighbors(r, n, blocked, sharing, elevations, direction::kBackward);
    auto out_neighbors =
        get_neighbors(r, n, blocked, sharing, elevations, direction::kForward);

    int shortcuts_count = 0;
    int const edges_removed = static_cast<int>(in_neighbors.size() + out_neighbors.size());

    for (auto const& v : in_neighbors) {
      for (auto const& u : out_neighbors) {
        if (v != u &&
            is_shortcut(v, n, u, w, r, blocked, sharing, elevations)) {
          ++shortcuts_count;
        }
      }
    }
    return shortcuts_count - edges_removed;
  }

  void contract_all_nodes(ways const& w,
                          ways::routing const& r,
                          bitvec<node_idx_t> const* blocked,
                          sharing_data const* sharing,
                          elevation_storage const* elevations) {
    using priority_pair = std::pair<int, node>;
    std::priority_queue<priority_pair, std::vector<priority_pair>, std::greater<>> pq;

    for (auto const& [n, priority] : node_priorities_) {
      pq.emplace(priority, n);
    }

    int level = 0;
    std::unordered_set<node> contracted;
    while (!pq.empty()) {
      auto [priority, n] = pq.top();
      pq.pop();

      if (contracted.contains(n)) {
        continue;
      }

      auto const current_priority =
          calculate_priority(n, w, r, blocked, sharing, elevations);
      if (current_priority != priority) {
        node_priorities_[n] = current_priority;
        pq.emplace(current_priority, n);
        continue;
      }

      auto affected_nodes = contract_node(n, r, w, blocked, sharing, elevations);
      contracted.emplace(n);
      node_levels_[n] = level++;

      for (auto const& affected : affected_nodes) {
        if (!contracted.contains(affected)) {
          auto new_priority =
              calculate_priority(affected, w, r, blocked, sharing, elevations);
          node_priorities_[affected] = new_priority;
          pq.emplace(new_priority, affected);
        }
      }
    }
  }

  bool is_shortcut(node const& v,
                   node const& n,
                   node const& u,
                   ways const& w,
                   ways::routing const& r,
                   bitvec<node_idx_t> const* blocked,
                   sharing_data const* sharing,
                   elevation_storage const* elevations) {
    auto const cost_vn = get_cost(v, n, r, blocked, sharing, elevations);
    auto const cost_nu = get_cost(n, u, r, blocked, sharing, elevations);
    if (cost_vn >= kInfeasible || cost_nu >= kInfeasible) { return false; }

    auto const shortcut_cost = static_cast<cost_t>(cost_vn + cost_nu);
    auto const limit = shortcut_cost >= kInfeasible - 1 ? kInfeasible : static_cast<cost_t>(shortcut_cost + 1U);
    auto const witness_cost = local_dijkstra(v, u, w, r, blocked, sharing, elevations, n, limit);
    return witness_cost > shortcut_cost;
  }

  std::unordered_set<node> contract_node(node const& u,
                                         ways::routing const& r,
                                         ways const& w,
                                         bitvec<node_idx_t> const* blocked,
                                         sharing_data const* sharing,
                                         elevation_storage const* elevations) {
    auto in_neighbors =
        get_neighbors(r, u, blocked, sharing, elevations, direction::kBackward);
    auto out_neighbors =
        get_neighbors(r, u, blocked, sharing, elevations, direction::kForward);

    std::unordered_set<node> affected_nodes;

    for (auto const& v_i : in_neighbors) {
      auto const cost_vu = get_cost(v_i, u, r, blocked, sharing, elevations);
      if (cost_vu >= kInfeasible) { continue; }
      affected_nodes.insert(v_i);

      for (auto const& w_j : out_neighbors) {
        if (v_i == w_j) { continue; }
        auto const cost_uw = get_cost(u, w_j, r, blocked, sharing, elevations);
        if (cost_uw >= kInfeasible) { continue; }
        affected_nodes.insert(w_j);
        auto const shortcut_cost = static_cast<cost_t>(cost_vu + cost_uw);
        auto const limit = shortcut_cost >= kInfeasible - 1 ? kInfeasible : static_cast<cost_t>(shortcut_cost + 1U);
        if (cost_t const witness = local_dijkstra(
                v_i, w_j, w, r, blocked, sharing, elevations, u, limit);
            witness > shortcut_cost) {
          add_shortcut(u, v_i, w_j, shortcut_cost);
        }
      }
    }
    return affected_nodes;
  }

  void add_shortcut(node const& via,
                    node const& from,
                    node const& to,
                    cost_t const cost) {
    // Store (from -> to). If directed asymmetry is needed, remove reverse insertion below.
    auto& out = shortcuts_[from];
    bool updated = false;
    for (auto& s : out) {
      if (s.to_ == to) {
        if (cost < s.cost_) { s.cost_ = cost; s.via_ = via; }
        updated = true;
        break;
      }
    }
    if (!updated) {
      out.push_back(shortcut{.from_ = from, .to_ = to, .cost_ = cost, .via_ = via});
    }

    // Also insert reverse for now (simplifies neighbor queries)
    auto& out_rev = shortcuts_[to];
    updated = false;
    for (auto& s : out_rev) {
      if (s.to_ == from) {
        if (cost < s.cost_) { s.cost_ = cost; s.via_ = via; }
        updated = true;
        break;
      }
    }
    if (!updated) {
      out_rev.push_back(shortcut{.from_ = to, .to_ = from, .cost_ = cost, .via_ = via});
    }
  }

  std::vector<node> get_neighbors(ways::routing const& r,
                                  node const& n,
                                  bitvec<node_idx_t> const* blocked,
                                  sharing_data const* sharing,
                                  elevation_storage const* elevations,
                                  direction const dir) const {
    std::vector<node> neighbors;

    auto collect = [&](auto dir_tag, auto blocked_tag) {
      if constexpr (std::is_same_v<decltype(dir_tag), std::integral_constant<direction, direction::kForward>>) {
        if constexpr (blocked_tag.value) {
          Profile::template adjacent<direction::kForward, true>(
              r, n, blocked, sharing, elevations,
              [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
                  std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
                if (cost < kInfeasible) { neighbors.push_back(neighbor); }
              });
        } else {
          Profile::template adjacent<direction::kForward, false>(
              r, n, blocked, sharing, elevations,
              [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
                  std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
                if (cost < kInfeasible) { neighbors.push_back(neighbor); }
              });
        }
      } else { // backward
        if constexpr (blocked_tag.value) {
          Profile::template adjacent<direction::kBackward, true>(
              r, n, blocked, sharing, elevations,
              [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
                  std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
                if (cost < kInfeasible) { neighbors.push_back(neighbor); }
              });
        } else {
          Profile::template adjacent<direction::kBackward, false>(
              r, n, blocked, sharing, elevations,
              [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
                  std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
                if (cost < kInfeasible) { neighbors.push_back(neighbor); }
              });
        }
      }
    };

    if (dir == direction::kForward) {
      if (blocked == nullptr) { collect(std::integral_constant<direction, direction::kForward>{}, std::false_type{}); }
      else { collect(std::integral_constant<direction, direction::kForward>{}, std::true_type{}); }
    } else {
      if (blocked == nullptr) { collect(std::integral_constant<direction, direction::kBackward>{}, std::false_type{}); }
      else { collect(std::integral_constant<direction, direction::kBackward>{}, std::true_type{}); }
    }

    // Shortcuts (outgoing). For backward neighbors we rely on reverse insertion done in add_shortcut.
    if (auto it = shortcuts_.find(n); it != end(shortcuts_)) {
      for (auto const& sc : it->second) {
        if (blocked != nullptr && blocked->test(sc.to_.get_node())) { continue; }
        neighbors.push_back(sc.to_);
      }
    }
    return neighbors;
  }

  cost_t get_cost(node const& from,
                  node const& to,
                  ways::routing const& r,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations) const {
    cost_t final_cost = kInfeasible;
    // forward adjacency only (caller ensures correct direction semantics)
    if (blocked == nullptr) {
      Profile::template adjacent<direction::kForward, false>(
          r, from, blocked, sharing, elevations,
          [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
              std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
            if (neighbor == to && cost < final_cost) { final_cost = cost; }
          });
    } else {
      Profile::template adjacent<direction::kForward, true>(
          r, from, blocked, sharing, elevations,
          [&](node const neighbor, cost_t const cost, distance_t, way_idx_t,
              std::uint16_t, std::uint16_t, elevation_storage::elevation, bool) {
            if (neighbor == to && cost < final_cost) { final_cost = cost; }
          });
    }
    if (auto it = shortcuts_.find(from); it != end(shortcuts_)) {
      for (auto const& sc : it->second) {
        if (sc.to_ == to) { final_cost = std::min(final_cost, sc.cost_); }
      }
    }
    return final_cost;
  }

  cost_t local_dijkstra(node const& from,
                        node const& to,
                        ways const& w,
                        ways::routing const& r,
                        bitvec<node_idx_t> const* blocked,
                        sharing_data const* sharing,
                        elevation_storage const* elevations) const {
    parameters_t params{};
    dijkstra_t d{};
    d.reset(kInfeasible);
    d.add_start(w, label{from, 0U});
    (void)d.run(params, w, r, kInfeasible, blocked, sharing, elevations, direction::kForward);
    return d.get_cost(to);
  }

  cost_t local_dijkstra(node const& from,
                        node const& to,
                        ways const& w,
                        ways::routing const& r,
                        bitvec<node_idx_t> const* blocked,
                        sharing_data const* sharing,
                        elevation_storage const* elevations,
                        node const& exclude,
                        cost_t const cost_limit) const {
    // Copy blocked bitmap (could be optimized by reusing a shared buffer per contraction)
    bitvec<node_idx_t> tmp{w.n_nodes()};
    if (blocked != nullptr) {
      for (node_idx_t::value_t i = 0U; i < w.n_nodes(); ++i) {
        if (blocked->test(node_idx_t{i})) { tmp.set(node_idx_t{i}, true); }
      }
    }
    tmp.set(exclude.get_node(), true);

    parameters_t params{};
    dijkstra_t d{};
    d.reset(cost_limit);
    d.add_start(w, label{from, 0U});
    (void)d.run(params, w, r, cost_limit, &tmp, sharing, elevations, direction::kForward);
    return d.get_cost(to);
  }

  node get_node(ways::routing const& r, node_idx_t const n) {
    node result{};
    Profile::resolve_all(r, n, level_t{}, [&](node const& node_ref) { result = node_ref; });
    return result;
  }

  std::unordered_map<node, int> node_levels_;
  std::unordered_map<node, std::vector<shortcut>> shortcuts_;

private:
  std::unordered_map<node, int> node_priorities_;
};

}  // namespace osr
