#pragma once

#include <list>
#include <map>
#include <unordered_set>
#include <algorithm>
#include <iterator>
#include <numeric>

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "osr/routing/profile.h"
#include "osr/routing/parameters.h"
#include "profiles/car.h"

namespace osr {

struct sharing_data;
struct ch_config {
  static bool constexpr kUseHeuristic = false;
  static float constexpr kHeuristicThresholdFactor = 0.8f;
  static bool constexpr kUseNodeImportance = true;
  static bool constexpr kRemoveCycles = false;
};

template <Profile P>
struct contraction_hierarchies {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;
  bool static constexpr kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_ = meet_point_.invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max) {
    pq_.clear();
    pq_.n_buckets(max + 1U);
    assert(max + 1U != 0);  // catch overflows
    cost1_.clear();
    cost2_.clear();
    pred_edges1_.clear();
    pred_edges2_.clear();
    clear_mp();
    max_reached_1_ = false;
    max_reached_2_ = false;
  }

  void add_start(label const l) {
    if (cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      pq_.push(l);
    }
  }

  void add_end(label const l) {
    if (cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      pq_.push(l);
    }
  }

  template <direction SearchDir>
  ankerl::unordered_dense::map<key, entry, hash>& get_cost_map() {
    return SearchDir == direction::kForward ? cost1_ : cost2_;
  }

  template <direction SearchDir>
  cost_t get_cost(node const n) const {
    if (SearchDir == direction::kForward) {
      auto const it = cost1_.find(n.get_key());
      return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_.find(n.get_key());
      return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t get_cost_to_mp() const {
    assert(meet_point_ != node::invalid());
    auto const f_cost = get_cost<direction::kForward>(meet_point_);
    auto const b_cost = get_cost<direction::kBackward>(meet_point_);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return f_cost + b_cost;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run([[maybe_unused]] ways const& w,
           [[maybe_unused]] ways::routing const& r,
           cost_t const max,
           [[maybe_unused]] bitvec<node_idx_t> const* blocked,
           [[maybe_unused]] sharing_data const* sharing,
           [[maybe_unused]] elevation_storage const* elevations) {
    while (!pq_.empty()) {
      auto const l = pq_.pop();
      auto const curr = l.get_node();
      auto to_id = [&](node const& n) -> std::int64_t {
        auto start = r.node_idx_to_identifier_[n.get_node()];
        for (auto candidate = start; candidate < start + entry::kN;
             ++candidate) {
          if (n == r.identifier_to_node_[candidate].template conv<node>())
            return candidate;
        }
        assert(false);
        return 0;
      };
      auto const curr_id = to_id(curr);
      auto const f_cost = get_cost<SearchDir>(curr);
      auto const b_cost = get_cost<opposite(SearchDir)>(curr);
      if (f_cost != kInfeasible && b_cost != kInfeasible) {
        auto candidate_cost = f_cost + b_cost;
        if (candidate_cost < best_cost_) {
          best_cost_ = static_cast<cost_t>(candidate_cost);
          meet_point_ = curr;
        }
      }
      if (f_cost < best_cost_ && f_cost == pq_.current_bucket_) {
        for (auto edge_idx : r.upwards_edges_outgoing_[curr_id]) {
          auto edge = r.contracted_edges_[edge_idx];
          auto const total = f_cost + edge.cost_;
          if (total > max) {
            max_reached_1_ = true;
            continue;
          }
          auto neighbor =
              r.identifier_to_node_[edge.to_].template conv<car::node>();
          if (get_cost_map<SearchDir>()[neighbor.n_].update(l, neighbor,
                                         static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            pq_.push(std::move(next));
            pred_edges1_[edge.to_] = edge_idx;
          }
        }
      }
      if (b_cost < best_cost_ && b_cost == pq_.current_bucket_) {
        for (auto edge_idx : r.downwards_edges_incoming_[curr_id]) {
          auto edge = r.contracted_edges_[edge_idx];
          auto const total = b_cost + edge.cost_;
          if (total > max) {
            max_reached_2_ = true;
            continue;
          }
          auto neighbor =
              r.identifier_to_node_[edge.from_].template conv<car::node>();
          if (get_cost_map<opposite(SearchDir)>()[neighbor.n_].update(l, neighbor,
                                         static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            pq_.push(std::move(next));
            pred_edges2_[edge.from_] = edge_idx;
          }
        }
      }
    }
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }

    return !max_reached_1_ || !max_reached_2_;
  }

  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(w, r, max, blocked, sharing,
                                                   elevations)
                 : run<direction::kBackward, false>(w, r, max, blocked, sharing,
                                                    elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(w, r, max, blocked, sharing,
                                                  elevations)
                 : run<direction::kBackward, true>(w, r, max, blocked, sharing,
                                                   elevations);
    }
  }
  [[nodiscard]] std::list<ways::routing::edge_idx_t> uncontract_edges(
      ways const& w) const {
    std::list<ways::routing::edge_idx_t> result;
    assert(best_cost_ != kInfeasible);
    bool found = false;
    ways::routing::node_identifier meet_point_ID =
        w.r_->node_idx_to_identifier_[meet_point_.get_node()];
    while (!found && w.r_->identifier_to_node_[meet_point_ID].n_ ==
                         meet_point_.get_node()) {
      if (meet_point_ ==
          w.r_->identifier_to_node_[meet_point_ID].conv<node>()) {
        found = true;
      } else {
        ++meet_point_ID;
      }
    }
    assert(found);
    auto current_ID = meet_point_ID;
    while (true) {
      auto pred = pred_edges1_.find(current_ID);
      if (pred == pred_edges1_.end()) {
        break;
      }
      auto edge_idx = pred->second;
      result.push_front(pred->second);
      current_ID = w.r_->contracted_edges_[edge_idx].from_;
    }
    current_ID = meet_point_ID;
    while (true) {
      auto pred = pred_edges2_.find(current_ID);
      if (pred == pred_edges2_.end()) {
        break;
      }
      auto const edge_idx = pred->second;
      result.push_back(pred->second);
      current_ID = w.r_->contracted_edges_[edge_idx].to_;
    }

    // uncontract all edges
    for (auto iterator = result.begin(); iterator != result.end();) {
      auto const edge_idx = *iterator;
      auto edge = w.r_->contracted_edges_[edge_idx];
      if (!edge.is_contracted()) {
        ++iterator;
        continue;
      }
      *iterator = edge.contracted_child2_;
      result.insert(iterator, edge.contracted_child1_);
      --iterator;
    }
    return result;
  }

  dial_ch<label, get_bucket> pq_{get_bucket{}};
  node meet_point_;
  cost_t best_cost_{kInfeasible};
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  std::map<ways::routing::node_identifier, ways::routing::edge_idx_t>
      pred_edges1_;
  std::map<ways::routing::node_identifier, ways::routing::edge_idx_t>
      pred_edges2_;
  bool max_reached_1_{false};
  bool max_reached_2_{false};
};

using dial_element_t = std::pair<ways::routing::node_identifier, cost_t>;
struct dial_element_to_bucket {
  cost_t operator()(dial_element_t const& el) const { return el.second; }
};
inline dial<dial_element_t, dial_element_to_bucket> ch_dial_{dial_element_to_bucket{}};

inline void dijkstra_ch(
  ways::routing::node_identifier start,
  std::vector<ways::routing::node_identifier> const& destinations,
  ways::routing& r_,
  std::vector<std::vector<ways::routing::edge_idx_t>> const& outgoing_edges,
  [[maybe_unused]] std::vector<ways::routing::node_identifier> const& node_to_level,
  cost_t const cutoff,
  std::uint64_t search_ID,
  std::vector<std::pair<cost_t, std::uint64_t>>& index_handler,
  [[maybe_unused]] ways::routing::node_identifier min_level) {

  ch_dial_.clear();
  ch_dial_.n_buckets(cutoff + 1);
  ch_dial_.push(std::make_pair(start, 0U));
  index_handler[start] = std::make_pair(0U, search_ID);

  std::size_t destinations_finished = 0;

  while (!ch_dial_.empty() && destinations_finished < destinations.size()) {
    auto [fst, snd] = ch_dial_.pop();
    ways::routing::node_identifier const node_ID = fst;
    cost_t const current_cost = snd;
    if (auto [cost, seen_ID] = index_handler[node_ID];
        current_cost > cost && search_ID == seen_ID) { // check if the element was already seen
      continue;
    }

    if (std::ranges::find(destinations, node_ID) != destinations.end()) {
      ++destinations_finished;
    }

    for (auto const edge_idx : outgoing_edges[node_ID]) {
      auto edge = r_.contracted_edges_[edge_idx];
      assert(node_to_level[edge.to_] > min_level);
      auto new_cost = current_cost + edge.cost_;
      auto [prev_cost, prev_search_ID] = index_handler[edge.to_];
      if (prev_search_ID == search_ID && prev_cost <= new_cost) continue;
      if (new_cost > cutoff) continue;
      ch_dial_.push(std::make_pair(edge.to_, new_cost));
      index_handler[edge.to_] = std::make_pair(new_cost, search_ID);
    }
  }

}

inline void local_1_hop_ch(
  ways::routing::node_identifier start,
  std::vector<std::vector<ways::routing::edge_idx_t>>& outgoing_edges,
  std::vector<std::vector<ways::routing::edge_idx_t>>& incoming_edges,
  ways::routing& r_){

  auto const& out_edges = outgoing_edges[start];
  std::map<ways::routing::node_identifier, std::pair<ways::routing::edge_idx_t, cost_t>> neighbor_to_edge_idx_cost;
  std::unordered_set<ways::routing::node_identifier> out_neighbors;

  for (auto const edge_idx : out_edges) {
    auto const& edge = r_.contracted_edges_[edge_idx];
    out_neighbors.insert(edge.to_);
    neighbor_to_edge_idx_cost[edge.to_] = std::make_pair(edge_idx, edge.cost_);
  }

  auto const& in_edges = incoming_edges[start];
  for (auto const in_edge_idx : in_edges) {
    auto remaining_neighbors = out_neighbors;

    auto& in_edge = r_.contracted_edges_[in_edge_idx];
    auto const in_cost = in_edge.cost_;
    auto const in_neighbor_ID = in_edge.from_;
    auto const& edges = outgoing_edges[in_neighbor_ID];

    for (auto const edge_idx : edges) {
      auto& edge = r_.contracted_edges_[edge_idx];
      if (remaining_neighbors.erase(edge.to_) == 1) {
        auto [seen_through, seen_with_cost] = neighbor_to_edge_idx_cost[edge.to_];
        if (seen_with_cost < kInfeasible - in_cost && edge.cost_ > seen_with_cost + in_cost) {
          r_.contracted_edges_[edge_idx].cost_ = seen_with_cost + in_cost;
          r_.contracted_edges_[edge_idx].contracted_child1_ = in_edge_idx;
          r_.contracted_edges_[edge_idx].contracted_child2_ = seen_through;
        }
      }
    }
    // introduce shortcuts for all neighbors not seen
    for (auto neighbor : remaining_neighbors) {
      auto [seen_through, seen_with_cost] = neighbor_to_edge_idx_cost[neighbor];
      if (seen_with_cost >= kInfeasible - in_cost) continue;
      if (neighbor == in_neighbor_ID) continue;
        auto new_edge = ways::routing::contracted_edge{
          in_edge_idx,
          seen_through,
          in_neighbor_ID,
          neighbor,
          static_cast<cost_t>(seen_with_cost + in_cost)
        };
        r_.contracted_edges_.emplace_back(new_edge);
        auto new_edge_idx = r_.contracted_edges_.size() - 1;
        outgoing_edges[in_neighbor_ID].push_back(new_edge_idx);
        incoming_edges[neighbor].push_back(new_edge_idx);
    }
  }
}

template <typename P>
[[nodiscard]]
std::pair<std::vector<typename P::node>, std::vector<cost_t>> eliminate_cycles(
      ways const& w, std::list<ways::routing::edge_idx_t> const& list_of_edges) {

  std::vector<cost_t> costs;
  std::vector<typename P::node> profile_nodes;

  auto const start_ID = w.r_->contracted_edges_[list_of_edges.front()].from_;
  auto const start_node = w.r_->identifier_to_node_[start_ID];

  profile_nodes.push_back(start_node.conv<typename P::node>());
  for (auto const edge_idx : list_of_edges) {
    auto edge = w.r_->contracted_edges_[edge_idx];
    profile_nodes.push_back(w.r_->identifier_to_node_[edge.to_].conv<typename P::node>());
    costs.push_back(edge.cost_);
  }

  if constexpr (!ch_config::kRemoveCycles) {
    return std::make_pair(profile_nodes, costs);
  }

  auto params = std::get<car::parameters>(get_parameters(search_profile::kCar));
  for (size_t index = 0; index < profile_nodes.size(); ++index) {
    auto it = std::find_if(profile_nodes.rbegin(), profile_nodes.rend(),
      [&](auto const& node) { return node.n_ == profile_nodes[index].n_; });
    if (*it == profile_nodes[index]) continue;

    if (it == profile_nodes.rbegin()) {
      profile_nodes.resize(index + 1);
      costs.resize(index);
      return std::make_pair(profile_nodes, costs);
    }
    auto dist = std::distance(profile_nodes.begin(), it.base()) - 1;
    cost_t sum_of_costs = static_cast<cost_t>(std::accumulate(costs.begin() + static_cast<long>(index), costs.begin() + dist, 0));
    cost_t sum_of_costs_2 = costs[static_cast<std::size_t>(dist)] + sum_of_costs;

    P::template adjacent<direction::kForward, false>(params, *w.r_,
      profile_nodes[index], nullptr, nullptr, nullptr, [&](car::node const tail,
      std::uint32_t const cost, distance_t, way_idx_t, std::uint16_t,
      std::uint16_t, elevation_storage::elevation, bool) {
        if (tail == *it.base() && cost <= sum_of_costs_2) {
          assert(cost == sum_of_costs_2);
          profile_nodes.erase(profile_nodes.begin()
            + static_cast<typename std::vector<typename P::node>::difference_type>(index) + 1,
            profile_nodes.begin() + dist + 1);
          costs.erase(costs.begin() + static_cast<long>(index), costs.begin() + dist);
          costs[index] = sum_of_costs_2;
        }
        if (tail == *it && cost <= sum_of_costs) {
          assert(cost == sum_of_costs);
          profile_nodes.erase(profile_nodes.begin()
            + static_cast<typename std::vector<typename P::node>::difference_type>(index) + 1,
            profile_nodes.begin() + dist);
          costs.erase(costs.begin() + static_cast<long>(index), costs.begin() + (dist - 1));
          costs[index] = sum_of_costs;
        }
      });
  }
  return std::make_pair(profile_nodes, costs);
}

}  // namespace osr

