#pragma once

#include <list>
#include <set>

#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "profiles/car.h"

namespace osr {

template <typename Profile>
struct contraction_hierarchies_m_to_n {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };


  void clear_mp() {
    meet_points_.clear();
    best_cost_.clear();
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
    max_reached_1_.clear();
    max_reached_2_.clear();
    starts_.clear();
    dests_.clear();
  }

  void add_starts_and_destinations(std::vector<label> const& starts, std::vector<label> const& dests) {
    for (auto const& start: starts) {
      for (auto const& dest: dests) {
        meet_points_.insert(std::make_pair(start, dest), node::invalid());
        best_cost_.insert(std::make_pair(start, dest), kInfeasible);
      }
    }
    for (auto const& start: starts) {
      cost1_.insert(start, {});
      cost1_[start][start.get_node().get_key()].update(start, start.get_node(), start.cost(),node::invalid());
      pq_.push(start);
      pred_edges1_.insert(std::make_pair(start, {}));
      starts_.insert(start);
    }
    for (auto const& dest: dests) {
      cost1_.insert(dest, {});
      cost1_[dest][dest.get_node().get_key()].update(dest, dest.get_node(), dest.cost(),node::invalid());
      pq_.push(dest);
      pred_edges2_.insert(std::make_pair(dest, {}));
      dests_.insert(dest);
    }
  }

  template <direction SearchDir>
  cost_t get_cost(node const n, label const l) const {
    if (SearchDir == direction::kForward) {
      auto const it = cost1_[l].find(n.get_key());
      return it != end(cost1_[l]) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = cost2_[l].find(n.get_key());
      return it != end(cost2_[l]) ? it->second.cost(n) : kInfeasible;
    }
  }

  cost_t get_cost_to_mp(label start, label end) const {
    auto candidate = meet_points_.at({start, end});
    assert(candidate != node::invalid());
    auto const f_cost = get_cost<direction::kForward>(candidate, start);
    auto const b_cost = get_cost<direction::kBackward>(candidate, end);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return f_cost + b_cost;
  }

  void run(ways::routing const& r,
           cost_t const max) {

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

      for (auto start : starts_) {
        for (auto dest : dests_) {
          auto const f_cost = get_cost<direction::kForward>(curr, start);
          auto const b_cost = get_cost<direction::kBackward>(curr, dest);
          auto best_cost = best_cost_.at({start, dest});
          if (f_cost != kInfeasible && b_cost != kInfeasible) {
            auto candidate_cost = f_cost + b_cost;
            if (candidate_cost < best_cost) {
              best_cost_.at({start, dest}) = candidate_cost;
              meet_points_.at({start, dest}) = curr;
            }
          }
          if (f_cost < best_cost && f_cost == pq_.current_bucket_) {
            for (auto edge_idx : r.upwards_edges_outgoing_[curr_id]) {
              auto edge = r.contracted_edges_[edge_idx];
              auto const total = f_cost + edge.cost_;
              if (total > max) {
                max_reached_1_.insert(start);
                continue;
              }
              auto neighbor =
                  r.identifier_to_node_[edge.to_].template conv<car::node>();
              if (cost1_[start][neighbor.n_].update(l, neighbor,
                                             static_cast<cost_t>(total), curr)) {
                auto next = label{neighbor, static_cast<cost_t>(total)};
                pq_.push(std::move(next));
                pred_edges1_[start][edge.to_] = edge_idx;
              }
            }
          }
          if (b_cost < best_cost && b_cost == pq_.current_bucket_) {
            for (auto edge_idx : r.downwards_edges_incoming_[curr_id]) {
              auto edge = r.contracted_edges_[edge_idx];
              auto const total = b_cost + edge.cost_;
              if (total > max) {
                max_reached_2_.insert(dest);
                continue;
              }
              auto neighbor =
                  r.identifier_to_node_[edge.from_].template conv<car::node>();
              if (cost2_[dest][neighbor.n_].update(l, neighbor,
                                             static_cast<cost_t>(total), curr)) {
                auto next = label{neighbor, static_cast<cost_t>(total)};
                pq_.push(std::move(next));
                pred_edges2_[dest][edge.from_] = edge_idx;
              }
            }
          }
        }
      }
    }
  }

  dial_ch<label, get_bucket> pq_{get_bucket{}};
  std::map<std::pair<label, label>, node> meet_points_;
  std::map<std::pair<label, label>, cost_t> best_cost_;
  std::map<label, ankerl::unordered_dense::map<key, entry, hash>> cost1_, cost2_;
  std::map<label, std::map<ways::routing::node_identifier, ways::routing::edge_idx_t>>
      pred_edges1_;
  std::map<label, std::map<ways::routing::node_identifier, ways::routing::edge_idx_t>>
      pred_edges2_;
  std::set<label> starts_, dests_;
  std::set<label> max_reached_1_;
  std::set<label> max_reached_2_;
};

} // namespace osr
