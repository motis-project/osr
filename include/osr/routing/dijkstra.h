#pragma once

#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <direction Dir>
constexpr direction flip(direction const dir) {
  return Dir == direction::kForward ? dir : opposite(dir);
}

template <typename Profile>
struct dijkstra {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max) {
    pq_.clear();
    pq_.n_buckets(max + 1U);
    cost_.clear();
  }

  void add_start(label const l) {
    if (cost_[l.get_node().get_key()].update(l.get_node(), l.cost(),
                                             node::invalid())) {
      push(l);
    }
  }

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  void push(label const& l) { pq_.push(l); }

  template <direction SearchDir>
  void run(ways const& w, cost_t const max) {
    while (!pq_.empty()) {
      auto l = pq_.pop();
      if (get_cost(l.get_node()) < l.cost()) {
        continue;
      }

      auto const curr = l.get_node();
      Profile::template adjacent<SearchDir>(
          w, curr,
          [&](node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t, std::uint16_t, std::uint16_t) {
            auto const total = l.cost() + cost;
            if (total < max &&
                cost_[neighbor.get_key()].update(
                    neighbor, static_cast<cost_t>(total), curr)) {
              pq_.push(label{neighbor, static_cast<cost_t>(total)});
            }
          });
    }
  }

  void run(ways const& w, cost_t const max, direction const dir) {
    dir == direction::kForward ? run<direction::kForward>(w, max)
                               : run<direction::kBackward>(w, max);
  }

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
};

}  // namespace osr