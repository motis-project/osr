#pragma once

#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

constexpr auto const kDebug = false;

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

  void add_start(ways const& w, label const l) {
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      pq_.push(l);
    }
  }

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing) {
    while (!pq_.empty()) {
      auto l = pq_.pop();
      if (get_cost(l.get_node()) < l.cost()) {
        continue;
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      auto const curr = l.get_node();
      Profile::template adjacent<SearchDir, WithBlocked>(
          r, curr, blocked, sharing,
          [&](node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t, std::uint16_t) {
            if constexpr (kDebug) {
              std::cout << "  NEIGHBOR ";
              neighbor.print(std::cout, w);
            }

            auto const total = l.cost() + cost;
            if (total < max &&
                cost_[neighbor.get_key()].update(
                    l, neighbor, static_cast<cost_t>(total), curr)) {
              auto next = label{neighbor, static_cast<cost_t>(total)};
              next.track(l, r, way, neighbor.get_node());
              pq_.push(std::move(next));

              if constexpr (kDebug) {
                std::cout << " -> PUSH\n";
              }
            } else {
              if constexpr (kDebug) {
                std::cout << " -> DOMINATED\n";
              }
            }
          });
    }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward
          ? run<direction::kForward, false>(w, r, max, blocked, sharing)
          : run<direction::kBackward, false>(w, r, max, blocked, sharing);
    } else {
      dir == direction::kForward
          ? run<direction::kForward, true>(w, r, max, blocked, sharing)
          : run<direction::kBackward, true>(w, r, max, blocked, sharing);
    }
  }

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
};

}  // namespace osr
