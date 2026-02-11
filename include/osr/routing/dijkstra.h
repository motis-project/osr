#pragma once

#include <cstdint>
#include <algorithm>
#include <vector>

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P, bool EarlyTermination = false>
struct dijkstra {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;

  static constexpr auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max) {
    pq_.clear();
    pq_.n_buckets(max + 1U);
    cost_.clear();
    max_reached_ = false;
    if constexpr (EarlyTermination) {
      destinations_.clear();
      remaining_destinations_ = 0U;
      early_termination_max_cost_ = kInfeasible;
      terminated_early_max_cost_ = false;
    }
  }

  void add_start(ways const& w, label const l) {
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_.n_buckets(),
                  "dijkstra::add_start: label cost exceeds max: {} >= {}",
                  l.cost(), pq_.n_buckets());
      pq_.push(l);
    }
  }

  void add_destination(node const n) {
    if constexpr (EarlyTermination) {
      auto it = std::lower_bound(begin(destinations_), end(destinations_), n);
      if (it == end(destinations_) || *it != n) {
        destinations_.insert(it, n);
        ++remaining_destinations_;
      }
    }
  }

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    while (!pq_.empty()) {
      auto l = pq_.pop();

      if (get_cost(l.get_node()) < l.cost()) {
        continue;
      }

      if constexpr (EarlyTermination) {
        if (std::find(begin(destinations_), end(destinations_), l.get_node()) !=
            end(destinations_)) {
          --remaining_destinations_;
          auto const curr_cost = get_cost(l.get_node());
          early_termination_max_cost_ = std::min(
              early_termination_max_cost_,
              static_cast<cost_t>(std::min(
                  {static_cast<std::uint64_t>(curr_cost) * 2,
                   static_cast<std::uint64_t>(
                       curr_cost + P::upper_bound_heuristic(params, 10000U)),
                   static_cast<std::uint64_t>(kInfeasible - 1U)})));
          if (remaining_destinations_ == 0U) {
            break;
          }
        }
        if (l.cost() > early_termination_max_cost_) {
          terminated_early_max_cost_ = true;
          break;
        }
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      auto const curr = l.get_node();
      P::template adjacent<SearchDir, WithBlocked>(
          params, r, curr, blocked, sharing, elevations,
          [&](node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t, std::uint16_t,
              elevation_storage::elevation, bool const track) {
            if constexpr (kDebug) {
              std::cout << "  NEIGHBOR ";
              neighbor.print(std::cout, w);
            }

            auto const total = static_cast<std::uint64_t>(l.cost()) + cost;
            if (total >= max) {
              max_reached_ = true;
              return;
            }
            if (cost_[neighbor.get_key()].update(
                    l, neighbor, static_cast<cost_t>(total), curr)) {
              auto next = label{neighbor, static_cast<cost_t>(total)};
              next.track(l, r, way, neighbor.get_node(), track);
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
    return !max_reached_;
  }

  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      return dir == direction::kForward
                 ? run<direction::kForward, false>(params, w, r, max, blocked,
                                                   sharing, elevations)
                 : run<direction::kBackward, false>(params, w, r, max, blocked,
                                                    sharing, elevations);
    } else {
      return dir == direction::kForward
                 ? run<direction::kForward, true>(params, w, r, max, blocked,
                                                  sharing, elevations)
                 : run<direction::kBackward, true>(params, w, r, max, blocked,
                                                   sharing, elevations);
    }
  }

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
  bool max_reached_{};

  // for early termination
  std::vector<node> destinations_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  bool terminated_early_max_cost_{false};
};

}  // namespace osr
