#pragma once

#include <cstdint>
#include <algorithm>
#include <vector>

#include "fmt/core.h"

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
struct dijkstra_bidir {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using settled_set = ankerl::unordered_dense::set<key, hash>;

  static constexpr auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max) {
    pqForward_.clear();
    pqForward_.n_buckets(max + 1U);
    pqBackward_.clear();
    pqBackward_.n_buckets(max + 1U);
    costForward_.clear();
    costBackward_.clear();
    settledForward_.clear();
    settledBackward_.clear();
    mu_ = kInfeasible;
    meet_ = node::invalid();
    max_reached_ = false;
  }

  void add_start(ways const& w, label const l) {
    if (costForward_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                    node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pqForward_.n_buckets(),
                  "dijkstra_bidir::add_start: label cost exceeds max: {} >= {}",
                  l.cost(), pqForward_.n_buckets());
      pqForward_.push(l);
    }
  }

  void add_destination(ways const& w, label const l) {
    // std::cout << "DEST ";
    // n.get_node().print(std::cout, w);
    // std::cout << "\n";
    if (costBackward_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                     node::invalid())) {
      if constexpr (kDebug) {
        std::cout << "DESTINATION ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(
          l.cost() < pqBackward_.n_buckets(),
          "dijkstra_bidir::add_destination: label cost exceeds max: {} >= {}",
          l.cost(), pqBackward_.n_buckets());
      pqBackward_.push(l);
    }
  }

  // cost_t get_cost(node const n) const {
  //   auto const it = cost_.find(n.get_key());
  //   return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  // }

  template <direction Dir>
  cost_t get_cost(node const n) const {
    if constexpr (Dir == direction::kForward) {
      auto const it = costForward_.find(n.get_key());
      return it != end(costForward_) ? it->second.cost(n) : kInfeasible;
    } else {
      auto const it = costBackward_.find(n.get_key());
      return it != end(costBackward_) ? it->second.cost(n) : kInfeasible;
    }
  }

  void update_mu(node const n, cost_t const f, cost_t const b) {
    if (f != kInfeasible && b != kInfeasible) {
      auto const candidate = clamp_cost(static_cast<std::uint64_t>(f) +
                                        static_cast<std::uint64_t>(b));
      if (candidate < mu_) {
        mu_ = candidate;
        meet_ = n;
      }
    }
  }

  template <direction Dir>
  bool settle(node const n) {
    if constexpr (Dir == direction::kForward) {
      if (!settledForward_.insert(n.get_key()).second) {
        return false;
      }
    } else {
      if (!settledBackward_.insert(n.get_key()).second) {
        return false;
      }
    }
    return true;
  }

  template <direction Dir>
  bool is_stale(label const& l) const {
    return get_cost<Dir>(l.get_node()) < l.cost();
  }

  template <direction Dir>
  void discard_stale_top(dial<label, get_bucket>& pq) {
    while (!pq.empty()) {
      auto const& candidate = pq.buckets_[pq.get_next_bucket()].back();
      if (!is_stale<Dir>(candidate)) {
        return;
      }
      pq.pop();
    }
  }

  bool done() {
    if (mu_ == kInfeasible) {
      return false;
    }

    if (pqForward_.empty() || pqBackward_.empty()) {
      return false;
    }

    return static_cast<std::uint64_t>(pqForward_.get_next_bucket()) +
               static_cast<std::uint64_t>(pqBackward_.get_next_bucket()) >=
           static_cast<std::uint64_t>(mu_);
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    while (!pqForward_.empty() || !pqBackward_.empty()) {
      discard_stale_top<direction::kForward>(pqForward_);
      discard_stale_top<direction::kBackward>(pqBackward_);
      if (done()) {
        fmt::println("dijkstra_bidir mu: {}", mu_);
        break;
      }
      if (pqForward_.empty() && pqBackward_.empty()) {
        break;
      }

      auto const forward =
          pqBackward_.empty() ||
          (!pqForward_.empty() &&
           pqForward_.get_next_bucket() <= pqBackward_.get_next_bucket());

      auto l = forward ? pqForward_.pop() : pqBackward_.pop();
      auto const curr = l.get_node();

      if (forward) {
        if (is_stale<direction::kForward>(l)) {
          continue;
        }
        if (!settle<direction::kForward>(curr)) {
          continue;
        }
      } else {
        if (is_stale<direction::kBackward>(l)) {
          continue;
        }
        if (!settle<direction::kBackward>(curr)) {
          continue;
        }
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      auto relax_neighbor = [&](node const neighbor, std::uint32_t const cost,
                                distance_t, way_idx_t const way, std::uint16_t,
                                std::uint16_t, elevation_storage::elevation,
                                bool const track) {
        if constexpr (kDebug) {
          std::cout << "  NEIGHBOR ";
          neighbor.print(std::cout, w);
        }

        auto const total = static_cast<std::uint64_t>(l.cost()) + cost;
        if (total >= max) {
          max_reached_ = true;
          return;
        }
        if (forward) {
          auto const total_cost = static_cast<cost_t>(total);
          auto const improved =
              costForward_[neighbor.get_key()].update(l, neighbor, total_cost,
                                                      curr);
          update_mu(neighbor, get_cost<direction::kForward>(neighbor),
                    get_cost<direction::kBackward>(neighbor));
          if (improved) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pqForward_.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        } else {
          auto const total_cost = static_cast<cost_t>(total);
          auto const improved =
              costBackward_[neighbor.get_key()].update(l, neighbor, total_cost,
                                                       curr);
          update_mu(neighbor, get_cost<direction::kForward>(neighbor),
                    get_cost<direction::kBackward>(neighbor));
          if (improved) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node(), track);
            pqBackward_.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        }
      };

      if (forward) {
        P::template adjacent<SearchDir, WithBlocked>(
            params, r, curr, blocked, sharing, elevations, relax_neighbor);
      } else {
        P::template adjacent<opposite(SearchDir), WithBlocked>(
            params, r, curr, blocked, sharing, elevations, relax_neighbor);
      }
    }
    return !max_reached_ && mu_ == kInfeasible;
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

  dial<label, get_bucket> pqForward_{get_bucket{}};
  dial<label, get_bucket> pqBackward_{get_bucket{}};

  ankerl::unordered_dense::map<key, entry, hash> costForward_;
  ankerl::unordered_dense::map<key, entry, hash> costBackward_;
  bool max_reached_{};

  settled_set settledForward_;
  settled_set settledBackward_;
  cost_t mu_{kInfeasible};
  node meet_{node::invalid()};

  // for early termination
  std::vector<node> destinations_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  bool terminated_early_max_cost_{false};
};

}  // namespace osr
