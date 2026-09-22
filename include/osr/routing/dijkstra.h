#pragma once

#include <cstdint>

#include <algorithm>
#include <optional>
#include <vector>

#include "utl/verify.h"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/entry_storage_arena.h"
#include "osr/routing/profile.h"
#include "osr/routing/search_params.h"
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
  using params_t = search_params<typename P::parameters>;

  static constexpr auto const kDebug = false;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(params_t const& p) {
    params_ = p;
    pq_.clear();
    pq_.n_buckets(params_.max_ + 1U);
    cost_.clear();
    arena_.reset();
    settle_horizon_ = 0U;
    if constexpr (EarlyTermination) {
      destinations_.clear();
      settled_.clear();
      remaining_destinations_ = 0U;
      early_termination_max_cost_ = kInfeasible;
      all_settled_key_ = kInfeasible;
      terminated_early_max_cost_ = false;
    }
  }

  void add_start(label const l) { add_start(l, duration_from_cost(l.cost())); }

  void add_start(label const l, duration_t const duration) {
    auto const& w = params_.w();
    if (cost_[l.get_node().get_key()].update(
            l, l.get_node(), {.cost_ = l.cost(), .duration_ = duration},
            node::invalid(), *w.r_, arena_)) {
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      utl::verify(l.cost() < pq_.n_buckets(),
                  "dijkstra::add_start: label cost exceeds max: {} >= {}",
                  l.cost(), pq_.n_buckets());
      extend_settle_horizon(l.cost(), duration);
      pq_.push(l);
    }
  }

  void add_destination(node const n) {
    if constexpr (EarlyTermination) {
      auto it = std::lower_bound(begin(destinations_), end(destinations_), n);
      if (it == end(destinations_) || *it != n) {
        settled_.insert(
            begin(settled_) + std::distance(begin(destinations_), it), false);
        destinations_.insert(it, n);
        ++remaining_destinations_;
      }
    }
  }

  bool settle_destination(node const n) {
    auto const it =
        std::lower_bound(begin(destinations_), end(destinations_), n);
    if (it == end(destinations_) || *it != n) {
      return false;
    }
    auto const idx =
        static_cast<std::size_t>(std::distance(begin(destinations_), it));
    if (settled_[idx]) {
      // equal-cost labels with shorter durations can re-enter the queue, so the
      // same destination can be popped more than once and must only be counted
      // once
      return false;
    }
    settled_[idx] = true;
    --remaining_destinations_;
    return true;
  }

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  // Drain the entire bucket: equal-cost labels can still improve duration.
  void extend_settle_horizon(cost_t const cost, duration_t const duration) {
    if (duration <= params_.max_duration_) {
      settle_horizon_ = std::max(settle_horizon_, cost);
    }
  }

  // Resumes a search stopped at its settle horizon until every state with a
  // cost <= `cost` is final. Returns whether the search had to continue.
  bool settle_up_to(cost_t const cost) {
    if (pq_.empty() || pq_.get_next_bucket() > cost) {
      return false;
    }
    settle_horizon_ = std::max(settle_horizon_, cost);
    run();
    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  void run() {
    auto const& params = params_.profile_;
    auto const& w = params_.w();
    auto const& r = params_.r();
    auto const max = params_.max_;
    auto const start_time = params_.start_time_;
    auto const* const blocked = params_.blocked_;
    auto const* const sharing = params_.sharing();
    auto const* const elevations = params_.elevations_;

    while (!pq_.empty() && pq_.get_next_bucket() <= settle_horizon_) {
      auto l = pq_.pop();

      if (get_cost(l.get_node()) < l.cost()) {
        continue;
      }

      if constexpr (EarlyTermination) {
        // Settling a destination fixes its cost, but not yet its duration:
        // The queue can still contain equal-cost entries with shorter
        // durations, and these could reach a destination using 0-cost edges.
        // Therefore, we can only terminate once the current queue bucket is
        // empty.
        if (all_settled_key_ == kInfeasible) {
          if (settle_destination(l.get_node())) {
            auto const curr_cost = get_cost(l.get_node());
            early_termination_max_cost_ = std::min(
                early_termination_max_cost_,
                static_cast<cost_t>(std::min(
                    {static_cast<std::uint64_t>(curr_cost) * 2 +
                         static_cast<std::uint64_t>(
                             P::upper_bound_heuristic(params, 1500U)),
                     static_cast<std::uint64_t>(
                         curr_cost + P::upper_bound_heuristic(params, 10000U)),
                     static_cast<std::uint64_t>(kInfeasible - 1U)})));
            if (remaining_destinations_ == 0U) {
              all_settled_key_ = l.cost();
            }
          }
          if (all_settled_key_ == kInfeasible &&
              l.cost() > early_termination_max_cost_) {
            terminated_early_max_cost_ = true;
            break;
          }
        } else if (l.cost() > all_settled_key_) {
          break;
        }
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      auto const curr = l.get_node();
      auto const curr_duration = cost_.at(curr.get_key()).duration(curr);
      P::template adjacent<SearchDir, WithBlocked>(
          params, r, w.timezones_, curr, curr_duration, start_time, blocked,
          sharing, elevations,
          [&](node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t, way_idx_t const way,
              std::uint16_t, std::uint16_t, elevation_storage::elevation,
              bool const track) {
            if constexpr (kDebug) {
              std::cout << "  NEIGHBOR ";
              neighbor.print(std::cout, w);
            }

            auto const total = static_cast<std::uint64_t>(l.cost()) + cost;
            if (total >= max) {
              return;
            }
            auto const next_cd = cost_and_duration{
                .cost_ = static_cast<cost_t>(total),
                .duration_ = clamp_add_duration(curr_duration, duration)};
            auto next = label{neighbor, next_cd.cost_};
            next.track(l, r, way, neighbor.get_node(), track);
            if (cost_[neighbor.get_key()].update(next, neighbor, next_cd, curr,
                                                 r, arena_)) {
              extend_settle_horizon(next_cd.cost_, next_cd.duration_);
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

  void run() {
    if (params_.blocked_ == nullptr) {
      if (params_.dir_ == direction::kForward) {
        run<direction::kForward, false>();
      } else {
        run<direction::kBackward, false>();
      }
    } else {
      if (params_.dir_ == direction::kForward) {
        run<direction::kForward, true>();
      } else {
        run<direction::kBackward, true>();
      }
    }
  }

  params_t params_;

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
  entry_storage_arena arena_;

  // Highest cost reached within the duration limit, or requested by
  // settle_up_to(). Over-limit labels still expand to dominate costlier routes,
  // but cannot raise the horizon because their successors are also over limit.
  cost_t settle_horizon_{0U};

  // for early termination
  std::vector<node> destinations_;
  std::vector<bool> settled_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  cost_t all_settled_key_{kInfeasible};
  bool terminated_early_max_cost_{false};
};

}  // namespace osr
