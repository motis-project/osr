#pragma once

#include <limits>
#include <optional>

#include "utl/verify.h"

#include "geo/constants.h"
#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/entry_storage_arena.h"
#include "osr/routing/profile.h"
#include "osr/routing/search_params.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <Profile P>
struct bidirectional {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;
  using params_t = search_params<typename P::parameters>;

  constexpr static auto const kDebug = false;
  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;
  constexpr static auto const kLongestNodeDistance = cost_t{1800};

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
    best_cost_ = kInfeasible;
    best_transition_ = {};
  }

  void reset(params_t const& p) {
    params_ = p;
    auto const max = params_.max_;
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    arena_.reset();
    clear_mp();
    start_pos_ = params_.start_loc_.pos_;
    end_pos_ = params_.end_loc_.pos_;
    distance_lon_degrees_ = geo::approx_distance_lng_degrees(
        std::abs(start_pos_.lat()) > std::abs(end_pos_.lat()) ? start_pos_
                                                              : end_pos_);
    auto const diameter = P::lower_bound_heuristic(
        params_.profile_, distapprox(start_pos_, end_pos_));
    search_bounds_valid_ =
        diameter < max && max + std::max(diameter, kLongestNodeDistance * 2.0) <
                              std::numeric_limits<cost_t>::max();
    auto const max_edge_radius =
        static_cast<cost_t>((static_cast<std::uint64_t>(max) + 1U) / 2U);
    radius_ = search_bounds_valid_
                  ? std::max({static_cast<cost_t>(diameter * 0.5),
                              kLongestNodeDistance, max_edge_radius})
                  : max;
    max_reached_1_ = false;
    max_reached_2_ = false;
    draining_ = false;
    drain_key_1_ = kInfeasible;
    drain_key_2_ = kInfeasible;
  }

  void add(label l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           duration_t const duration) {
    auto const& w = params_.w();
    radius_ = std::max(radius_, l.cost());
    auto const heur =
        heuristic(params_.profile_, w, l.n_, dir, params_.sharing());
    if (l.cost() + heur < d.n_buckets() - 1U &&
        cost_map[l.get_node().get_key()].update(
            l, l.get_node(), {.cost_ = l.cost(), .duration_ = duration},
            node::invalid(), *w.r_, arena_)) {
      l.cost_ = static_cast<cost_t>(l.cost() + heur);
      d.push(std::move(l));
    }
  }

  void add_start(label const l) { add_start(l, duration_from_cost(l.cost())); }

  void add_start(label const l, duration_t const duration) {
    if (kDebug) {
      l.get_node().print(std::cout, params_.w());
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    add(l, direction::kForward, cost1_, pq1_, duration);
  }

  void add_end(label const l) { add_end(l, duration_from_cost(l.cost())); }

  void add_end(label const l, duration_t const duration) {
    if (kDebug) {
      l.get_node().print(std::cout, params_.w());
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    add(l, direction::kBackward, cost2_, pq2_, duration);
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

  double distapprox(geo::latlng const& p1, geo::latlng const& p2) const {
    auto const y = std::abs(p1.lat() - p2.lat()) * kDistanceLatDegrees;
    auto const xdiff = std::abs(p1.lng() - p2.lng());
    auto const x =
        (xdiff > 180.0 ? (360 - xdiff) : xdiff) * distance_lon_degrees_;
    return std::max(std::max(y, x), (y + x) / 1.42);
  }

  double heuristic(P::parameters const& params,
                   ways const& w,
                   node_idx_t idx,
                   direction const dir,
                   sharing_data const* sharing) const {
    auto const get_node_pos = [&](node_idx_t const n) -> geo::latlng {
      if (n == node_idx_t::invalid()) {
        return {};
      } else if (w.is_additional_node(n)) {
        return sharing->get_additional_node_coordinates(n);
      } else {
        return w.get_node_pos(n).as_latlng();
      }
    };
    auto const p = get_node_pos(idx);
    auto const dist = distapprox(p, end_pos_);
    auto const other_dist = distapprox(p, start_pos_);
    return 0.5 *
           (P::lower_bound_heuristic(params, dist) -
            P::lower_bound_heuristic(params, other_dist)) *
           (dir == direction::kForward ? 1 : -1);
  }

  duration_t get_duration_to_mp(node const n1, node const n2) const {
    auto const f = cost1_.find(n1.get_key());
    auto const b = cost2_.find(n2.get_key());
    if (f == end(cost1_) || b == end(cost2_)) {
      return kMaxDuration;
    }
    return clamp_add_duration(f->second.duration(n1), b->second.duration(n2));
  }

  duration_t best_duration() const {
    return best_cost_ == kInfeasible
               ? kMaxDuration
               : clamp_add_duration(
                     get_duration_to_mp(meet_point_1_, meet_point_2_),
                     best_transition_.duration_);
  }

  static cost_t next_key(dial<label, get_bucket> const& pq) {
    return static_cast<cost_t>(pq.get_next_bucket());
  }

  static bool bucket_drained(dial<label, get_bucket> const& pq,
                             cost_t const key) {
    return pq.empty() || next_key(pq) > key;
  }

  template <direction SearchDir, bool WithBlocked, direction PathDir>
  bool run_single(dial<label, get_bucket>& pq, cost_map& costs) {
    auto const& params = params_.profile_;
    auto const& w = params_.w();
    auto const& r = params_.r();
    auto const max = params_.max_;
    auto const* blocked = params_.blocked_;
    auto const* sharing = params_.sharing();
    auto const* elevations = params_.elevations_;

    auto const is_fwd = PathDir == direction::kForward;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);
    auto const curr_duration = costs.at(curr.get_key()).duration(curr);
    if (static_cast<std::int64_t>(curr_cost) <
        static_cast<std::int64_t>(l.cost()) -
            static_cast<std::int64_t>(
                heuristic(params, w, l.n_, PathDir, sharing))) {
      return true;
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    P::template adjacent<SearchDir, WithBlocked>(
        params, r, w.timezones_, curr, curr_duration, std::nullopt, blocked,
        sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost,
            duration_t const duration, distance_t, way_idx_t const way,
            std::uint16_t, std::uint16_t, elevation_storage::elevation const,
            bool const track) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total =
              clamp_cost(static_cast<std::uint64_t>(curr_cost) + cost);
          auto const next_cd = cost_and_duration{
              .cost_ = total,
              .duration_ = clamp_add_duration(curr_duration, duration)};
          auto const heur =
              clamp_cost(static_cast<std::int64_t>(total) +
                         static_cast<std::int64_t>(heuristic(
                             params, w, neighbor.n_, PathDir, sharing)));
          if (total >= max) {
            if (is_fwd) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }
          auto const updated = [&]() {
            if (heur >= max) {
              return false;
            }
            auto next = label{neighbor, static_cast<cost_t>(heur)};
            next.track(l, r, way, neighbor.get_node(), track);
            if (!costs[neighbor.get_key()].update(next, neighbor, next_cd, curr,
                                                  r, arena_)) {
              return false;
            }
            pq.push(std::move(next));
            return true;
          }();
          if (updated) {

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        });

    auto best_duration_now = kMaxDuration;
    auto best_duration_known = false;
    auto const tie_break_duration = [&]() {
      if (!best_duration_known) {
        best_duration_now = best_duration();
        best_duration_known = true;
      }
      return best_duration_now;
    };

    auto const evaluate_meetpoint =
        [&](cost_t cost, cost_t other_cost, node meetpoint1, node meetpoint2,
            cost_and_duration const transition = cost_and_duration{}) {
          if constexpr (kDebug) {
            std::cout << "  potential MEETPOINT found by start ";
            meetpoint1.print(std::cout, w);
          }
          auto const tentative = static_cast<std::uint64_t>(cost) +
                                 static_cast<std::uint64_t>(other_cost) +
                                 static_cast<std::uint64_t>(transition.cost_);
          if (tentative > static_cast<std::uint64_t>(best_cost_)) {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
            return;
          }
          auto const tentative_duration = clamp_add_duration(
              get_duration_to_mp(meetpoint1, meetpoint2), transition.duration_);
          if (tentative < static_cast<std::uint64_t>(best_cost_) ||
              tentative_duration < tie_break_duration()) {
            meet_point_1_ = meetpoint1;
            meet_point_2_ = meetpoint2;
            best_cost_ = clamp_cost(tentative);
            best_transition_ = transition;
            best_duration_now = tentative_duration;
            best_duration_known = true;

            if constexpr (kDebug) {
              std::cout << " with cost " << best_cost_ << " -> ACCEPTED\n";
            }
          } else if constexpr (kDebug) {
            std::cout << " -> DOMINATED\n";
          }
        };

    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map = is_fwd ? &cost2_ : &cost1_;
      auto const opposite_candidate = opposite_cost_map->find(curr.get_key());
      if (opposite_candidate != end(*opposite_cost_map)) {
        if constexpr (bidirectional_meet_policy<P>::kEnumerateStates) {
          P::resolve_all(r, curr.get_node(), [&](node const other) {
            auto const other_cost = opposite_candidate->second.cost(other);
            if (other_cost == kInfeasible) {
              return;
            }
            // Which of the two states arrives at and which one leaves the
            // meet point depends on the edge direction this search expands,
            // not on which of the two queues it belongs to.
            constexpr auto const kCurrArrives =
                SearchDir == direction::kForward;
            auto const transition = P::bidirectional_meet_cost(
                params, r, kCurrArrives ? curr : other,
                kCurrArrives ? other : curr, sharing);
            if (!transition.feasible()) {
              return;
            }
            evaluate_meetpoint(curr_cost, other_cost, is_fwd ? curr : other,
                               is_fwd ? other : curr, transition);
          });
        } else {
          auto const other_cost = opposite_candidate->second.cost(curr);
          if (other_cost != kInfeasible) {
            evaluate_meetpoint(curr_cost, other_cost, curr, curr);
          }
        }
      }
    };

    handle_end_of_way_meetpoint();

    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? get_cost<direction::kForward>(meet_point_1_)
                       : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? get_cost<direction::kBackward>(meet_point_2_)
                       : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (static_cast<std::uint64_t>(top_f) + top_r >
          static_cast<std::uint64_t>(best_cost_) +
              static_cast<std::uint64_t>(radius_)) {
        // Re-evaluate equal-cost meeting states with shorter durations before
        // stopping: their labels may still be in either queue's current bucket.
        if (!draining_) {
          draining_ = true;
          drain_key_1_ = pq1_.empty() ? kInfeasible : next_key(pq1_);
          drain_key_2_ = pq2_.empty() ? kInfeasible : next_key(pq2_);
        }
        if (bucket_drained(pq1_, drain_key_1_) &&
            bucket_drained(pq2_, drain_key_2_)) {
          if (kDebug) {
            std::cout << "stopping criterion met " << top_f << " " << top_r
                      << " " << best_cost_ << " " << radius_ << std::endl;
          }
          return false;
        }
      } else {
        draining_ = false;
      }
    }
    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run() {
    if (!search_bounds_valid_) {
      return false;
    }
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() &&
          !run_single<SearchDir, WithBlocked, direction::kForward>(pq1_,
                                                                   cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked, direction::kBackward>(
              pq2_, cost2_)) {
        break;
      }
    }
    if (best_cost_ != kInfeasible && best_cost_ > params_.max_) {
      clear_mp();
      return false;
    }
    return !max_reached_1_ || !max_reached_2_;
  }

  bool run() {
    if (params_.blocked_ == nullptr) {
      return params_.dir_ == direction::kForward
                 ? run<direction::kForward, false>()
                 : run<direction::kBackward, false>();
    } else {
      return params_.dir_ == direction::kForward
                 ? run<direction::kForward, true>()
                 : run<direction::kBackward, true>();
    }
  }

  params_t params_;

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  cost_and_duration best_transition_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  entry_storage_arena arena_;
  cost_t radius_;
  geo::latlng start_pos_;
  geo::latlng end_pos_;
  double distance_lon_degrees_;
  bool search_bounds_valid_{};
  bool max_reached_1_;
  bool max_reached_2_;
  bool draining_{false};
  cost_t drain_key_1_{kInfeasible};
  cost_t drain_key_2_{kInfeasible};
};

}  // namespace osr
