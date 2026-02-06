#pragma once

#include <limits>

#include "utl/verify.h"

#include "geo/constants.h"
#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
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

  constexpr static auto const kDebug = false;
  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;
  constexpr static auto const kLongestNodeDistance = cost_t{300};

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
    best_cost_ = kInfeasible;
  }

  void reset(P::parameters const& params,
             cost_t const max,
             location const& start_loc,
             location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);
    cost1_.clear();
    cost2_.clear();
    clear_mp();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    distance_lon_degrees_ = geo::approx_distance_lng_degrees(
        std::abs(start_loc_.pos_.lat()) > std::abs(end_loc_.pos_.lat())
            ? start_loc_.pos_
            : end_loc_.pos_);
    auto const diameter =
        P::heuristic(params, distapprox(start_loc_.pos_, end_loc_.pos_));
    radius_ =
        diameter < max && max + std::max(diameter, kLongestNodeDistance * 2.0) <
                              std::numeric_limits<cost_t>::max()
            ? std::max(static_cast<cost_t>(diameter * 0.5),
                       kLongestNodeDistance)
            : max;
    max_reached_1_ = false;
    max_reached_2_ = false;
  }

  void add(P::parameters const& params,
           ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d,
           sharing_data const* sharing) {
    auto const heur = heuristic(params, w, l.n_, dir, sharing);
    if (l.cost() + heur < d.n_buckets() - 1U &&
        cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      auto const total = static_cast<cost_t>(l.cost() + heur);
      d.push(label{l.get_node(), total});
    }
  }

  void add_start(P::parameters const& params,
                 ways const& w,
                 label const l,
                 sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    add(params, w, l, direction::kForward, cost1_, pq1_, sharing);
  }

  void add_end(P::parameters const& params,
               ways const& w,
               label const l,
               sharing_data const* sharing) {
    if (kDebug) {
      l.get_node().print(std::cout, w);
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    add(params, w, l, direction::kBackward, cost2_, pq2_, sharing);
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
    auto const dist = distapprox(p, end_loc_.pos_);
    auto const other_dist = distapprox(p, start_loc_.pos_);
    return 0.5 *
           (P::heuristic(params, dist) - P::heuristic(params, other_dist)) *
           (dir == direction::kForward ? 1 : -1);
  }

  static constexpr cost_t clamp_cost(std::uint64_t const c) {
    return static_cast<cost_t>(
        std::min(c, static_cast<std::uint64_t>(kInfeasible - 1U)));
  }

  cost_t get_cost_to_mp(node const n1, node const n2) const {
    auto const f_cost = get_cost<direction::kForward>(n1);
    auto const b_cost = get_cost<direction::kBackward>(n2);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return clamp_cost(static_cast<std::uint64_t>(f_cost) + b_cost);
  }

  template <direction SearchDir, bool WithBlocked, direction PathDir>
  bool run_single(P::parameters const& params,
                  ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    auto const adjusted_max =
        clamp_cost((static_cast<std::uint64_t>(max) + radius_) / 2U);
    auto const is_fwd = PathDir == direction::kForward;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost<PathDir>(curr);
    if (curr_cost < l.cost() - heuristic(params, w, l.n_, PathDir, sharing)) {
      return true;
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    P::template adjacent<SearchDir, WithBlocked>(
        params, r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total =
              clamp_cost(static_cast<std::uint64_t>(curr_cost) + cost);
          auto const heur =
              total + heuristic(params, w, neighbor.n_, PathDir, sharing);
          if (total >= adjusted_max) {
            if (is_fwd) {
              max_reached_1_ = true;
            } else {
              max_reached_2_ = true;
            }
            return;
          }
          if (heur < max &&
              costs[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {

            auto next = label{neighbor, static_cast<cost_t>(heur)};
            next.track(l, r, way, neighbor.get_node(), track);
            pq.push(std::move(next));

            if constexpr (kDebug) {
              std::cout << " -> PUSH\n";
            }
          } else {
            if constexpr (kDebug) {
              std::cout << " -> DOMINATED\n";
            }
          }
        });

    auto const evaluate_meetpoint = [&](cost_t cost, cost_t other_cost,
                                        node meetpoint1, node meetpoint2) {
      if constexpr (kDebug) {
        std::cout << "  potential MEETPOINT found by start ";
        meetpoint1.print(std::cout, w);
      }
      auto const tentative = clamp_cost(cost + other_cost);
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        assert(tentative == get_cost_to_mp(meet_point_1_, meet_point_2_));
        best_cost_ = static_cast<cost_t>(tentative);

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
        auto const other_cost = opposite_candidate->second.cost(curr);
        if (other_cost != kInfeasible) {
          evaluate_meetpoint(curr_cost, other_cost, curr, curr);
        } else {
          auto const pred_it = costs.find(curr.get_key());
          if (pred_it == end(costs)) {
            return;
          }
          auto const pred = pred_it->second.pred(curr);
          if (!pred.has_value()) {
            return;
          }
          P::template adjacent<opposite(SearchDir), WithBlocked>(
              params, r, curr, blocked, sharing, elevations,
              [&](node const neighbor, std::uint32_t const, distance_t,
                  way_idx_t const, std::uint16_t, std::uint16_t,
                  elevation_storage::elevation const, bool const) {
                if (neighbor.get_key() != pred->get_key()) {
                  return;
                }
                auto const opposite_it =
                    opposite_cost_map->find(neighbor.get_key());
                if (opposite_it == end(*opposite_cost_map)) {
                  return;
                }
                auto const opposite_curr = opposite_it->second.pred(neighbor);
                if (!opposite_curr.has_value() ||
                    opposite_curr->get_key() != curr.get_key()) {
                  return;
                }
                auto const opposite_curr_cost =
                    opposite_candidate->second.cost(*opposite_curr);
                auto const pred_cost = get_cost<PathDir>(*pred);
                auto const opposite_pred_cost =
                    opposite_it->second.cost(neighbor);
                auto const evaluate_meetpoint_with_potential_u_turn_cost =
                    [&](cost_t const cost_1, cost_t const cost_2,
                        node const meet_1, node const meet_2) {
                      evaluate_meetpoint(cost_1, cost_2,
                                         is_fwd ? meet_1 : meet_2,
                                         is_fwd ? meet_2 : meet_1);
                    };
                if (static_cast<std::uint64_t>(pred_cost) + opposite_pred_cost >
                    static_cast<std::uint64_t>(curr_cost) +
                        opposite_curr_cost) {
                  evaluate_meetpoint_with_potential_u_turn_cost(
                      pred_cost, opposite_pred_cost, *pred, neighbor);
                } else {
                  evaluate_meetpoint_with_potential_u_turn_cost(
                      curr_cost, opposite_curr_cost, curr, *opposite_curr);
                }
              });
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
      if (static_cast<std::uint64_t>(top_f) + top_r >=
          static_cast<std::uint64_t>(best_cost_) + radius_) {
        if (kDebug) {
          std::cout << "stopping criterion met " << top_f << " " << top_r << " "
                    << best_cost_ << " " << radius_ << std::endl;
        }
        return false;
      }
    }
    return true;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(P::parameters const& params,
           ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    if (radius_ == max) {
      return false;
    }
    while (!pq1_.empty() || !pq2_.empty()) {
      if (!pq1_.empty() &&
          !run_single<SearchDir, WithBlocked, direction::kForward>(
              params, w, r, max, blocked, sharing, elevations, pq1_, cost1_)) {
        break;
      }
      if (!pq2_.empty() &&
          !run_single<opposite(SearchDir), WithBlocked, direction::kBackward>(
              params, w, r, max, blocked, sharing, elevations, pq2_, cost2_)) {
        break;
      }
    }
    if (best_cost_ != kInfeasible && best_cost_ > max) {
      clear_mp();
      return false;
    }
    return !max_reached_1_ || !max_reached_2_;
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

  dial<label, get_bucket> pq1_{get_bucket{}};
  dial<label, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  node meet_point_1_;
  node meet_point_2_;
  cost_t best_cost_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t radius_;
  double distance_lon_degrees_;
  bool max_reached_1_;
  bool max_reached_2_;
};

}  // namespace osr
