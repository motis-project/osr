#pragma once

#include <limits>

#include "utl/verify.h"

#include "geo/constants.h"
#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <typename Profile>
struct bidirectional {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  constexpr static auto const kDebug = false;
  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void clear_mp() {
    meet_point_1_ = meet_point_1_.invalid();
    meet_point_2_ = meet_point_2_.invalid();
    best_cost_ = kInfeasible;
  }

  void reset(cost_t const max,
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
    distance_lon_degrees_ =
        std::clamp(1.0 - std::max(std::abs(start_loc_.pos_.lat()),
                                  std::abs(end_loc_.pos_.lat())) /
                             90.0,
                   0.0, 1.0) *
        kDistanceLatDegrees;
    auto const diameter =
        Profile::heuristic(distapprox(start_loc_.pos_, end_loc_.pos_));
    radius_ =
        diameter < max && max + diameter < std::numeric_limits<cost_t>::max()
            ? static_cast<cost_t>(diameter * 0.5)
            : max;
  }

  void add(ways const& w,
           label const l,
           direction const dir,
           cost_map& cost_map,
           dial<label, get_bucket>& d) {
    auto const heur = heuristic(w, l.n_, dir);
    if (l.cost() + heur < d.n_buckets() - 1 &&
        cost_map[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                                node::invalid())) {
      auto const total = static_cast<cost_t>(l.cost() + heur);
      d.push(label{l.get_node(), total});
    }
  }

  void add_start(ways const& w, label const l) {
    if (kDebug) {
      std::cout << "starting" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kForward, cost1_, pq1_);
  }

  void add_end(ways const& w, label const l) {
    if (kDebug) {
      std::cout << "ending" << l.get_node().n_ << std::endl;
    }
    add(w, l, direction::kBackward, cost2_, pq2_);
  }

  cost_t get_cost(node const n, direction const dir) const {
    auto const cost = dir == direction::kForward ? &cost1_ : &cost2_;  // TODO
    auto const it = cost->find(n.get_key());
    return it != end(*cost) ? it->second.cost(n) : kInfeasible;
  }

  double distapprox(geo::latlng const& p1, geo::latlng const& p2) const {
    auto const x = std::abs(p1.lat() - p2.lat()) * kDistanceLatDegrees;
    auto const y = std::abs(p1.lng() - p2.lng()) * distance_lon_degrees_;

    return std::max(std::max(x, y), (x + y) / 1.42);
  }

  double heuristic(ways const& w, node_idx_t idx, direction const dir) const {
    auto const p = w.get_node_pos(idx).as_latlng();
    auto const dist = distapprox(p, end_loc_.pos_);
    auto const other_dist = distapprox(p, start_loc_.pos_);
    return 0.5 * (Profile::heuristic(dist) - Profile::heuristic(other_dist)) *
           (dir == direction::kForward ? 1 : -1);
  }

  cost_t get_cost_from_x(node const n, cost_map const& cost) const {
    auto const it = cost.find(n.get_key());
    return it == cost.end() ? kInfeasible : it->second.cost(n);
  }

  cost_t get_cost_from_start(node const n) const {
    return get_cost_from_x(n, cost1_);
  }

  cost_t get_cost_from_end(node const n) const {
    return get_cost_from_x(n, cost2_);
  }

  cost_t get_cost_to_mp(node const n1, node const n2) const {
    auto const f_cost = get_cost(n1, direction::kForward);
    auto const b_cost = get_cost(n2, direction::kBackward);
    if (f_cost == kInfeasible || b_cost == kInfeasible) {
      return kInfeasible;
    }
    return f_cost + b_cost;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run_single(ways const& w,
                  ways::routing const& r,
                  cost_t const max,
                  bitvec<node_idx_t> const* blocked,
                  sharing_data const* sharing,
                  elevation_storage const* elevations,
                  dial<label, get_bucket>& pq,
                  cost_map& costs) {
    auto const adjusted_max = (max + radius_) / 2U;

    auto const l = pq.pop();
    auto const curr = l.get_node();
    auto const curr_cost = get_cost(curr, SearchDir);
    if (curr_cost < l.cost() - heuristic(w, l.n_, SearchDir)) {
      return true;
    }
    if constexpr (kDebug) {
      std::cout << "EXTRACT ";
      l.get_node().print(std::cout, w);
      std::cout << "\n";
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked, sharing, elevations,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t,
            elevation_storage::elevation const, bool const track) {
          if constexpr (kDebug) {
            std::cout << "  NEIGHBOR ";
            neighbor.print(std::cout, w);
          }
          auto const total = curr_cost + cost;
          if (neighbor.n_ >= w.node_to_osm_.size()) {
            return;
          }
          auto const heur = total + heuristic(w, neighbor.n_, SearchDir);
          if (total < adjusted_max && heur < max &&
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
      auto const tentative = cost + other_cost;
      if (tentative < best_cost_) {
        meet_point_1_ = meetpoint1;
        meet_point_2_ = meetpoint2;
        assert(tentative == get_cost_to_mp(meet_point_1_, meet_point_2_));
        best_cost_ = get_cost_to_mp(meet_point_1_, meet_point_2_);

        if constexpr (kDebug) {
          std::cout << " with cost " << best_cost_ << " -> ACCEPTED\n";
        }
      } else if constexpr (kDebug) {
        std::cout << " -> DOMINATED\n";
      }
    };

    auto const handle_end_of_way_meetpoint = [&]() {
      auto const opposite_cost_map =
          opposite(SearchDir) == direction::kForward ? &cost1_ : &cost2_;
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
          Profile::template adjacent<opposite(SearchDir), WithBlocked>(
              r, curr, blocked, sharing, elevations,
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
                if (SearchDir == direction::kForward) {
                  evaluate_meetpoint(
                      curr_cost,
                      opposite_candidate->second.cost(*opposite_curr), curr,
                      *opposite_curr);
                } else {
                  evaluate_meetpoint(
                      curr_cost,
                      opposite_candidate->second.cost(*opposite_curr),
                      *opposite_curr, curr);
                }
              });
        }
      }
    };

    handle_end_of_way_meetpoint();

    if (best_cost_ != kInfeasible) {
      auto const top_f =
          pq1_.empty() ? get_cost_from_start(meet_point_1_)
                       : pq1_.buckets_[pq1_.get_next_bucket()].back().cost();
      auto const top_r =
          pq2_.empty() ? get_cost_from_end(meet_point_2_)
                       : pq2_.buckets_[pq2_.get_next_bucket()].back().cost();
      if (top_f + top_r >= best_cost_ + radius_) {
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
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations) {
    if (radius_ == max) {
      return;
    }
    auto pq = &pq1_;
    auto dir = direction::kForward;

    while (!pq->empty()) {
      auto const cont =
          dir == direction::kForward
              ? run_single<SearchDir, WithBlocked>(w, r, max, blocked, sharing,
                                                   elevations, *pq, cost1_)
              : run_single<opposite(SearchDir), WithBlocked>(
                    w, r, max, blocked, sharing, elevations, *pq, cost2_);
      if (!cont) {
        break;
      }
      pq = dir == direction::kForward ? &pq2_ : &pq1_;
      dir = opposite(dir);
    }
    if (best_cost_ > max) {
      clear_mp();
    }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           sharing_data const* sharing,
           elevation_storage const* elevations,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward ? run<direction::kForward, false>(
                                       w, r, max, blocked, sharing, elevations)
                                 : run<direction::kBackward, false>(
                                       w, r, max, blocked, sharing, elevations);
    } else {
      dir == direction::kForward ? run<direction::kForward, true>(
                                       w, r, max, blocked, sharing, elevations)
                                 : run<direction::kBackward, true>(
                                       w, r, max, blocked, sharing, elevations);
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
};

}  // namespace osr
