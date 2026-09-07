#pragma once

#include <cassert>
#include <cstdint>

#include <algorithm>
#include <limits>
#include <optional>
#include <vector>

#include "utl/to_vec.h"
#include "utl/verify.h"

#include "geo/constants.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
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
struct astar {
  using profile_t = P;
  using key = typename P::key;
  using label = typename P::label;
  using node = typename P::node;
  using entry = typename P::entry;
  using hash = typename P::hash;
  using params_t = search_params<typename P::parameters>;

  static constexpr auto const kDebug = false;

  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(params_t const& p) {
    params_ = p;
    pq_.clear();
    pq_.n_buckets(params_.max_ + 1U);
    cost_.clear();
    arena_.reset();
    max_reached_ = false;
    destinations_.clear();
    settled_.clear();
    remaining_destinations_ = 0U;
    early_termination_max_cost_ = kInfeasible;
    all_settled_key_ = kInfeasible;
    terminated_early_max_cost_ = false;
    auto const& from = params_.start_loc_;
    auto const& to = params_.end_loc_;
    distance_lon_degrees_ = geo::approx_distance_lng_degrees(
        std::abs(from.pos_.lat()) > std::abs(to.pos_.lat()) ? from.pos_
                                                            : to.pos_);
    beeline_distance_ = geo::distance(from.pos_, to.pos_);
  }

  void reset_pq() { pq_.buckets_ = {}; }

  void add_start(label const l) { add_start(l, duration_from_cost(l.cost())); }

  void add_start(label const l, duration_t const duration) {
    utl::verify(!destinations_.empty(),
                "astar: add_destination must be called before add_start");
    auto const& w = params_.w();
    auto const heur = heuristic(params_.profile_, w, params_.sharing(),
                                l.get_node().get_node());
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid(), duration, *w.r_,
                                             arena_)) {
      auto const cost_with_heur = l.cost() + heur;
      if constexpr (kDebug) {
        std::cout << "START ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }
      if (cost_with_heur >= pq_.n_buckets()) {
        if constexpr (kDebug) {
          std::cout << "  start label skipped: cost = " << cost_with_heur
                    << " (" << l.cost() << " + " << heur
                    << ") >= " << pq_.n_buckets() << "\n";
        }
        return;
      }
      auto const l_with_heur =
          label{l.get_node(), static_cast<cost_t>(cost_with_heur)};
      pq_.push(l_with_heur);
    }
  }

  void add_destination(node const n) {
    auto const& w = params_.w();
    auto const* sharing = params_.sharing();
    auto it = std::lower_bound(begin(destinations_), end(destinations_), n);
    if (it == end(destinations_) || *it != n) {
      settled_.insert(begin(settled_) + std::distance(begin(destinations_), it),
                      false);
      destinations_.insert(it, n);
      ++remaining_destinations_;

      // recalculate centroid and radius
      auto positions = utl::to_vec(destinations_, [&](auto const& dest) {
        return get_node_pos(w, sharing, dest.get_node());
      });
      double sum_lat = 0.0;
      double sum_lng = 0.0;
      for (auto const& p : positions) {
        sum_lat += p.lat_;
        sum_lng += p.lng_;
      }
      auto const count = static_cast<double>(positions.size());
      dest_centroid_ = {sum_lat / count, sum_lng / count};
      dest_radius_ = 0.0;
      for (auto const& p : positions) {
        dest_radius_ = std::max(dest_radius_, geo::distance(dest_centroid_, p));
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

  template <direction SearchDir, bool WithBlocked>
  bool run() {
    auto const& params = params_.profile_;
    auto const& w = params_.w();
    auto const& r = params_.r();
    auto const max = params_.max_;
    auto const start_time = params_.start_time_;
    auto const* const blocked = params_.blocked_;
    auto const* const sharing = params_.sharing();
    auto const* const elevations = params_.elevations_;

    while (!pq_.empty()) {
      auto l = pq_.pop();
      auto const curr_node = l.get_node();
      auto const curr_cost = get_cost(curr_node);
      auto const curr_heur =
          heuristic(params, w, sharing, curr_node.get_node());

      if (curr_cost < l.cost() - curr_heur) {
        continue;
      }

      if constexpr (EarlyTermination) {
        // Same duration tie break as in dijkstra, but the queue is keyed on
        // l.cost() = curr_cost + curr_heur, so we can only terminate once
        // that bucket is empty.
        // Unlike in dijkstra, 0-cost edges aren't needed here: an edge the
        // heuristic is exact on also puts both of its nodes in one bucket.
        if (all_settled_key_ == kInfeasible) {
          if (settle_destination(curr_node)) {
            early_termination_max_cost_ = std::min(
                early_termination_max_cost_,
                static_cast<cost_t>(std::min(
                    {static_cast<std::uint64_t>(curr_cost) * 2 +
                         static_cast<std::uint64_t>(P::upper_bound_heuristic(
                             params, std::min(beeline_distance_, 1000.0))),
                     static_cast<std::uint64_t>(
                         curr_cost + P::upper_bound_heuristic(params, 10000U)),
                     static_cast<std::uint64_t>(kInfeasible - 1U)})));
            if (remaining_destinations_ == 0U) {
              all_settled_key_ = l.cost();
            }
          }
          if (all_settled_key_ == kInfeasible &&
              curr_cost > early_termination_max_cost_) {
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

      P::template adjacent<SearchDir, WithBlocked>(
          params, r, w.timezones_, curr_node,
          cost_.at(curr_node.get_key()).duration(curr_node), start_time,
          blocked, sharing, elevations,
          [&](node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t, way_idx_t const way,
              std::uint16_t, std::uint16_t, elevation_storage::elevation,
              bool const track) {
            if constexpr (kDebug) {
              std::cout << "  NEIGHBOR ";
              neighbor.print(std::cout, w);
            }

            auto const total = static_cast<std::uint64_t>(curr_cost) + cost;
            auto const heur =
                static_cast<double>(total) +
                heuristic(params, w, sharing, neighbor.get_node());
            if (total >= max) {
              max_reached_ = true;
              return;
            }
            auto const total_duration = clamp_add_duration(
                cost_.at(curr_node.get_key()).duration(curr_node), duration);
            auto const updated = [&]() {
              if (heur >= max) {
                return false;
              }
              auto next = label{neighbor, static_cast<cost_t>(heur)};
              next.track(l, r, way, neighbor.get_node(), track);
              if (!cost_[neighbor.get_key()].update(
                      next, neighbor, static_cast<cost_t>(total), curr_node,
                      total_duration, r, arena_)) {
                return false;
              }
              pq_.push(std::move(next));
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
    }
    return !max_reached_;
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

  static geo::latlng get_node_pos(ways const& w,
                                  sharing_data const* sharing,
                                  node_idx_t const n) {
    if (n == node_idx_t::invalid()) {
      return {};
    } else if (w.is_additional_node(n)) {
      assert(sharing != nullptr);
      return sharing->get_additional_node_coordinates(n);
    } else {
      return w.get_node_pos(n).as_latlng();
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
                   sharing_data const* sharing,
                   node_idx_t const n) const {
    auto const node_pos = get_node_pos(w, sharing, n);
    auto const dist = distapprox(node_pos, dest_centroid_) - dest_radius_;
    return dist > 0.0 ? P::lower_bound_heuristic(params, dist) : 0.0;
  }

  params_t params_;

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
  entry_storage_arena arena_;
  bool max_reached_{};

  std::vector<node> destinations_;
  std::vector<bool> settled_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  cost_t all_settled_key_{kInfeasible};
  bool terminated_early_max_cost_{false};
  geo::latlng dest_centroid_{};
  double dest_radius_{};
  double distance_lon_degrees_{};
  double beeline_distance_{};
};

}  // namespace osr
