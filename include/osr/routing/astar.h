#pragma once

#include <cassert>
#include <cstdint>
#include <algorithm>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/dial.h"
#include "osr/routing/profile.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "utl/to_vec.h"
#include "utl/verify.h"

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

  static constexpr auto const kDebug = false;

  constexpr static auto const kDistanceLatDegrees =
      geo::kEarthRadiusMeters * geo::kPI / 180;

  struct get_bucket {
    cost_t operator()(label const& l) { return l.cost(); }
  };

  void reset(cost_t const max,
             location const& start_loc,
             location const& end_loc) {
    pq_.clear();
    pq_.n_buckets(max + 1U);
    cost_.clear();
    max_reached_ = false;
    destinations_.clear();
    remaining_destinations_ = 0U;
    early_termination_max_cost_ = kInfeasible;
    terminated_early_max_cost_ = false;
    distance_lon_degrees_ = geo::approx_distance_lng_degrees(
        std::abs(start_loc.pos_.lat()) > std::abs(end_loc.pos_.lat())
            ? start_loc.pos_
            : end_loc.pos_);
  }

  void add_start(P::parameters const& params,
                 ways const& w,
                 sharing_data const* sharing,
                 label const l) {
    utl::verify(!destinations_.empty(),
                "astar: add_destination must be called before add_start");
    auto const heur = heuristic(params, w, sharing, l.get_node().get_node());
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
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

  void add_destination(P::parameters const&,
                       ways const& w,
                       sharing_data const* sharing,
                       node const n) {
    auto it = std::lower_bound(begin(destinations_), end(destinations_), n);
    if (it == end(destinations_) || *it != n) {
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
      auto const curr_node = l.get_node();
      auto const curr_cost = get_cost(curr_node);
      auto const curr_heur =
          heuristic(params, w, sharing, curr_node.get_node());

      if (curr_cost < l.cost() - curr_heur) {
        continue;
      }

      if constexpr (EarlyTermination) {
        if (std::find(begin(destinations_), end(destinations_), curr_node) !=
            end(destinations_)) {
          --remaining_destinations_;
          early_termination_max_cost_ = std::min(
              early_termination_max_cost_,
              static_cast<cost_t>(
                  std::min({static_cast<std::uint64_t>(curr_cost) * 2,
                            static_cast<std::uint64_t>(
                                curr_cost + P::slow_heuristic(params, 10000U)),
                            static_cast<std::uint64_t>(kInfeasible - 1U)})));
          if (remaining_destinations_ == 0U) {
            break;
          }
        }
        if (curr_cost > early_termination_max_cost_) {
          terminated_early_max_cost_ = true;
          break;
        }
      }

      if constexpr (kDebug) {
        std::cout << "EXTRACT ";
        l.get_node().print(std::cout, w);
        std::cout << "\n";
      }

      P::template adjacent<SearchDir, WithBlocked>(
          params, r, curr_node, blocked, sharing, elevations,
          [&](node const neighbor, std::uint32_t const cost, distance_t,
              way_idx_t const way, std::uint16_t, std::uint16_t,
              elevation_storage::elevation, bool const track) {
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
            if (heur < max &&
                cost_[neighbor.get_key()].update(
                    l, neighbor, static_cast<cost_t>(total), curr_node)) {
              auto next = label{neighbor, static_cast<cost_t>(heur)};
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
    return dist > 0.0 ? P::heuristic(params, dist) : 0.0;
  }

  dial<label, get_bucket> pq_{get_bucket{}};
  ankerl::unordered_dense::map<key, entry, hash> cost_;
  bool max_reached_{};

  std::vector<node> destinations_;
  std::size_t remaining_destinations_{0U};
  cost_t early_termination_max_cost_{kInfeasible};
  bool terminated_early_max_cost_{false};
  geo::latlng dest_centroid_{};
  double dest_radius_{};
  double distance_lon_degrees_{};
};

}  // namespace osr
