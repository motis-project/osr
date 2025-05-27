#pragma once

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

constexpr auto const kElevationNoCost = 0U;
constexpr auto const kElevationLowCost = 570U;
constexpr auto const kElevationHighCost = 3700U;

// Routing const configuration (cost, exp)
// cost:
// Higher costs will favor flatter routes, even if these take way longer.
// exp:
// Increase cost to more penalize ways with higher incline
// Examples:
// (250, 1000)  // Low costs, penalize total elevation
// (800, 1000)  // Higher costs, penalize total elevation
// (570, 2100)  // Low costs, penalize ways with higher incline
// (3700, 2100)  // Higher costs, penalize ways with higher incline

template <unsigned int ElevationUpCost,
          unsigned int ElevationExponentThousandth = 2100U>
struct bike {
  static constexpr auto const kMaxMatchDistance = 100U;
  static constexpr auto const kOffroadPenalty = 1U;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid()};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }

    constexpr node get_key() const noexcept { return *this; }

    static constexpr mode get_mode() noexcept { return mode::kBike; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << w.node_to_osm_[n_];
    }

    node_idx_t n_;
  };

  using key = node;

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    level_t lvl_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_)));
    }
  };

  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      return pred_ == node_idx_t::invalid() ? std::nullopt
                                            : std::optional{node{pred_}};
    }
    constexpr cost_t cost(node) const noexcept { return cost_; }
    constexpr bool update(label const&,
                          node,
                          cost_t const c,
                          node const pred) noexcept {
      if (c < cost_) {
        cost_ = c;
        pred_ = pred.n_;
        return true;
      }
      return false;
    }

    void write(node, path&) const {}

    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
  };

  template <typename Fn>
  static void resolve_start_node(ways::routing const&,
                                 way_idx_t,
                                 node_idx_t const n,
                                 level_t,
                                 direction,
                                 Fn&& f) {
    f(node{n});
  }

  template <typename Fn>
  static void resolve_all(ways::routing const&,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    f(node{n});
  }

  static bool is_dest_reachable(
      ways::routing const&, node, way_idx_t, direction, direction) {
    return true;
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const* elevations,
                       Fn&& fn) {
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }
        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const elevation = [&]() {
          auto const e = (from < to) ? get_elevations(elevations, way, from)
                                     : get_elevations(elevations, way, to);
          auto const in_direction =
              (SearchDir == direction::kForward) == (from < to);
          return in_direction ? e : e.swapped();
        }();
        auto const elevation_cost = static_cast<cost_t>(
            ElevationUpCost > 0U && dist > 0U
                ? (ElevationExponentThousandth > 1000U
                       ? ElevationUpCost *
                             std::pow(
                                 static_cast<double>(to_idx(elevation.up_)) /
                                     dist,
                                 ElevationExponentThousandth / 1000.0)
                       : ElevationUpCost * to_idx(elevation.up_) / dist)
                : 0);
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop) + elevation_cost;
        fn(node{target_node}, static_cast<std::uint32_t>(cost), dist, way, from,
           to, elevation, false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  static constexpr cost_t way_cost(way_properties const e,
                                   direction,
                                   std::uint16_t const dist) {
    if (e.is_bike_accessible()) {
      return static_cast<cost_t>(std::round(dist / 4.F));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }

  static constexpr double heuristic(double dist) { return dist / 4.F; }

  static constexpr node get_reverse(node const n) { return n; }
};

}  // namespace osr
