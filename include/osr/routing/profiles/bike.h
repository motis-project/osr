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
constexpr auto const kBikeSpeedMetersPerSecond = 3.8F;

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

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(), .dir_ = direction::kForward};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    static constexpr mode get_mode() noexcept { return mode::kBike; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_)
                 << ")";
    }

    node_idx_t n_;
    direction dir_;
  };

  using key = node_idx_t;

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, dir_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    direction dir_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  struct entry {
    entry() { utl::fill(cost_, kInfeasible); }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_dir_[idx]}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(label const&,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_dir_[idx] = pred.dir_;
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      return n.dir_ == direction::kForward ? 0U : 1U;
    }

    static constexpr node get_node(node_idx_t const n,
                                   std::size_t const index) {
      return node{n, index == 0U ? direction::kForward : direction::kBackward};
    }

    void write(node, path&) const {}

    std::array<node_idx_t, 2U> pred_;
    std::array<direction, 2U> pred_dir_;
    std::array<cost_t, 2U> cost_;
  };

  template <typename Fn>
  static void resolve_start_node(ways::routing const&,
                                 way_idx_t,
                                 node_idx_t const n,
                                 level_t,
                                 direction,
                                 Fn&& f) {
    f(node{n, direction::kForward});
    f(node{n, direction::kBackward});
  }

  template <typename Fn>
  static void resolve_all(ways::routing const&,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    f(node{n, direction::kForward});
    f(node{n, direction::kBackward});
  }

  static bool is_dest_reachable(ways::routing const& w,
                                node,
                                way_idx_t const way,
                                direction const way_dir,
                                direction) {
    return way_cost(w.way_properties_[way], way_dir, 0U) != kInfeasible;
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
        fn(node{target_node, way_dir}, static_cast<std::uint32_t>(cost), dist,
           way, from, to, elevation, false);
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
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_bike_accessible() &&
        (dir == direction::kForward || !e.is_oneway_bike())) {
      return static_cast<cost_t>(std::round(dist / kBikeSpeedMetersPerSecond));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }

  static constexpr double heuristic(double const dist) {
    return dist / kBikeSpeedMetersPerSecond;
  }

  static constexpr node get_reverse(node const n) {
    return {n, opposite(n.dir_)};
  }
};

}  // namespace osr
