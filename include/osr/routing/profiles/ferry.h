#pragma once

#include <chrono>
#include <bitset>
#include <optional>

#include "boost/json/object.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct ferry {
  static constexpr auto const kName = "ferry";
  static constexpr auto const kMaxMatchDistance = 200U;

  using key = node_idx_t;

  struct parameters {
    using profile_t = ferry;
  };

  struct node {
    friend bool operator==(node, node) = default;

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return a.n_ == b.n_;
    }

    static constexpr node invalid() noexcept {
      return node{.n_ = node_idx_t::invalid()};
    }

    boost::json::object geojson_properties(ways const&) const {
      return boost::json::object{{"node_id", n_.v_}, {"type", "ferry"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return {};
    }

    static constexpr mode get_mode() noexcept { return mode::kFerry; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      if (n_ >= w.n_nodes()) {
        return out << "(node=" << osm_node_idx_t{to_idx(n_)} << "*)";
      } else {
        return out << "(node=" << w.node_to_osm_[n_] << ")";
      }
    }

    node_idx_t n_;
  };

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    cost_t cost_;
  };

  struct entry {
    constexpr std::optional<node> pred(node const) const noexcept {
      return pred_ == node_idx_t::invalid() ? std::nullopt
                                            : std::optional{node{pred_}};
    }

    constexpr cost_t cost(node const) const noexcept { return cost_; }

    constexpr bool update(label const&,
                          node const,
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

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  static node create_node(node_idx_t const n,
                          level_t const,
                          way_pos_t const,
                          direction const) {
    return node{n};
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const&,
                                 way_idx_t const,
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

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(parameters const& params,
                       ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* additional,
                       elevation_storage const*,
                       Fn&& fn) {
    if (additional != nullptr) {
      for_each_additional_edge<ferry>(
          params, w, n, additional,
          [&](additional_edge const& ae, cost_t const edge_cost,
              direction const) {
            auto const target = node{ae.to_};
            auto cost = edge_cost;

            if (!additional->is_additional_node(ae.to_)) {
              cost = clamp_cost(static_cast<std::uint64_t>(cost) +
                                node_cost(params, w.node_properties_[ae.to_]));
            }

            fn(target, cost, ae.distance_, ae.underlying_way_, 0, 0,
               elevation_storage::elevation{}, false);
          });

      if (additional->is_additional_node(n.n_)) {
        return;
      }
    }

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
        if (node_cost(params, target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(params, target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        auto const dist = w.get_way_node_distance(way, std::min(from, to));
        auto const target = node{target_node};
        auto const cost = way_cost(params, target_way_prop, way_dir, dist) +
                          node_cost(params, target_node_prop);
        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(params, target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    return true;
  }

  static constexpr cost_t way_cost(parameters const&,
                                   way_properties const& e,
                                   direction const,
                                   distance_t const dist) {
    if (e.is_ferry_accessible()) {
      return static_cast<cost_t>((dist / 10));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(parameters const&, node_properties const&) {
    return 0U;
  }

  static constexpr double lower_bound_heuristic(parameters const&,
                                                double const dist) {
    return dist / 10;
  }

  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return lower_bound_heuristic(params, dist);
  }

  static constexpr node get_reverse(node const n) { return n; }
};

}  // namespace osr
