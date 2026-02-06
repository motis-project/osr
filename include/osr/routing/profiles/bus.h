#pragma once

#include <bitset>
#include <optional>
#include <tuple>

#include "boost/json/object.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct bus {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{60U};
  static constexpr auto const kPrivateGatePenalty = cost_t{60U};

  using key = node_idx_t;

  struct parameters {
    using profile_t = bus;
  };

  struct node {
    friend bool operator==(node, node) = default;

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return std::tie(a.n_, a.way_, a.dir_) < std::tie(b.n_, b.way_, b.dir_);
    }

    static constexpr node invalid() noexcept {
      return node{
          .n_ = node_idx_t::invalid(), .way_ = 0U, .dir_ = direction::kForward};
    }

    boost::json::object geojson_properties(ways const&) const {
      return boost::json::object{{"node_id", n_.v_}, {"type", "bus"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

    static constexpr mode get_mode() noexcept { return mode::kCar; }

    way_idx_t get_way(ways::routing const& w,
                      sharing_data const* additional) const {
      if (additional != nullptr && additional->is_additional_node(n_)) {
        auto const& edges = additional->additional_edges_.at(n_);
        auto const& edge = edges.at(cista::to_idx(way_));
        return edge.underlying_way_;
      }
      return w.node_ways_[n_][way_];
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      if (n_ >= w.n_nodes()) {
        return out << "(node=" << osm_node_idx_t{to_idx(n_)}
                   << "*, dir=" << to_str(dir_) << ")";
      } else {
        return out << "(node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_)
                   << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                   << ")";
      }
    }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
  };

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    cost_t cost_;
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U /* FWD+BWD */;

    entry() { utl::fill(cost_, kInfeasible); }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
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
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        return true;
      }
      return false;
    }

    void write(node, path&) const {}

    static constexpr node get_node(node_idx_t const n,
                                   std::size_t const index) {
      return node{n, static_cast<way_pos_t>(index % kMaxWays),
                  to_dir((index / kMaxWays) != 0U)};
    }

    static constexpr std::size_t get_index(node const n) {
      return (n.dir_ == direction::kForward ? 0U : 1U) * kMaxWays + n.way_;
    }

    static constexpr direction to_dir(bool const b) {
      return b == false ? direction::kForward : direction::kBackward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kForward ? false : true;
    }

    std::array<node_idx_t, kN> pred_;
    std::array<way_pos_t, kN> pred_way_;
    std::bitset<kN> pred_dir_;
    std::array<cost_t, kN> cost_;
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
                          way_pos_t const way,
                          direction const dir) {
    return node{n, way, dir};
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t,
                                 direction,
                                 Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      if (ways[i] == way) {
        f(node{n, i, direction::kForward});
        f(node{n, i, direction::kBackward});
      }
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      f(node{n, i, direction::kForward});
      f(node{n, i, direction::kBackward});
    }
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
      if (auto const it = additional->additional_edges_.find(n.n_);
          it != end(additional->additional_edges_)) {
        for (auto const& ae : it->second) {
          auto const edge_dir =
              ae.reverse_ ? direction::kBackward : direction::kForward;
          assert(ae.underlying_way_ != way_idx_t::invalid());
          auto const way_props = w.way_properties_[ae.underlying_way_];

          auto const edge_cost =
              way_cost(params, way_props, edge_dir, ae.distance_);
          if (edge_cost == kInfeasible) {
            continue;
          }

          if (!is_additional_node(additional, ae.node_)) {
            auto const target_node_prop = w.node_properties_[ae.node_];
            if (node_cost(target_node_prop) == kInfeasible) {
              continue;
            }
          }

          if (!is_additional_node(additional, n.n_)) {
            if (w.is_restricted<SearchDir>(
                    n.n_, n.way_,
                    w.get_way_pos(n.n_, ae.underlying_way_ /*,
                                  ae.node_in_way_idx_from_*/))) {
              continue;
            }
          }

          auto const prev_way = n.get_way(w, additional);

          auto const is_u_turn =
              prev_way == ae.underlying_way_ && n.dir_ != edge_dir;

          auto const target =
              node{ae.node_,
                   additional->get_way_pos(w, ae.node_,
                   ae.underlying_way_/*,
                                           target_node_in_way_idx*/),
                   edge_dir};
          auto cost = edge_cost;

          if (is_u_turn) {
            cost += kUturnPenalty;
          }

          if (!is_additional_node(additional, ae.node_)) {
            cost += node_cost(w.node_properties_[ae.node_]);
          }

          fn(target, cost, ae.distance_, ae.underlying_way_, 0, 0,
             elevation_storage::elevation{}, false);
        }
      }

      if (is_additional_node(additional, n)) {
        return;
      }
    }

    auto way_pos = way_pos_t{0U};
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
        if (way_cost(params, target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const target =
            node{target_node, w.get_way_pos(target_node, way, to), way_dir};
        auto const cost = way_cost(params, target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);
        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           false);
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(params, target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    if (w.is_restricted(n.n_, n.way_, w.get_way_pos(n.n_, way), search_dir)) {
      return false;
    }

    return true;
  }

  static constexpr cost_t way_cost(parameters const&,
                                   way_properties const& e,
                                   direction const dir,
                                   distance_t const dist) {
    auto const accessible = e.is_bus_accessible();
    auto const accessible_with_penalty = e.is_bus_accessible_with_penalty();
    if ((accessible || accessible_with_penalty) &&
        (dir == direction::kForward || !e.is_oneway_psv())) {
      auto cost = static_cast<cost_t>((dist / e.max_speed_m_per_s()) *
                                      (e.in_route() ? 1.0 : 1.2));
      if (e.is_parking()) {
        cost *= 2U;
      }
      if (accessible_with_penalty) {
        cost *= e.in_route() ? 2U : 4U;
      }
      return cost;
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const& n) {
    return n.is_bus_accessible()
               ? 0U
               : (n.is_bus_accessible_with_penalty() ? kPrivateGatePenalty
                                                     : kInfeasible);
  }

  static constexpr double heuristic(parameters const&, double const dist) {
    return dist / (130U / 3.6);
  }

  static constexpr double slow_heuristic(parameters const&, double const dist) {
    return dist / (15U / 3.6);
  }

  static constexpr node get_reverse(node const n) {
    return {n.n_, n.way_, opposite(n.dir_)};
  }

  static bool is_additional_node(sharing_data const* additional,
                                 node_idx_t const n) {
    return additional != nullptr && additional->is_additional_node(n);
  }

  static bool is_additional_node(sharing_data const* additional,
                                 node const& n) {
    return is_additional_node(additional, n.n_);
  }
};

}  // namespace osr
