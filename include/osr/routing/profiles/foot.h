#pragma once

#include "utl/for_each_bit_set.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/tracking.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <bool IsWheelchair, typename Tracking = noop_tracking>
struct foot {
  static constexpr auto const kMaxMatchDistance = 100U;
  static constexpr auto const kSpeedMetersPerSecond =
      (IsWheelchair ? 0.8 : 1.2F);

  struct node {
    friend bool operator==(node const a, node const b) {
      auto const is_zero = [](level_t const l) {
        return l == kNoLevel || l == level_t{0.F};
      };
      return a.n_ == b.n_ &&
             (a.lvl_ == b.lvl_ || (is_zero(a.lvl_) && is_zero(b.lvl_)));
    }

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(), .lvl_{kNoLevel}};
    }
    constexpr node_idx_t get_node() const noexcept { return n_; }

    constexpr node get_key() const noexcept { return *this; }

    static constexpr mode get_mode() noexcept {
      return IsWheelchair ? mode::kWheelchair : mode::kFoot;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", level=" << lvl_ << ")";
    }

    node_idx_t n_;
    level_t lvl_;
  };

  using key = node;

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, cost_{c}, lvl_{n.lvl_} {}

    constexpr node get_node() const noexcept { return {n_, lvl_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    void track(label const& l,
               ways::routing const& r,
               way_idx_t const w,
               node_idx_t const n,
               bool) {
      tracking_.track(l.tracking_, r, w, n, false);
    }

    node_idx_t n_;
    cost_t cost_;
    level_t lvl_;
    [[no_unique_address]] Tracking tracking_;
  };

  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_lvl_}};
    }
    constexpr cost_t cost(node) const noexcept { return cost_; }
    constexpr bool update(label const& l,
                          node,
                          cost_t const c,
                          node const pred) noexcept {
      if (c < cost_) {
        tracking_ = l.tracking_;
        cost_ = c;
        pred_ = pred.n_;
        pred_lvl_ = pred.lvl_;
        return true;
      }
      return false;
    }

    void write(node, path& p) const { tracking_.write(p); }

    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    level_t pred_lvl_;
    [[no_unique_address]] Tracking tracking_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(auto const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::mix(
          wyhash::hash(static_cast<std::uint64_t>(
              to_idx(n.lvl_ == kNoLevel ? level_t{0.F} : n.lvl_))),
          wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_))));
    }
  };

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t const lvl,
                                 direction,
                                 Fn&& f) {
    auto const p = w.way_properties_[way];
    if (lvl == kNoLevel ||
        (p.from_level() == lvl || p.to_level() == lvl ||
         can_use_elevator(w, n, lvl)) ||
        (lvl == level_t{0.F} &&
         (p.from_level() == kNoLevel && p.to_level() == kNoLevel))) {
      f(node{n, p.from_level()});
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    auto levels = hash_set<level_t>{};
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      // TODO what's with stairs? need to resolve to from_level or to_level?
      auto const p = w.way_properties_[w.node_ways_[n][i]];
      if (lvl == kNoLevel) {
        if (levels.emplace(p.from_level()).second) {
          f(node{n, p.from_level()});
        }
        if (levels.emplace(p.to_level()).second) {
          f(node{n, p.to_level()});
        }
      } else if ((p.from_level() == lvl || p.to_level() == lvl ||
                  p.from_level() == kNoLevel || can_use_elevator(w, n, lvl)) &&
                 levels.emplace(lvl).second) {
        f(node{n, lvl});
      }
    }
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
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

        if (can_use_elevator(w, target_node, n.lvl_)) {
          for_each_elevator_level(
              w, target_node, [&](level_t const target_lvl) {
                auto const dist = w.way_node_dist_[way][std::min(from, to)];
                auto const cost = way_cost(target_way_prop, way_dir, dist) +
                                  node_cost(target_node_prop);
                fn(node{target_node, target_lvl},
                   static_cast<std::uint32_t>(cost), dist, way, from, to,
                   elevation_storage::elevation{}, false);
              });
        } else {
          auto const target_lvl = get_target_level(w, n.n_, n.lvl_, way);
          if (!target_lvl.has_value()) {
            return;
          }

          auto const dist = w.way_node_dist_[way][std::min(from, to)];
          auto const cost = way_cost(target_way_prop, way_dir, dist) +
                            node_cost(target_node_prop);
          fn(node{target_node, *target_lvl}, static_cast<std::uint32_t>(cost),
             dist, way, from, to, elevation_storage::elevation{}, false);
        }
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  static bool is_dest_reachable(ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    if (!get_target_level(w, n.n_, n.lvl_, way).has_value()) {
      return false;
    }

    return true;
  }

  static std::optional<level_t> get_target_level(ways::routing const& w,
                                                 node_idx_t const from_node,
                                                 level_t const from_level,
                                                 way_idx_t const to_way) {
    auto const way_prop = w.way_properties_[to_way];

    if (IsWheelchair && way_prop.is_steps()) {
      return std::nullopt;
    }

    if (way_prop.is_steps() || way_prop.is_ramp()) {
      if (from_level == kNoLevel) {
        return way_prop.from_level() == level_t{0.F} ? way_prop.to_level()
                                                     : way_prop.from_level();
      } else if (way_prop.from_level() == from_level) {
        return way_prop.to_level();
      } else if (way_prop.to_level() == from_level) {
        return way_prop.from_level();
      } else {
        return std::nullopt;
      }
    } else if (can_use_elevator(w, to_way, from_level)) {
      return from_level;
    } else if (can_use_elevator(w, from_node, way_prop.from_level(),
                                from_level)) {
      return way_prop.from_level();
    } else if (way_prop.from_level() == from_level ||
               way_prop.from_level() == kNoLevel || from_level == kNoLevel) {
      return way_prop.from_level();
    } else {
      return std::nullopt;
    }
  }

  static bool can_use_elevator(ways::routing const& w,
                               way_idx_t const way,
                               level_t const a,
                               level_t const b = kNoLevel) {
    return w.way_properties_[way].is_elevator() &&
           can_use_elevator(w, w.way_nodes_[way][0], a, b);
  }

  template <typename Fn>
  static void for_each_elevator_level(ways::routing const& w,
                                      node_idx_t const n,
                                      Fn&& f) {
    auto const p = w.node_properties_[n];
    if (p.is_multi_level()) {
      utl::for_each_set_bit(get_elevator_multi_levels(w, n), [&](auto&& l) {
        f(level_t{static_cast<std::uint8_t>(l)});
      });
    } else {
      f(p.from_level());
      f(p.to_level());
    }
  }

  static bool can_use_elevator(ways::routing const& w,
                               node_idx_t const n,
                               level_t const a,
                               level_t const b = kNoLevel) {
    auto const p = w.node_properties_[n];
    if (!p.is_elevator()) {
      return false;
    }

    if (p.is_multi_level()) {
      auto const levels = get_elevator_multi_levels(w, n);
      return (a == kNoLevel || utl::has_bit_set(levels, to_idx(a))) &&
             (b == kNoLevel || utl::has_bit_set(levels, to_idx(b)));
    } else {
      return (a == kNoLevel || a == p.from_level() || a == p.to_level()) &&
             (b == kNoLevel || b == p.from_level() || b == p.to_level());
    }
  }

  static level_bits_t get_elevator_multi_levels(ways::routing const& w,
                                                node_idx_t const n) {
    auto const it = std::lower_bound(
        begin(w.multi_level_elevators_), end(w.multi_level_elevators_), n,
        [](auto&& x, auto&& y) { return x.first < y; });
    assert(it != end(w.multi_level_elevators_) && it->first == n);
    return it->second;
  }

  static constexpr cost_t way_cost(way_properties const e,
                                   direction,
                                   std::uint16_t const dist) {
    if ((e.is_foot_accessible() ||
         (!e.is_sidewalk_separate() && e.is_bike_accessible())) &&
        (!IsWheelchair || !e.is_steps())) {
      return (!e.is_foot_accessible() || e.is_sidewalk_separate() ? 90 : 0) +
             static_cast<cost_t>(std::round(
                 dist / (kSpeedMetersPerSecond + (e.is_big_street_ ? -0.2 : 0) +
                         (e.motor_vehicle_no_ ? 0.1 : 0.0))));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return n.is_walk_accessible() ? (n.is_elevator() ? 90U : 0U) : kInfeasible;
  }

  static constexpr double heuristic(double const dist) {
    return dist / (kSpeedMetersPerSecond + 0.1);
  }

  static constexpr node get_reverse(node const n) { return n; }
};

}  // namespace osr
