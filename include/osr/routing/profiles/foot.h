#pragma once

#include <optional>

#include "utl/for_each_bit_set.h"

#include "osr/elevation_storage.h"
#include "osr/routing/entry_storage_arena.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/tracking.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

template <bool IsWheelchair, typename Tracking = noop_tracking>
struct foot {
  static constexpr auto const kMaxMatchDistance = 100U;

  struct parameters {
    using profile_t = foot<IsWheelchair, Tracking>;
    float speed_meters_per_second_{IsWheelchair ? 0.8F : 1.2F};
  };

  struct node {
    friend constexpr bool operator==(node const&, node const&) = default;

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return std::tie(a.n_, a.lvl_) < std::tie(b.n_, b.lvl_);
    }

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(), .lvl_{kNoLevel}};
    }
    constexpr node_idx_t get_node() const noexcept { return n_; }

    constexpr node get_key() const noexcept { return *this; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return {};
    }

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
    OSR_NO_UNIQUE_ADDRESS Tracking tracking_;
  };

  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_lvl_}};
    }
    constexpr cost_t cost(node) const noexcept { return cost_; }

    constexpr duration_t duration(node) const noexcept { return duration_; }

    constexpr cost_and_duration cd() const noexcept {
      return {.cost_ = cost_, .duration_ = duration_};
    }

    constexpr bool update(label const& l,
                          node,
                          cost_and_duration const c,
                          node const pred,
                          ways::routing const&,
                          entry_storage_arena&) noexcept {
      if (c < cd()) {
        tracking_ = l.tracking_;
        cost_ = c.cost_;
        duration_ = c.duration_;
        pred_ = pred.n_;
        pred_lvl_ = pred.lvl_;
        return true;
      }
      return false;
    }

    void write(node, path& p) const { tracking_.write(p); }

    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    duration_t duration_{kMaxDuration};
    level_t pred_lvl_;
    OSR_NO_UNIQUE_ADDRESS Tracking tracking_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(auto const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      auto const packed = (static_cast<std::uint64_t>(to_idx(n.lvl_)) << 32U) |
                          static_cast<std::uint64_t>(to_idx(n.n_));
      return wyhash::hash(packed);
    }
  };

  static node create_node(node_idx_t const n,
                          level_t const lvl,
                          way_pos_t const,
                          direction const) {
    return node{n, lvl};
  }

  template <typename Fn>
  static void for_each_node_level(ways::routing const& w,
                                  node_idx_t const n,
                                  Fn&& f) {
    auto levels = level_bits_t{0U};
    auto const emit = [&](level_t const lvl) {
      auto const mask = level_bits_t{1U} << to_idx(lvl);
      if ((levels & mask) == 0U) {
        levels |= mask;
        f(node{n, lvl});
      }
    };
    resolve_all(w, n, [&](node const x) { emit(x.lvl_); });
    if (w.node_properties_[n].is_elevator()) {
      for_each_elevator_level(w, n, emit);
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w, node_idx_t const n, Fn&& f) {
    auto levels = std::uint64_t{0U};
    auto const emit = [&](level_t const l) {
      auto const mask = std::uint64_t{1U} << to_idx(l);
      if ((levels & mask) == 0U) {
        levels |= mask;
        f(node{n, l});
      }
    };
    for (auto const way : w.node_ways_[n]) {
      // TODO what's with stairs? need to resolve to from_level or to_level?
      auto const p = w.way_properties_[way];
      emit(p.from_level());
      emit(p.to_level());
    }
  }

  template <typename Fn>
  static void resolve_endpoint(ways::routing const& w,
                               way_idx_t const way,
                               node_idx_t const n,
                               level_t const lvl,
                               route_end const end,
                               endpoint_role,
                               Fn&& f) {
    auto const p = w.way_properties_[way];
    auto const level_compatible =
        lvl == kNoLevel || p.from_level() == lvl || p.to_level() == lvl ||
        can_use_elevator(w, n, lvl) ||
        (lvl == level_t{0.F} && p.from_level() == kNoLevel &&
         p.to_level() == kNoLevel);
    if (!level_compatible) {
      return;
    }

    auto const node_side_level = [&]() {
      if ((p.is_steps() || p.is_ramp()) && n == w.way_nodes_[way].back()) {
        return p.to_level();
      }
      return p.from_level();
    }();
    if (end == route_end::kOrigin) {
      f(node{n, node_side_level});
      return;
    }

    for_each_node_level(w, n, [&](node const candidate) {
      if (node_side_level == kNoLevel || candidate.lvl_ == kNoLevel ||
          candidate.lvl_ == node_side_level ||
          can_use_elevator(w, n, candidate.lvl_, node_side_level)) {
        f(candidate);
      }
    });
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(parameters const& params,
                       ways::routing const& w,
                       timezone_cache_t const& timezones,
                       node const n,
                       duration_t const current_duration,
                       std::optional<routing_time_t> const start_time,
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
        auto const cost_node =
            SearchDir == direction::kForward ? target_node : n.n_;
        if constexpr (WithBlocked) {
          if (blocked->test(cost_node)) {
            return;
          }
        }

        auto const cost_node_prop = w.node_properties_[cost_node];
        if (node_cost(params, cost_node_prop).cost_ == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(params, w, timezones, way, target_way_prop, way_dir, 0U,
                     start_time, current_duration, SearchDir)
                .cost_ == kInfeasible) {
          return;
        }

        auto const emit = [&](level_t const target_lvl) {
          auto const dist = w.get_way_node_distance(way, std::min(from, to));
          auto const step = clamp_add(
              way_cost(params, w, timezones, way, target_way_prop, way_dir,
                       dist, start_time, current_duration, SearchDir),
              node_cost(params, cost_node_prop));
          fn(node{target_node, target_lvl}, step.cost_, step.duration_, dist,
             way, from, to, elevation_storage::elevation{}, false);
        };

        if constexpr (SearchDir == direction::kForward) {
          if (can_use_elevator(w, target_node, n.lvl_)) {
            for_each_elevator_level(w, target_node, emit);
          } else if (auto const target_lvl =
                         get_target_level(w, n.n_, n.lvl_, way);
                     target_lvl.has_value()) {
            emit(*target_lvl);
          }
        } else {
          for_each_node_level(w, target_node, [&](node const predecessor) {
            auto const predecessor_lvl = predecessor.lvl_;
            if (can_use_elevator(w, n.n_, predecessor_lvl)) {
              auto reaches_current = false;
              for_each_elevator_level(w, n.n_, [&](level_t const lvl) {
                reaches_current = reaches_current || lvl == n.lvl_;
              });
              if (reaches_current) {
                emit(predecessor_lvl);
              }
            } else if (auto const reached = get_target_level(
                           w, target_node, predecessor_lvl, way);
                       reached.has_value() && node{n.n_, *reached} == n) {
              emit(predecessor_lvl);
            }
          });
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

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                timezone_cache_t const& timezones,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir,
                                std::optional<routing_time_t> const start_time,
                                duration_t const current_duration) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(params, w, timezones, way, target_way_prop, way_dir, 0U,
                 start_time, current_duration, search_dir)
            .cost_ == kInfeasible) {
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
      if (way_prop.from_level() == kNoLevel &&
          way_prop.to_level() == kNoLevel) {
        return kNoLevel;
      }
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
    } else if (from_level != kNoLevel &&
               can_use_elevator(w, to_way, from_level)) {
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

  static constexpr cost_and_duration way_cost(
      parameters const& params,
      ways::routing const&,
      timezone_cache_t const&,
      way_idx_t const,
      way_properties const e,
      direction,
      distance_t const dist,
      std::optional<routing_time_t> const,
      duration_t const,
      direction const) {
    if (IsWheelchair && e.is_steps()) {
      return infeasible_cost_and_duration();
    }
    if (!e.is_foot_accessible() && !e.is_bike_accessible()) {
      return infeasible_cost_and_duration();
    }
    auto const duration = duration_from_cost(static_cast<cost_t>(std::round(
        static_cast<double>(dist) / params.speed_meters_per_second_)));
    auto const cost = (!e.is_foot_accessible() ? 90U : 0U) +
                      (e.is_sidewalk_separate() ? 45U : 0U) +
                      static_cast<cost_t>(std::round(
                          dist / (params.speed_meters_per_second_ +
                                  (e.is_big_street_ ? -0.2 : 0) +
                                  (e.motor_vehicle_no_ ? 0.1 : 0.0))));
    return {.cost_ = cost, .duration_ = duration};
  }

  static constexpr cost_and_duration endpoint_way_cost(
      parameters const& params,
      ways::routing const& w,
      timezone_cache_t const& timezones,
      node const,
      way_idx_t const way,
      way_properties const& properties,
      direction const way_dir,
      distance_t const distance,
      std::optional<routing_time_t> const start_time,
      duration_t const current_duration,
      direction const search_dir) {
    return way_cost(params, w, timezones, way, properties, way_dir, distance,
                    start_time, current_duration, search_dir);
  }

  static constexpr bool endpoint_root_allowed(parameters const&,
                                              node const,
                                              direction) {
    return true;
  }

  static constexpr cost_and_duration endpoint_transition_cost(
      parameters const&,
      ways::routing const&,
      timezone_cache_t const&,
      node const,
      way_idx_t,
      direction,
      direction,
      std::optional<routing_time_t>,
      duration_t) {
    return {};
  }

  static constexpr cost_and_duration endpoint_node_cost(
      parameters const& params, node const, node_properties const& n) {
    return node_cost(params, n);
  }

  static bool endpoint_way_feasible(parameters const& params,
                                    endpoint_way_query const& q) {
    return q.template feasible<typename parameters::profile_t>(params);
  }

  static constexpr cost_and_duration node_cost(parameters const&,
                                               node_properties const n) {
    return n.is_walk_accessible()
               ? cost_and_duration_from_cost(n.is_elevator() ? 90U : 0U)
               : infeasible_cost_and_duration();
  }

  static constexpr double lower_bound_heuristic(parameters const& params,
                                                double const dist) {
    return dist / (params.speed_meters_per_second_ + 0.1);
  }
  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return dist / (params.speed_meters_per_second_ - 0.2);
  }

  static constexpr node get_reverse(node const n) { return n; }
};

}  // namespace osr
