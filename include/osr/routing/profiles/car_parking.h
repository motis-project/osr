#pragma once

#include <optional>

#include "boost/json.hpp"

#include "osr/elevation_storage.h"
#include "osr/routing/entry_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"
#include "utl/helpers/algorithm.h"

namespace osr {

struct sharing_data;

struct car_parking_slot {
  node_idx_t pred_{node_idx_t::invalid()};
  cost_t cost_{kInfeasible};
  level_t pred_lvl_{kNoLevel};
  std::uint8_t pred_way_ : 5 {0U};
  std::uint8_t pred_dir_ : 1 {0U};
  std::uint8_t pred_type_ : 1 {0U};
  duration_t duration_{kMaxDuration};

  constexpr cost_and_duration cd() const noexcept {
    return {.cost_ = cost_, .duration_ = duration_};
  }
};

template <bool IsWheelchair, bool UseParking = true>
struct car_parking {
  using footp = foot<IsWheelchair>;

  static constexpr auto const kSwitchPenalty = cost_t{200U};
  static constexpr auto const kMaxMatchDistance = car::kMaxMatchDistance;

  struct key {
    friend constexpr bool operator==(key const&, key const&) = default;

    node_idx_t n_{node_idx_t::invalid()};
    level_t lvl_{};
  };

  using hash = typename footp::hash;

  enum class node_type : std::uint8_t { kCar, kFoot, kInvalid };

  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kCar: return "car";
      case node_type::kFoot: return "foot";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }

  struct parameters {
    using profile_t = car_parking<IsWheelchair, UseParking>;
    car::parameters car_{};
    footp::parameters foot_{};
  };

  struct node {
    friend constexpr bool operator==(node const&, node const&) = default;

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return std::tie(a.n_, a.type_, a.lvl_, a.way_, a.dir_) <
             std::tie(b.n_, b.type_, b.lvl_, b.way_, b.dir_);
    }

    boost::json::object geojson_properties(ways const& w) const {
      auto properties =
          boost::json::object{{"osm_node_id", to_idx(w.node_to_osm_[n_])},
                              {"level", lvl_.to_float()},
                              {"type", node_type_to_str(type_)}};
      if (is_car_node()) {
        properties.emplace("direction", to_str(dir_));
      }
      return properties;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", level=" << lvl_
                 << ", dir=" << to_str(dir_)
                 << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                 << ", type=" << node_type_to_str(type_) << ")";
    }

    static constexpr node invalid() noexcept { return node{}; }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr key get_key() const noexcept { return {n_, lvl_}; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

    constexpr mode get_mode() const noexcept {
      return is_car_node() ? mode::kCar : mode::kFoot;
    }

    constexpr bool is_car_node() const noexcept {
      return type_ == node_type::kCar;
    }

    constexpr bool is_foot_node() const noexcept {
      return type_ == node_type::kFoot;
    }

    constexpr bool is_invalid_node() const noexcept {
      return type_ == node_type::kInvalid;
    }

    node_idx_t n_{node_idx_t::invalid()};
    node_type type_{node_type::kInvalid};
    level_t lvl_;
    direction dir_;
    way_pos_t way_;
  };

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_},
          cost_{c},
          type_{n.type_},
          lvl_{n.lvl_},
          dir_{n.dir_},
          way_(n.way_) {}

    constexpr node get_node() const noexcept {
      return {
          .n_ = n_, .type_ = type_, .lvl_ = lvl_, .dir_ = dir_, .way_ = way_};
    }

    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    cost_t cost_;
    node_type type_;
    level_t lvl_;
    direction dir_;
    way_pos_t way_;
  };

  struct entry {
    using slot_t = car_parking_slot;

    using storage_t = entry_storage<slot_t, 1U>;  // 1 extra for foot
    static constexpr auto const kN = storage_t::kN;

    std::optional<node> pred(node const n) const noexcept {
      auto const s = s_[get_index(n)];
      return s.pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{.n_ = s.pred_,
                                      .type_ = to_node_type(s.pred_type_),
                                      .lvl_ = s.pred_lvl_,
                                      .dir_ = to_dir(s.pred_dir_),
                                      .way_ = s.pred_way_}};
    }

    cost_t cost(node const n) const noexcept { return s_[get_index(n)].cost_; }

    duration_t duration(node const n) const noexcept {
      return s_[get_index(n)].duration_;
    }

    bool update(label const,
                node const n,
                cost_and_duration const c,
                node const pred,
                ways::routing const& w,
                entry_storage_arena& a) {
      auto& s = s_.slot(get_index(n), w, n.n_, a);
      if (c >= s.cd()) {
        return false;
      }
      s.cost_ = c.cost_;
      s.duration_ = c.duration_;
      s.pred_ = pred.n_;
      s.pred_lvl_ = pred.lvl_;
      s.pred_type_ = to_bool(pred.type_);
      s.pred_way_ = pred.way_;
      s.pred_dir_ = to_bool(pred.dir_);
      return true;
    }

    static constexpr std::size_t get_index(node const n) {
      return n.is_foot_node() ? 0U : storage_t::index(n.way_, n.dir_);
    }

    static constexpr direction to_dir(bool const b) {
      return b ? direction::kBackward : direction::kForward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kBackward;
    }

    static constexpr node_type to_node_type(bool const b) {
      return b ? node_type::kFoot : node_type::kCar;
    }

    static constexpr bool to_bool(node_type const t) {
      return t == node_type::kFoot;
    }

    void write(node, path&) const {}

    storage_t s_;
  };

  static car::node to_car(node const n) {
    return {.n_ = n.n_, .way_ = n.way_, .dir_ = n.dir_};
  }

  static footp::node to_foot(node const n) {
    return {.n_ = n.n_, .lvl_ = n.lvl_};
  }

  static node to_node(car::node const n) {
    return {.n_ = n.n_,
            .type_ = node_type::kCar,
            .lvl_ = kNoLevel,  // car nodes don't have levels
            .dir_ = n.dir_,
            .way_ = n.way_};
  }

  static node to_node(footp::node const n) {
    return {.n_ = n.n_,
            .type_ = node_type::kFoot,
            .lvl_ = n.lvl_,
            .dir_ = direction::kForward,
            .way_ = 0};
  }

  static node create_node(node_idx_t const n,
                          level_t const lvl,
                          way_pos_t const way,
                          direction const dir) {
    return node{n, node_type::kInvalid, lvl, dir, way};
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    footp::resolve_all(
        w, n, lvl, [&](footp::node const neighbor) { f(to_node(neighbor)); });
    car::resolve_all(w, n, lvl,
                     [&](car::node const neighbor) { f(to_node(neighbor)); });
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
                       elevation_storage const* elevations,
                       Fn&& fn) {
    if (n.is_foot_node()) {
      footp::template adjacent<SearchDir, WithBlocked>(
          params.foot_, w, timezones, to_foot(n), current_duration, start_time,
          blocked, nullptr, elevations,
          [&](footp::node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t const dist,
              way_idx_t const way, std::uint16_t const from,
              std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            fn(to_node(neighbor), cost, duration, dist, way, from, to,
               elevation, false);
          });
    } else {
      car::template adjacent<SearchDir, WithBlocked>(
          params.car_, w, timezones, to_car(n), current_duration, start_time,
          blocked, nullptr, elevations,
          [&](car::node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t const dist,
              way_idx_t const way, std::uint16_t const from,
              std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            fn(to_node(neighbor), cost, duration, dist, way, from, to,
               elevation, false);
          });
    }

    if (!can_leave_car(w, n.n_)) {
      return;
    }
    auto const leave_car = [&](node const target) {
      fn(target, kSwitchPenalty, duration_from_cost(kSwitchPenalty),
         distance_t{0U}, way_idx_t::invalid(), 0U, 0U,
         elevation_storage::elevation{}, false);
    };
    if constexpr (SearchDir == direction::kForward) {
      if (n.is_car_node()) {
        footp::resolve_all(w, n.n_, kNoLevel,
                           [&](footp::node const foot_state) {
                             leave_car(to_node(foot_state));
                           });
      }
    } else {
      if (n.is_foot_node() && is_resolved_foot_state<footp>(
                                  w, n, [&](footp::node const foot_state) {
                                    return to_node(foot_state);
                                  })) {
        car::resolve_all(w, n.n_, kNoLevel,
                         [&](car::node const cn) { leave_car(to_node(cn)); });
      }
    }
  }

  static bool can_leave_car(ways::routing const& w, node_idx_t const n) {
    return !UseParking || w.node_properties_[n].is_parking() ||
           utl::any_of(w.node_ways_[n], [&](way_idx_t const way) {
             return w.way_properties_[way].is_parking();
           });
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t lvl,
                                 direction search_dir,
                                 Fn&& f) {
    search_dir == direction::kForward
        ? car::resolve_start_node(w, way, n, lvl, search_dir,
                                  [&](car::node const cn) { f(to_node(cn)); })
        : footp::resolve_start_node(
              w, way, n, lvl, search_dir,
              [&](footp::node const fn) { f(to_node(fn)); });
  }

  template <endpoint_role Role, typename Fn>
  static void resolve_endpoint(ways::routing const& w,
                               way_idx_t const way,
                               node_idx_t const n,
                               level_t const lvl,
                               direction const search_dir,
                               Fn&& f) {
    if (search_dir == direction::kForward) {
      car::template resolve_endpoint<Role>(
          w, way, n, lvl, search_dir,
          [&](car::node const cn) { f(to_node(cn)); });
    } else {
      footp::template resolve_endpoint<Role>(
          w, way, n, lvl, search_dir,
          [&](typename footp::node const fn) { f(to_node(fn)); });
    }
  }

  static cost_and_duration endpoint_transition_cost(
      parameters const& params,
      ways::routing const& w,
      timezone_cache_t const& timezones,
      node const n,
      way_idx_t const way,
      direction const way_dir,
      direction const search_dir,
      std::optional<routing_time_t> const start_time,
      duration_t const current_duration) {
    return n.is_car_node()
               ? car::endpoint_transition_cost(
                     params.car_, w, timezones, to_car(n), way, way_dir,
                     search_dir, start_time, current_duration)
               : cost_and_duration{};
  }

  static constexpr bool endpoint_root_allowed(parameters const& params,
                                              node const n,
                                              direction const way_dir) {
    return !n.is_car_node() ||
           car::endpoint_root_allowed(params.car_, to_car(n), way_dir);
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
    return n.is_foot_node()
               ? footp::is_dest_reachable(params.foot_, w, timezones,
                                          to_foot(n), way, way_dir, search_dir,
                                          start_time, current_duration)
               : car::is_dest_reachable(params.car_, w, timezones, to_car(n),
                                        way, way_dir, search_dir, start_time,
                                        current_duration);
  }

  static constexpr cost_and_duration way_cost(
      parameters const& params,
      ways::routing const& w,
      timezone_cache_t const& timezones,
      way_idx_t const way,
      way_properties const& e,
      direction const dir,
      distance_t const dist,
      std::optional<routing_time_t> const start_time,
      duration_t const current_duration,
      direction const search_dir) {
    return footp::way_cost(params.foot_, w, timezones, way, e, dir, dist,
                           start_time, current_duration, search_dir);
  }

  static constexpr cost_and_duration endpoint_way_cost(
      parameters const& params,
      ways::routing const& w,
      timezone_cache_t const& timezones,
      node const n,
      way_idx_t const way,
      way_properties const& properties,
      direction const way_dir,
      distance_t const distance,
      std::optional<routing_time_t> const start_time,
      duration_t const current_duration,
      direction const search_dir) {
    return n.is_car_node()
               ? car::way_cost(params.car_, w, timezones, way, properties,
                               way_dir, distance, start_time, current_duration,
                               search_dir)
               : footp::way_cost(params.foot_, w, timezones, way, properties,
                                 way_dir, distance, start_time,
                                 current_duration, search_dir);
  }

  static constexpr cost_and_duration node_cost(parameters const& params,
                                               node_properties const n) {
    return footp::node_cost(params.foot_, n);
  }

  static constexpr double lower_bound_heuristic(parameters const& params,
                                                double const dist) {
    return car::lower_bound_heuristic(params.car_, dist);
  }

  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return car::upper_bound_heuristic(params.car_, dist);
  }

  static constexpr node get_reverse(node n) {
    return {n.n_, n.type_, n.lvl_, opposite(n.dir_), n.way_};
  }
};

}  // namespace osr
