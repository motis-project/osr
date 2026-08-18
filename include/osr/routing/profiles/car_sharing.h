#pragma once

#include <cassert>

#include <array>
#include <optional>
#include <string_view>
#include <type_traits>

#include "boost/json.hpp"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/entry_storage.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/tracking.h"
#include "osr/ways.h"

namespace osr {

template <typename Tracking = noop_tracking>
struct car_sharing {
  using footp = foot<false>;

  // initial foot -> rental
  static constexpr auto const kStartSwitchPenalty = cost_t{30U};
  // rental -> trailing foot
  static constexpr auto const kEndSwitchPenalty = cost_t{30U};

  static constexpr auto const kAdditionalWayProperties =
      way_properties{.is_foot_accessible_ = true,
                     .is_bike_accessible_ = true,
                     .is_car_accessible_ = true,
                     .is_destination_ = false,
                     .is_oneway_car_ = false,
                     .is_oneway_bike_ = false,
                     .is_elevator_ = false,
                     .is_steps_ = false,
                     .speed_limit_ = speed_limit::kmh_10,
                     .is_platform_ = 0,
                     .is_parking_ = false,
                     .is_ramp_ = false,
                     .is_sidewalk_separate_ = false,
                     .motor_vehicle_no_ = false,
                     .from_level_ = 0,
                     .has_toll_ = false,
                     .is_big_street_ = false,
                     .to_level_ = 0,
                     .is_bus_accessible_ = false,
                     .in_route_ = false,
                     .is_railway_accessible_ = false,
                     .is_oneway_bus_psv_ = false,
                     .is_incline_down_ = false,
                     .is_bus_accessible_with_penalty_ = false,
                     .is_ferry_accessible_ = false,
                     .is_railway_accessible_with_penalty_ = false,
                     .has_hgv_info_ = false,
                     .has_conditionals_ = false,
                     .is_in_low_emission_zone_ = false,
                     .is_detour_ = false,
                     .is_oneway_reverse_ = false};

  static constexpr auto const kAdditionalNodeProperties =
      node_properties{.from_level_ = 0,
                      .is_foot_accessible_ = true,
                      .is_bike_accessible_ = true,
                      .is_car_accessible_ = true,
                      .is_bus_accessible_ = true,
                      .is_elevator_ = false,
                      .is_entrance_ = false,
                      .is_multi_level_ = false,
                      .is_parking_ = false,
                      .to_level_ = 0,
                      .is_bus_accessible_with_penalty_ = false};

  enum class node_type : std::uint8_t {
    kInitialFoot,
    kRental,
    kTrailingFoot,
    kInvalid,
  };

  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kInitialFoot: return "initial_foot";
      case node_type::kRental: return "rental";
      case node_type::kTrailingFoot: return "trailing_foot";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }

  struct parameters {
    using profile_t = car_sharing<Tracking>;
    car::parameters car_{};
    footp::parameters foot_{};
  };

  struct key {
    friend constexpr bool operator==(key const&, key const&) = default;

    node_idx_t n_{node_idx_t::invalid()};
    level_t lvl_{};
  };

  using hash = footp::hash;

  struct node {
    friend bool operator==(node const a, node const b) {
      return a.n_ == b.n_ && a.type_ == b.type_ && a.dir_ == b.dir_ &&
             a.way_ == b.way_ && a.lvl_ == b.lvl_;
    }

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return std::tie(a.n_, a.type_, a.lvl_, a.way_, a.dir_) <
             std::tie(b.n_, b.type_, b.lvl_, b.way_, b.dir_);
    }

    boost::json::object geojson_properties(ways const& w) const {
      auto properties =
          boost::json::object{{"osm_node_id", to_idx(w.node_to_osm_[n_])},
                              {"level", lvl_.to_float()},
                              {"type", node_type_to_str(type_)}};
      if (is_rental_node()) {
        properties.emplace("direction", to_str(dir_));
      }
      return properties;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node="
                 << (n_ >= w.n_nodes() ? osm_node_idx_t{to_idx(n_)}
                                       : w.node_to_osm_[n_])
                 << (n_ >= w.n_nodes() ? "*" : "") << " (" << n_ << ")"
                 << ", level=" << lvl_ << ", dir=" << dir_
                 << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                 << " (" << static_cast<unsigned>(way_) << ")"
                 << ", type=" << node_type_to_str(type_) << ")";
    }

    static constexpr node invalid() noexcept { return {}; }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr key get_key() const noexcept { return {n_, lvl_}; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

    constexpr mode get_mode() const noexcept {
      return is_rental_node() ? mode::kCar : mode::kFoot;
    }

    constexpr bool is_initial_foot_node() const noexcept {
      return type_ == node_type::kInitialFoot;
    }

    constexpr bool is_rental_node() const noexcept {
      return type_ == node_type::kRental;
    }

    constexpr bool is_trailing_foot_node() const noexcept {
      return type_ == node_type::kTrailingFoot;
    }

    constexpr bool is_invalid_node() const noexcept {
      return type_ == node_type::kInvalid;
    }

    constexpr bool is_additional_node(
        sharing_data const* sharing) const noexcept {
      return to_idx(n_) >= sharing->additional_node_offset_;
    }

    node_idx_t n_{node_idx_t::invalid()};
    node_type type_{node_type::kInvalid};
    level_t lvl_{};
    direction dir_{direction::kForward};
    way_pos_t way_{};
  };

  struct label {
    constexpr label(node const n, cost_t const c)
        : n_{n.n_},
          cost_{c},
          type_{n.type_},
          lvl_{n.lvl_},
          dir_{n.dir_},
          way_{n.way_} {}

    constexpr node get_node() const noexcept {
      return {
          .n_ = n_, .type_ = type_, .lvl_ = lvl_, .dir_ = dir_, .way_ = way_};
    }

    constexpr cost_t cost() const noexcept { return cost_; }

    void track(label const& l,
               ways::routing const& r,
               way_idx_t const w,
               node_idx_t const n,
               bool const track) {
      tracking_.track(l.tracking_, r, w, n, track);
    }

    node_idx_t n_;
    cost_t cost_;
    node_type type_;
    level_t lvl_;
    direction dir_;
    way_pos_t way_;
    OSR_NO_UNIQUE_ADDRESS Tracking tracking_{};
  };

  struct slot {
    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    level_t pred_lvl_{kNoLevel};
    std::uint8_t pred_way_ : 5 {0U};
    std::uint8_t pred_dir_ : 1 {0U};
    std::uint8_t pred_type_
        : 2 {static_cast<std::uint8_t>(node_type::kInvalid)};
    duration_t duration_{kMaxDuration};
    OSR_NO_UNIQUE_ADDRESS Tracking tracking_{};

    constexpr cost_and_duration cd() const noexcept {
      return {.cost_ = cost_, .duration_ = duration_};
    }
  };

  struct entry {
    using slot_t = slot;

    using storage_t =
        entry_storage<slot_t, 2U>;  // 2 extra: initial + trailing foot
    static constexpr auto const kN = storage_t::kN;

    std::optional<node> pred(node const n) const noexcept {
      auto const s = s_[get_index(n)];
      return s.pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{
                       node{.n_ = s.pred_,
                            .type_ = static_cast<node_type>(s.pred_type_),
                            .lvl_ = s.pred_lvl_,
                            .dir_ = to_dir(s.pred_dir_),
                            .way_ = s.pred_way_}};
    }

    cost_t cost(node const n) const noexcept { return s_[get_index(n)].cost_; }

    duration_t duration(node const n) const noexcept {
      return s_[get_index(n)].duration_;
    }

    bool update(label const& l,
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
      s.pred_way_ = pred.way_;
      s.pred_dir_ = to_bool(pred.dir_);
      s.pred_type_ = static_cast<std::uint8_t>(pred.type_);
      s.tracking_ = l.tracking_;
      return true;
    }

    static constexpr std::size_t get_index(node const n) {
      switch (n.type_) {
        case node_type::kInitialFoot: return 0U;
        case node_type::kTrailingFoot: return 1U;
        default:  // node_type::kRental
          return storage_t::index(n.way_, n.dir_);
      }
    }

    static constexpr direction to_dir(bool const b) {
      return b ? direction::kBackward : direction::kForward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kBackward;
    }

    void write(node const n, path& p) const {
      s_[get_index(n)].tracking_.write(p);
    }

    storage_t s_;
  };

  static footp::node to_foot(node const n) {
    return {.n_ = n.n_, .lvl_ = n.lvl_};
  }

  static car::node to_rental(node const n) {
    return {.n_ = n.n_, .way_ = n.way_, .dir_ = n.dir_};
  }

  static node to_node(footp::node const n, node_type const type) {
    return {.n_ = n.n_, .type_ = type, .lvl_ = n.lvl_};
  }

  static node to_node(car::node const n, level_t const lvl) {
    return {.n_ = n.n_,
            .type_ = node_type::kRental,
            .lvl_ = lvl,
            .dir_ = n.dir_,
            .way_ = n.way_};
  }

  static node create_node(node_idx_t const n,
                          level_t const lvl,
                          way_pos_t const way,
                          direction const dir) {
    return node{n, node_type::kInvalid, lvl, dir, way};
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t lvl,
                                 direction search_dir,
                                 Fn&& f) {
    footp::resolve_start_node(w, way, n, lvl, search_dir,
                              [&](footp::node const fn) {
                                f(to_node(fn, search_dir == direction::kForward
                                                  ? node_type::kInitialFoot
                                                  : node_type::kTrailingFoot));
                              });
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    footp::resolve_all(w, n, lvl, [&](footp::node const neighbor) {
      f(to_node(neighbor, node_type::kInitialFoot));
      f(to_node(neighbor, node_type::kTrailingFoot));
      f(to_node(neighbor, node_type::kRental));
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
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       Fn&& fn) {
    assert(sharing != nullptr);

    auto const& handle_additional_edge = [&](additional_edge const& ae,
                                             node_type const nt,
                                             cost_and_duration const edge) {
      auto const emit = [&](node const target) {
        fn(target, edge.cost_, edge.duration_, ae.distance_,
           way_idx_t::invalid(), 0, 1, elevation_storage::elevation{}, false);
      };
      if (nt == node_type::kRental && !sharing->is_additional_node(ae.to_)) {
        // Picking the vehicle up at a station leaves it without an incoming
        // way, so every way/direction state has to be offered - otherwise the
        // first driving edge is charged a turn against a fabricated way that
        // the opposite search direction never pays.
        car::resolve_all(w, ae.to_, kNoLevel, [&](car::node const rental) {
          emit(to_node(rental, kNoLevel));
        });
        return;
      }
      emit(node{.n_ = ae.to_,
                .type_ = nt,
                .lvl_ = nt == node_type::kRental ? kNoLevel : n.lvl_});
    };

    auto const& continue_on_foot = [&](node_type const nt,
                                       bool const include_additional_edges) {
      footp::template adjacent<SearchDir, WithBlocked>(
          params.foot_, w, timezones, to_foot(n), current_duration, start_time,
          blocked, nullptr, elevations,
          [&](footp::node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t const dist,
              way_idx_t const way, std::uint16_t const from,
              std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            fn(to_node(neighbor, nt), cost, duration, dist, way, from, to,
               elevation, false);
          });
      if (include_additional_edges) {
        // walk to station or free-floating vehicle
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, nt,
                footp::way_cost(params.foot_, w, timezones,
                                way_idx_t::invalid(), kAdditionalWayProperties,
                                direction::kForward, ae.distance_, start_time,
                                current_duration, SearchDir));
          }
        }
      }
    };

    auto const& switch_mode = [&](node_type const to_type,
                                  bitvec<node_idx_t> const* allowed,
                                  cost_t const penalty) {
      if (!is_allowed(allowed, n.n_) ||
          car::node_cost(params.car_, w.node_properties_[n.n_]).cost_ ==
              kInfeasible) {
        return;
      }
      auto const emit = [&](node const target) {
        fn(target, penalty, duration_from_cost(penalty), distance_t{0U},
           way_idx_t::invalid(), 0U, 0U, elevation_storage::elevation{}, false);
      };
      if (to_type == node_type::kRental) {
        // foot -> vehicle: only from a level the vehicle side resolves back to
        if (!is_resolved_foot_state<footp>(
                w, n, [&](footp::node const foot_state) {
                  return to_node(foot_state, n.type_);
                })) {
          return;
        }
        car::resolve_all(w, n.n_, kNoLevel, [&](car::node const rental) {
          emit(to_node(rental, kNoLevel));
        });
      } else {
        footp::resolve_all(w, n.n_, kNoLevel,
                           [&](footp::node const foot_state) {
                             emit(to_node(foot_state, to_type));
                           });
      }
    };

    auto const& continue_with_vehicle = [&]() {
      car::adjacent<SearchDir, WithBlocked>(
          params.car_, w, timezones, to_rental(n), current_duration, start_time,
          blocked, nullptr, elevations,
          [&](car::node const neighbor, std::uint32_t const cost,
              duration_t const duration, distance_t const dist,
              way_idx_t const way, std::uint16_t const from,
              std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            // the forward orientation of this edge arrives at `neighbor`
            // going forward and at `n` going backward
            if (is_allowed(
                    sharing->through_allowed_,
                    SearchDir == direction::kForward ? neighbor.n_ : n.n_)) {
              fn(to_node(neighbor, kNoLevel), cost, duration, dist, way, from,
                 to, elevation, false);
            }
          });
      // drive to station
      if (auto const it = sharing->additional_edges_.find(n.n_);
          it != end(sharing->additional_edges_)) {
        for (auto const& ae : it->second) {
          handle_additional_edge(
              ae, node_type::kRental,
              car::way_cost(params.car_, w, timezones, way_idx_t::invalid(),
                            kAdditionalWayProperties, direction::kForward,
                            ae.distance_, start_time, current_duration,
                            SearchDir));
        }
      }
    };

    if (SearchDir == direction::kForward) {

      if (n.is_additional_node(sharing)) {
        // additional node - station or free-floating vehicle
        // switch mode and use additional edge
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            if (n.is_initial_foot_node() &&
                is_allowed(sharing->start_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kRental,
                  clamp_add(car::way_cost(
                                params.car_, w, timezones, way_idx_t::invalid(),
                                kAdditionalWayProperties, direction::kForward,
                                ae.distance_, start_time, current_duration,
                                SearchDir),
                            kStartSwitchPenalty));
            } else if (n.is_rental_node() &&
                       is_allowed(sharing->end_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kTrailingFoot,
                  clamp_add(footp::way_cost(
                                params.foot_, w, timezones,
                                way_idx_t::invalid(), kAdditionalWayProperties,
                                direction::kForward, ae.distance_, start_time,
                                current_duration, SearchDir),
                            kEndSwitchPenalty));
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_initial_foot_node());
          if (n.is_initial_foot_node()) {
            switch_mode(node_type::kRental, sharing->start_allowed_,
                        kStartSwitchPenalty);
          }
        } else if (n.is_rental_node()) {
          continue_with_vehicle();
          switch_mode(node_type::kTrailingFoot, sharing->end_allowed_,
                      kEndSwitchPenalty);
        }
      }

    } else /* backward */ {

      if (n.is_additional_node(sharing)) {
        // additional node - station or free-floating vehicle
        // switch mode and use additional edge
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            if (n.is_trailing_foot_node() &&
                is_allowed(sharing->end_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kRental,
                  clamp_add(car::way_cost(
                                params.car_, w, timezones, way_idx_t::invalid(),
                                kAdditionalWayProperties, direction::kForward,
                                ae.distance_, start_time, current_duration,
                                SearchDir),
                            kEndSwitchPenalty));
            } else if (n.is_rental_node() &&
                       is_allowed(sharing->start_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kInitialFoot,
                  clamp_add(footp::way_cost(
                                params.foot_, w, timezones,
                                way_idx_t::invalid(), kAdditionalWayProperties,
                                direction::kForward, ae.distance_, start_time,
                                current_duration, SearchDir),
                            kStartSwitchPenalty));
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_trailing_foot_node());
          if (n.is_trailing_foot_node()) {
            switch_mode(node_type::kRental, sharing->end_allowed_,
                        kEndSwitchPenalty);
          }
        } else if (n.is_rental_node()) {
          continue_with_vehicle();
          switch_mode(node_type::kInitialFoot, sharing->start_allowed_,
                      kStartSwitchPenalty);
        }
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
    return footp::is_dest_reachable(params.foot_, w, timezones, to_foot(n), way,
                                    way_dir, search_dir, start_time,
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

  static constexpr cost_and_duration node_cost(parameters const& params,
                                               node_properties const n) {
    return footp::node_cost(params.foot_, n);
  }

  template <endpoint_role Role, typename Fn>
  static void resolve_endpoint(ways::routing const& w,
                               way_idx_t const way,
                               node_idx_t const n,
                               level_t const lvl,
                               direction const search_dir,
                               Fn&& f) {
    footp::template resolve_endpoint<Role>(
        w, way, n, lvl, search_dir, [&](footp::node const resolved) {
          if constexpr (Role == endpoint_role::kSource) {
            f(to_node(resolved, search_dir == direction::kForward
                                    ? node_type::kInitialFoot
                                    : node_type::kTrailingFoot));
          } else {
            f(to_node(resolved, node_type::kInitialFoot));
            f(to_node(resolved, node_type::kTrailingFoot));
          }
        });
  }

  static cost_and_duration endpoint_way_cost(
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
    return footp::way_cost(params.foot_, w, timezones, way, properties, way_dir,
                           distance, start_time, current_duration, search_dir);
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

  static constexpr double lower_bound_heuristic(parameters const& params,
                                                double const dist) {
    return car::lower_bound_heuristic(params.car_, dist);
  }

  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return car::upper_bound_heuristic(params.car_, dist);
  }

  static constexpr node get_reverse(node const n) {
    return {.n_ = n.n_,
            .type_ = n.type_,
            .lvl_ = n.lvl_,
            .dir_ = opposite(n.dir_),
            .way_ = n.way_};
  }
};

}  // namespace osr
