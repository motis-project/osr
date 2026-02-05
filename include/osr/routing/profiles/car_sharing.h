#pragma once

#include <cassert>
#include <array>
#include <optional>
#include <string_view>
#include <type_traits>

#include "boost/json.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
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
                     .is_oneway_psv_ = false,
                     .is_incline_down_ = false,
                     .is_bus_accessible_with_penalty_ = false,
                     .is_ferry_accessible_ = false};

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
    car::parameters const car_{};
    footp::parameters const foot_{};
  };

  struct key {
    friend bool operator==(key const a, key const b) {
      auto const is_zero = [](level_t const l) {
        return l == kNoLevel || l == level_t{0.F};
      };
      return a.n_ == b.n_ &&
             (a.lvl_ == b.lvl_ || (is_zero(a.lvl_) && is_zero(b.lvl_)));
    }

    node_idx_t n_{node_idx_t::invalid()};
    level_t lvl_{};
  };

  using hash = footp::hash;

  struct node {
    friend bool operator==(node const a, node const b) {
      auto const is_zero = [](level_t const l) {
        return l == kNoLevel || l == level_t{0.F};
      };
      return a.n_ == b.n_ && a.type_ == b.type_ && a.dir_ == b.dir_ &&
             a.way_ == b.way_ &&
             (a.lvl_ == b.lvl_ || (is_zero(a.lvl_) && is_zero(b.lvl_)));
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
#ifdef _MSC_VER
    [[no_unique_address]] [[msvc::no_unique_address]] Tracking tracking_{};
#else
    [[no_unique_address]] Tracking tracking_{};
#endif
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN =
        kMaxWays * 2U + 2 /* FWD+BWD + initial foot + trailing foot */;

    entry() {
      utl::fill(pred_, node_idx_t::invalid());
      utl::fill(cost_, kInfeasible);
      utl::fill(pred_lvl_, kNoLevel);
      utl::fill(pred_way_, way_pos_t{0});
      utl::fill(pred_type_, node_type::kInvalid);
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{.n_ = pred_[idx],
                                      .type_ = pred_type_[idx],
                                      .lvl_ = pred_lvl_[idx],
                                      .dir_ = to_dir(pred_dir_[idx]),
                                      .way_ = pred_way_[idx]}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(label const& l,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_lvl_[idx] = pred.lvl_;
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        pred_type_[idx] = pred.type_;
        tracking_[idx] = l.tracking_;
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      switch (n.type_) {
        case node_type::kInitialFoot: return 0U;
        case node_type::kTrailingFoot: return 1U;
        default:  // node_type::kRental
          return 2U + (n.dir_ == direction::kForward ? 0U : kMaxWays) + n.way_;
      }
    }

    static constexpr direction to_dir(bool const b) {
      return b ? direction::kBackward : direction::kForward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kBackward;
    }

    void write(node const n, path& p) const {
      tracking_[get_index(n)].write(p);
    }

    std::array<node_idx_t, kN> pred_{};
    std::array<cost_t, kN> cost_{};
    std::array<level_t, kN> pred_lvl_{};
    std::array<way_pos_t, kN> pred_way_{};
    std::bitset<kN> pred_dir_{};
    std::array<node_type, kN> pred_type_{};
#ifdef _MSC_VER
    [[no_unique_address]] [[msvc::no_unique_address]] std::array<Tracking, kN>
        tracking_;
#else
    [[no_unique_address]] std::array<Tracking, kN> tracking_;
#endif
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
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       Fn&& fn) {
    assert(sharing != nullptr);

    auto const& handle_additional_edge =
        [&](additional_edge const& ae, node_type const nt, cost_t const cost) {
          fn(node{.n_ = ae.node_,
                  .type_ = nt,
                  .lvl_ = nt == node_type::kRental ? kNoLevel : n.lvl_},
             cost, ae.distance_, way_idx_t::invalid(), 0, 1,
             elevation_storage::elevation{}, false);
        };

    auto const& continue_on_foot = [&](node_type const nt,
                                       bool const include_additional_edges,
                                       cost_t const switch_penalty = 0) {
      footp::template adjacent<SearchDir, WithBlocked>(
          params.foot_, w, to_foot(n), blocked, nullptr, elevations,
          [&](footp::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            if (is_allowed(sharing->through_allowed_, neighbor.n_)) {
              fn(to_node(neighbor, nt), cost + switch_penalty, dist, way, from,
                 to, elevation, switch_penalty != 0);
            }
          });
      if (include_additional_edges) {
        // walk to station or free-floating vehicle
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, nt,
                footp::way_cost(params.foot_, kAdditionalWayProperties,
                                direction::kForward, ae.distance_) +
                    switch_penalty);
          }
        }
      }
    };

    auto const& continue_with_vehicle = [&](bool const include_additional_edges,
                                            cost_t const switch_penalty = 0) {
      car::adjacent<SearchDir, WithBlocked>(
          params.car_, w, to_rental(n), blocked, nullptr, elevations,
          [&](car::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            if (is_allowed(sharing->through_allowed_, neighbor.n_)) {
              fn(to_node(neighbor, kNoLevel), cost + switch_penalty, dist, way,
                 from, to, elevation, false);
            }
          });
      if (include_additional_edges) {
        // drive to station
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, node_type::kRental,
                car::way_cost(params.car_, kAdditionalWayProperties,
                              direction::kForward, ae.distance_) +
                    switch_penalty);
          }
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
                  car::way_cost(params.car_, kAdditionalWayProperties,
                                direction::kForward, ae.distance_) +
                      kStartSwitchPenalty);
            } else if (n.is_rental_node() &&
                       is_allowed(sharing->end_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kTrailingFoot,
                  footp::way_cost(params.foot_, kAdditionalWayProperties,
                                  direction::kForward, ae.distance_) +
                      kEndSwitchPenalty);
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_initial_foot_node());
          if (n.is_initial_foot_node() &&
              is_allowed(sharing->start_allowed_, n.n_)) {
            // switch to vehicle
            continue_with_vehicle(false, kStartSwitchPenalty);
          }
        } else if (n.is_rental_node()) {
          continue_with_vehicle(true);
          if (is_allowed(sharing->end_allowed_, n.n_)) {
            // switch to foot
            continue_on_foot(node_type::kTrailingFoot, false,
                             kEndSwitchPenalty);
          }
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
                  car::way_cost(params.car_, kAdditionalWayProperties,
                                direction::kForward, ae.distance_) +
                      kEndSwitchPenalty);
            } else if (n.is_rental_node() &&
                       is_allowed(sharing->start_allowed_, n.n_)) {
              handle_additional_edge(
                  ae, node_type::kInitialFoot,
                  footp::way_cost(params.foot_, kAdditionalWayProperties,
                                  direction::kForward, ae.distance_) +
                      kStartSwitchPenalty);
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_trailing_foot_node());
          if (n.is_trailing_foot_node() &&
              is_allowed(sharing->end_allowed_, n.n_)) {
            // switch to vehicle
            continue_with_vehicle(false, kEndSwitchPenalty);
          }
        } else if (n.is_rental_node()) {
          continue_with_vehicle(true);
          if (is_allowed(sharing->start_allowed_, n.n_)) {
            // switch to foot
            continue_on_foot(node_type::kInitialFoot, false, kEndSwitchPenalty);
          }
        }
      }
    }
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    return !n.is_rental_node() &&
           footp::is_dest_reachable(params.foot_, w, to_foot(n), way, way_dir,
                                    search_dir);
  }

  static constexpr cost_t way_cost(parameters const& params,
                                   way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    return footp::way_cost(params.foot_, e, dir, dist);
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return footp::node_cost(n);
  }

  static constexpr double heuristic(parameters const& params,
                                    double const dist) {
    return car::heuristic(params.car_, dist);
  }

  static constexpr double slow_heuristic(parameters const& params,
                                         double dist) {
    return car::slow_heuristic(params.car_, dist);
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
