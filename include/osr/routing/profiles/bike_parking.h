#pragma once

#include <cassert>
#include <array>
#include <optional>
#include <string_view>

#include "boost/json.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/additional_edge.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

namespace osr {

struct bike_parking {
  using footp = foot<false>;
  using bikep = bike<bike_costing::kSafe, kElevationNoCost>;

  static constexpr auto const kSwitchPenalty = cost_t{200U};
  static constexpr auto const kMaxMatchDistance = bikep::kMaxMatchDistance;

  static constexpr auto const kAdditionalWayProperties =
      way_properties{.is_foot_accessible_ = true,
                     .is_bike_accessible_ = true,
                     .is_car_accessible_ = false,
                     .is_destination_ = false,
                     .is_oneway_car_ = false,
                     .is_oneway_bike_ = false,
                     .is_elevator_ = false,
                     .is_steps_ = false,
                     .speed_limit_ = 0,
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
                     .is_railway_accessible_with_penalty_ = false};

  enum class node_type : std::uint8_t { kBike, kFoot, kInvalid };

  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kBike: return "bike";
      case node_type::kFoot: return "foot";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }

  struct parameters {
    using profile_t = bike_parking;
    bikep::parameters const bike_{};
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
             (a.lvl_ == b.lvl_ || (is_zero(a.lvl_) && is_zero(b.lvl_)));
    }

    friend constexpr bool operator<(node const& a, node const& b) noexcept {
      return std::tie(a.n_, a.type_, a.lvl_, a.dir_) <
             std::tie(b.n_, b.type_, b.lvl_, b.dir_);
    }

    boost::json::object geojson_properties(ways const& w) const {
      auto properties = boost::json::object{
          {"osm_node_id", to_idx(n_ >= w.n_nodes() ? osm_node_idx_t{to_idx(n_)}
                                                   : w.node_to_osm_[n_])},
          {"level", lvl_.to_float()},
          {"type", node_type_to_str(type_)}};
      if (is_bike_node()) {
        properties.emplace("direction", to_str(dir_));
      }
      return properties;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node="
                 << (n_ >= w.n_nodes() ? osm_node_idx_t{to_idx(n_)}
                                       : w.node_to_osm_[n_])
                 << (n_ >= w.n_nodes() ? "*" : "") << ", level=" << lvl_
                 << ", dir=" << to_str(dir_)
                 << ", type=" << node_type_to_str(type_) << ")";
    }

    static constexpr node invalid() noexcept { return {}; }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr key get_key() const noexcept { return {n_, lvl_}; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

    constexpr mode get_mode() const noexcept {
      return is_bike_node() ? mode::kBike : mode::kFoot;
    }

    constexpr bool is_bike_node() const noexcept {
      return type_ == node_type::kBike;
    }

    constexpr bool is_foot_node() const noexcept {
      return type_ == node_type::kFoot;
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
  };

  struct label {
    constexpr label(node const n, cost_t const c)
        : n_{n.n_}, cost_{c}, type_{n.type_}, lvl_{n.lvl_}, dir_{n.dir_} {}

    constexpr node get_node() const noexcept {
      return {.n_ = n_, .type_ = type_, .lvl_ = lvl_, .dir_ = dir_};
    }

    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    cost_t cost_;
    node_type type_;
    level_t lvl_;
    direction dir_;
  };

  struct entry {
    static constexpr auto const kN = 3U;  // foot + bike fwd + bike bwd

    entry() {
      utl::fill(pred_, node_idx_t::invalid());
      utl::fill(cost_, kInfeasible);
      utl::fill(pred_lvl_, kNoLevel);
      utl::fill(pred_type_, node_type::kInvalid);
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{.n_ = pred_[idx],
                                      .type_ = pred_type_[idx],
                                      .lvl_ = pred_lvl_[idx],
                                      .dir_ = pred_dir_[idx]}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(label const,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.n_;
        pred_lvl_[idx] = pred.lvl_;
        pred_type_[idx] = pred.type_;
        pred_dir_[idx] = pred.dir_;
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      return n.is_foot_node() ? 0U
                              : 1U + (n.dir_ == direction::kForward ? 0U : 1U);
    }

    void write(node, path&) const {}

    std::array<node_idx_t, kN> pred_{};
    std::array<cost_t, kN> cost_{};
    std::array<level_t, kN> pred_lvl_{};
    std::array<node_type, kN> pred_type_{};
    std::array<direction, kN> pred_dir_{};
  };

  static footp::node to_foot(node const n) {
    return {.n_ = n.n_, .lvl_ = n.lvl_};
  }

  static bikep::node to_bike(node const n) {
    return {.n_ = n.n_, .dir_ = n.dir_};
  }

  static node to_node(footp::node const n) {
    return {.n_ = n.n_,
            .type_ = node_type::kFoot,
            .lvl_ = n.lvl_,
            .dir_ = direction::kForward};
  }

  static node to_node(bikep::node const n, level_t const lvl) {
    return {.n_ = n.n_, .type_ = node_type::kBike, .lvl_ = lvl, .dir_ = n.dir_};
  }

  static node create_node(node_idx_t const n,
                          level_t const lvl,
                          way_pos_t const,
                          direction const dir) {
    return node{n, node_type::kInvalid, lvl, dir};
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t lvl,
                                 direction search_dir,
                                 Fn&& f) {
    search_dir == direction::kForward
        ? bikep::resolve_start_node(
              w, way, n, lvl, search_dir,
              [&](bikep::node const bn) { f(to_node(bn, lvl)); })
        : footp::resolve_start_node(
              w, way, n, lvl, search_dir,
              [&](footp::node const fn) { f(to_node(fn)); });
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    footp::resolve_all(
        w, n, lvl, [&](footp::node const neighbor) { f(to_node(neighbor)); });
    bikep::resolve_all(w, n, lvl, [&](bikep::node const neighbor) {
      f(to_node(neighbor, lvl));
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

    static constexpr auto const kFwd = SearchDir == direction::kForward;
    static constexpr auto const kBwd = SearchDir == direction::kBackward;

    auto const is_parking = is_allowed(sharing->end_allowed_, n.n_);

    auto const handle_additional_edge =
        [&](additional_edge const& ae, node_type const nt, cost_t const cost) {
          fn(node{.n_ = ae.to_,
                  .type_ = nt,
                  .lvl_ = nt == node_type::kBike ? kNoLevel : n.lvl_,
                  .dir_ = direction::kForward},
             cost, ae.distance_, way_idx_t::invalid(), 0, 1,
             elevation_storage::elevation{}, false);
        };

    auto const continue_on_foot = [&](bool const include_additional_edges,
                                      cost_t const switch_penalty) {
      if (!n.is_additional_node(sharing)) {
        footp::template adjacent<SearchDir, WithBlocked>(
            params.foot_, w, to_foot(n), blocked, nullptr, elevations,
            [&](footp::node const neighbor, std::uint32_t const cost,
                distance_t const dist, way_idx_t const way,
                std::uint16_t const from, std::uint16_t const to,
                elevation_storage::elevation const elevation, bool) {
              if (is_allowed(sharing->through_allowed_, neighbor.n_)) {
                fn(to_node(neighbor), cost + switch_penalty, dist, way, from,
                   to, elevation, false);
              }
            });
      }
      if (include_additional_edges) {
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, node_type::kFoot,
                footp::way_cost(params.foot_, kAdditionalWayProperties,
                                direction::kForward, ae.distance_) +
                    switch_penalty);
          }
        }
      }
    };

    auto const continue_on_bike = [&](bool const include_additional_edges,
                                      cost_t const switch_penalty) {
      if (!n.is_additional_node(sharing)) {
        bikep::template adjacent<SearchDir, WithBlocked>(
            params.bike_, w, to_bike(n), blocked, nullptr, elevations,
            [&](bikep::node const neighbor, std::uint32_t const cost,
                distance_t const dist, way_idx_t const way,
                std::uint16_t const from, std::uint16_t const to,
                elevation_storage::elevation const elevation, bool) {
              if (is_allowed(sharing->through_allowed_, neighbor.n_)) {
                fn(to_node(neighbor, kNoLevel), cost + switch_penalty, dist,
                   way, from, to, elevation, false);
              }
            });
      }
      if (include_additional_edges) {
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, node_type::kBike,
                bikep::way_cost(params.bike_, kAdditionalWayProperties,
                                direction::kForward, ae.distance_) +
                    switch_penalty);
          }
        }
      }
    };

    if (n.is_foot_node() || (kFwd && n.is_bike_node() && is_parking)) {
      continue_on_foot(true, n.is_foot_node() ? 0 : kSwitchPenalty);
    }

    if (n.is_bike_node() || (kBwd && n.is_foot_node() && is_parking)) {
      continue_on_bike(true, n.is_bike_node() ? 0 : kSwitchPenalty);
    }
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    return search_dir == direction::kForward
               ? n.is_foot_node() &&
                     footp::is_dest_reachable(params.foot_, w, to_foot(n), way,
                                              way_dir, search_dir)
               : n.is_bike_node() &&
                     bikep::is_dest_reachable(params.bike_, w, to_bike(n), way,
                                              way_dir, search_dir);
  }

  static constexpr cost_t way_cost(parameters const& params,
                                   way_properties const& e,
                                   direction const dir,
                                   distance_t const dist) {
    return footp::way_cost(params.foot_, e, dir, dist);
  }

  static constexpr cost_t node_cost(parameters const& params,
                                    node_properties const n) {
    return footp::node_cost(params.foot_, n);
  }

  static constexpr double lower_bound_heuristic(parameters const& params,
                                                double const dist) {
    return bikep::lower_bound_heuristic(params.bike_, dist);
  }

  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return bikep::upper_bound_heuristic(params.bike_, dist);
  }

  static constexpr node get_reverse(node const n) {
    return {n.n_, n.type_, n.lvl_, opposite(n.dir_)};
  }
};

}  // namespace osr
