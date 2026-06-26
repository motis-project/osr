#pragma once

#include <cmath>

#include <algorithm>
#include <bitset>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>

#include "boost/json/object.hpp"
#include "boost/math/ccmath/ccmath.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/conditional.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profiles/common.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/turns.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

struct hgv {
  static constexpr auto const kName = "hgv";
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kExactBidirectional = true;
  static constexpr auto const kDetourCostFactor = 0.9F;

  using key = node_idx_t;

  struct parameters {
    using profile_t = hgv;
    cost_t uturn_penalty_{2000U};
    quantized_angle_t slow_turn_angle_{quantize_turn_angle(65.0)};
    quantized_angle_t sharp_turn_angle_{quantize_turn_angle(110.0)};
    cost_t slow_turn_penalty_{10U};
    cost_t sharp_turn_penalty_{25U};
    std::uint16_t height_cm_{400U};
    std::uint16_t width_cm_{255U};
    std::uint16_t length_cm_{1880U};
    std::uint16_t weight_100kg_{400U};
    bool hazmat_{false};
    bool hazmat_water_{false};
    std::uint8_t axle_count_{5U};
    std::uint16_t axle_load_100kg_{115U};
    bool trailer_{true};
    std::uint8_t top_speed_km_h_{80U};
    bool low_emission_zone_access_{true};
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
      return boost::json::object{{"node_id", n_.v_}, {"type", "hgv"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    static constexpr mode get_mode() noexcept { return mode::kHgv; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

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

    entry() {
      utl::fill(pred_, node_idx_t::invalid());
      utl::fill(cost_, kInfeasible);
      utl::fill(duration_, kMaxDuration);
    }

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

    constexpr duration_t duration(node const n) const noexcept {
      return duration_[get_index(n)];
    }

    constexpr bool update(label const& l,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      return update(l, n, c, pred, duration_from_cost(c));
    }

    constexpr bool update(label const&,
                          node const n,
                          cost_t const c,
                          node const pred,
                          duration_t const duration) noexcept {
      n_ = n.n_;
      auto const idx = get_index(n);
      if (c < cost_[idx] || (c == cost_[idx] && duration < duration_[idx])) {
        cost_[idx] = c;
        duration_[idx] = duration;
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
    std::array<duration_t, kN> duration_;
    node_idx_t n_{node_idx_t::invalid()};
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  struct way_state {
    bool accessible_{};
    bool destination_penalty_{};
    bool designated_preference_{};
    access_value hgv_access_{access_value::kUnknown};
    hgv_way_info const* info_{};
    std::optional<std::uint16_t> max_speed_km_h_{};
    std::optional<conditional_oneway_value> oneway_{};
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
                       duration_t const current_duration,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* additional,
                       elevation_storage const*,
                       Fn&& fn) {
    adjacent<SearchDir, WithBlocked>(params, w, n, current_duration,
                                     std::nullopt, blocked, additional, nullptr,
                                     std::forward<Fn>(fn));
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(parameters const& params,
                       ways::routing const& w,
                       node const n,
                       duration_t const current_duration,
                       std::optional<routing_time_t> const start_time,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* additional,
                       elevation_storage const*,
                       Fn&& fn) {
    if (additional != nullptr) {
      for_each_additional_edge<hgv>(
          params, w, n, additional, start_time, current_duration, SearchDir,
          [&](additional_edge const& ae, cost_and_duration const edge_cost,
              direction const edge_dir) {
            if (!additional->is_additional_node(n.n_)) {
              if (is_restricted<SearchDir>(
                      params, w, n.n_, n.way_,
                      w.get_way_pos(n.n_, ae.underlying_way_), start_time,
                      current_duration, SearchDir)) {
                return;
              }
            }

            auto const [target, cost, duration] =
                get_adjacent_additional_node<hgv>(params, w, n, additional, ae,
                                                  edge_dir, edge_cost,
                                                  params.uturn_penalty_);
            if (cost == kInfeasible) {
              return;
            }

            fn(target, cost, duration, ae.distance_, ae.underlying_way_, 0, 0,
               elevation_storage::elevation{}, false);
          });

      if (additional->is_additional_node(n.n_)) {
        return;
      }
    }

    for_each_adjacent_node<hgv, SearchDir, WithBlocked, true>(
        params, w, n, blocked, params.uturn_penalty_, start_time,
        current_duration, SearchDir, fn);
  }

  template <direction SearchDir>
  static bool is_restricted(parameters const& params,
                            ways::routing const& w,
                            node_idx_t const n,
                            std::uint8_t const from,
                            std::uint8_t const to,
                            std::optional<routing_time_t> const start_time,
                            duration_t const current_duration,
                            direction const search_dir) {
    return is_restricted(params, w, n, from, to, SearchDir, start_time,
                         current_duration, search_dir);
  }

  static bool is_restricted(parameters const& params,
                            ways::routing const& w,
                            node_idx_t const n,
                            std::uint8_t const from,
                            std::uint8_t const to,
                            direction const restriction_dir,
                            std::optional<routing_time_t> const start_time,
                            duration_t const current_duration,
                            direction const search_dir) {
    if (!w.node_is_restricted_[n]) {
      return false;
    }

    auto const from_way = restriction_dir == direction::kForward
                              ? way_pos_t{from}
                              : way_pos_t{to};
    auto const to_way = restriction_dir == direction::kForward
                            ? way_pos_t{to}
                            : way_pos_t{from};
    auto const current_time =
        current_routing_time(start_time, search_dir, current_duration);
    return utl::any_of(w.node_restrictions_[n], [&](restriction const& x) {
      if (x.from_ != from_way || x.to_ != to_way || !x.applies_to_hgv_) {
        return false;
      }
      return x.condition_set_ == conditional_condition_set_idx_t::invalid() ||
             matches_profile_condition_set_utc(params, w, x.condition_set_,
                                               current_time);
    });
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir,
                                std::optional<routing_time_t> const start_time,
                                duration_t const current_duration) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(params, w, way, target_way_prop, way_dir, 0U, start_time,
                 current_duration, search_dir)
            .cost_ == kInfeasible) {
      return false;
    }

    if (is_restricted(params, w, n.n_, n.way_, w.get_way_pos(n.n_, way),
                      search_dir, start_time, current_duration, search_dir)) {
      return false;
    }

    return true;
  }

  static cost_and_duration way_cost(
      parameters const& params,
      ways::routing const& w,
      way_idx_t const way,
      way_properties const& e,
      direction const dir,
      distance_t const dist,
      std::optional<routing_time_t> const start_time,
      duration_t const current_duration,
      direction const search_dir) {
    if (!params.low_emission_zone_access_ && e.is_in_low_emission_zone()) {
      return infeasible_cost_and_duration();
    }

    auto accessible = e.is_car_accessible();
    auto destination_penalty = e.is_destination();
    auto state = way_state{.accessible_ = accessible,
                           .destination_penalty_ = destination_penalty,
                           .info_ = w.get_hgv_info(way),
                           .oneway_ = static_oneway_value(e)};

    if (auto const* info = state.info_; info != nullptr) {
      if (info->has(hgv_info_field::kAccess)) {
        state.hgv_access_ = info->hgv_access();
        state.accessible_ = access_allowed(state.hgv_access_);
        state.destination_penalty_ = false;
      }
      if (!fits_vehicle(params, *info)) {
        return infeasible_cost_and_duration();
      }
      apply_static_access_preferences(params, *info, state);
    }

    apply_conditional_restrictions(
        params, w, way, dir,
        current_routing_time(start_time, search_dir, current_duration), state);

    if (!conditional_oneway_accessible(state.oneway_, dir)) {
      return infeasible_cost_and_duration();
    }

    if (!state.accessible_) {
      return infeasible_cost_and_duration();
    }
    return {.cost_ = distance_cost(params, e, state.info_, dist,
                                   state.destination_penalty_,
                                   state.designated_preference_,
                                   state.hgv_access_, state.max_speed_km_h_),
            .duration_ = distance_duration(params, e, state.info_, dist,
                                           state.max_speed_km_h_)};
  }

  static constexpr cost_and_duration node_cost(parameters const&,
                                               node_properties const& n) {
    return n.is_car_accessible() ? cost_and_duration_from_cost(0U)
                                 : infeasible_cost_and_duration();
  }

  static constexpr cost_t turn_cost(parameters const& params,
                                    quantized_angle_t const turn_angle) {
    auto cost = cost_t{0U};
    if (turn_angle > params.slow_turn_angle_) {
      cost += params.slow_turn_penalty_;
    }
    if (turn_angle > params.sharp_turn_angle_) {
      cost += params.sharp_turn_penalty_;
    }
    return cost;
  }

  static double lower_bound_heuristic(parameters const& params,
                                      double const dist) {
    return (3.6 / static_cast<double>(params.top_speed_km_h_)) * dist;
  }

  static constexpr double upper_bound_heuristic(parameters const&,
                                                double const dist) {
    return (3.6 / 15U) * dist;
  }

  static constexpr node get_reverse(node const n) {
    return {n.n_, n.way_, opposite(n.dir_)};
  }

  static std::uint32_t vehicle_property(parameters const& params,
                                        conditional_vehicle_property const p) {
    switch (p) {
      case conditional_vehicle_property::kWeight: return params.weight_100kg_;
      case conditional_vehicle_property::kWeightRating:
        return params.weight_100kg_;
      case conditional_vehicle_property::kLength: return params.length_cm_;
      case conditional_vehicle_property::kWidth: return params.width_cm_;
      case conditional_vehicle_property::kHeight: return params.height_cm_;
      case conditional_vehicle_property::kAxleLoad:
        return params.axle_load_100kg_;
      case conditional_vehicle_property::kAxles: return params.axle_count_;
    }
    return 0U;
  }

  static bool matches_profile_condition(parameters const& params,
                                        conditional_condition const& c) {
    switch (c.type_) {
      case conditional_condition_type::kOpeningHours: return false;
      case conditional_condition_type::kVehicleProperty:
        return compare_conditional_value(
            vehicle_property(
                params, static_cast<conditional_vehicle_property>(c.selector_)),
            c.comparison_, c.value_.value_);
      case conditional_condition_type::kAccessPurpose: return false;
      case conditional_condition_type::kVehicleUsage: {
        auto const value =
            static_cast<conditional_symbolic_condition>(c.selector_);
        return (value == conditional_symbolic_condition::kHazmat &&
                (params.hazmat_ || params.hazmat_water_)) ||
               (value == conditional_symbolic_condition::kHazmatWater &&
                params.hazmat_water_) ||
               (value == conditional_symbolic_condition::kTrailer &&
                params.trailer_);
      }
    }
    return false;
  }

  static bool matches_profile_condition_set(
      parameters const& params,
      ways::routing const& w,
      conditional_condition_set_idx_t const idx,
      std::optional<conditional_wall_time> const& t) {
    return matches_conditional_condition_set(
        w, idx, t, [&](conditional_condition const& c) {
          return matches_profile_condition(params, c);
        });
  }

  static bool matches_profile_condition_set_utc(
      parameters const& params,
      ways::routing const& w,
      conditional_condition_set_idx_t const idx,
      std::optional<routing_time_t> const t) {
    return matches_conditional_condition_set_utc(
        w, idx, t, [&](conditional_condition const& c) {
          return matches_profile_condition(params, c);
        });
  }

  static bool mode_applies(conditional_transport_mode const mode) {
    switch (mode) {
      case conditional_transport_mode::kUnspecified: [[fallthrough]];
      case conditional_transport_mode::kAccess: [[fallthrough]];
      case conditional_transport_mode::kVehicle: [[fallthrough]];
      case conditional_transport_mode::kMotorVehicle: [[fallthrough]];
      case conditional_transport_mode::kHgv: return true;
      default: return false;
    }
  }

  static bool is_forbidden(access_value const value) {
    switch (value) {
      case access_value::kNo: [[fallthrough]];
      case access_value::kPrivate: [[fallthrough]];
      case access_value::kDiscouraged: return true;
      default: return false;
    }
  }

  static bool is_destination_like(access_value const value) {
    switch (value) {
      case access_value::kDestination: [[fallthrough]];
      case access_value::kDelivery: return true;
      default: return false;
    }
  }

  static bool is_designated(access_value const value) {
    return value == access_value::kDesignated;
  }

  static constexpr bool conditional_oneway_accessible(
      std::optional<conditional_oneway_value> const value,
      direction const dir) {
    if (!value.has_value() || *value == conditional_oneway_value::kNo) {
      return true;
    }
    return (*value == conditional_oneway_value::kForward &&
            dir == direction::kForward) ||
           (*value == conditional_oneway_value::kBackward &&
            dir == direction::kBackward);
  }

  static constexpr std::optional<conditional_oneway_value> static_oneway_value(
      way_properties const& e) {
    if (!e.is_oneway_car()) {
      return std::nullopt;
    }
    return e.is_oneway_reverse() ? conditional_oneway_value::kBackward
                                 : conditional_oneway_value::kForward;
  }

  static constexpr bool access_allowed(access_value const value) {
    switch (value) {
      case access_value::kNo: [[fallthrough]];
      case access_value::kPrivate: [[fallthrough]];
      case access_value::kDiscouraged: return false;
      default: return true;
    }
  }

  static bool hazmat_applies(parameters const& params) {
    return params.hazmat_ || params.hazmat_water_;
  }

  static bool hazmat_water_applies(parameters const& params) {
    return params.hazmat_water_;
  }

  static bool trailer_applies(parameters const& params) {
    return params.trailer_;
  }

  static bool exceeds_limit(parameters const& params,
                            conditional_restriction_field const field,
                            conditional_numeric_value const& value) {
    if (value.state_ != conditional_numeric_state::kValue) {
      return false;
    }
    switch (field) {
      case conditional_restriction_field::kMaxLength:
        return params.length_cm_ > value.value_;
      case conditional_restriction_field::kMaxWeightRating:
        return params.weight_100kg_ > value.value_;
      case conditional_restriction_field::kMaxHeight:
        return params.height_cm_ > value.value_;
      case conditional_restriction_field::kMaxWidth:
        return params.width_cm_ > value.value_;
      case conditional_restriction_field::kMaxWeight:
        return params.weight_100kg_ > value.value_;
      case conditional_restriction_field::kMaxAxleLoad:
        return params.axle_load_100kg_ > value.value_;
      case conditional_restriction_field::kMaxAxles:
        return params.axle_count_ > value.value_;
      default: return false;
    }
  }

  static void apply_access_condition(conditional_access_restriction const& r,
                                     parameters const& params,
                                     direction const dir,
                                     way_state& state) {
    if (!conditional_direction_applies(r.direction_, dir)) {
      return;
    }
    switch (r.field_) {
      case conditional_restriction_field::kAccess:
        if (!mode_applies(r.mode_)) {
          return;
        }
        state.hgv_access_ = r.value_;
        if (is_forbidden(r.value_)) {
          state.accessible_ = false;
        } else {
          state.accessible_ = true;
          state.destination_penalty_ = is_destination_like(r.value_);
        }
        return;
      case conditional_restriction_field::kHazmat:
        if (hazmat_applies(params)) {
          if (is_forbidden(r.value_)) {
            state.accessible_ = false;
          } else if (is_designated(r.value_)) {
            state.designated_preference_ = true;
          }
        }
        return;
      case conditional_restriction_field::kHazmatWater:
        if (hazmat_water_applies(params)) {
          if (is_forbidden(r.value_)) {
            state.accessible_ = false;
          } else if (is_designated(r.value_)) {
            state.designated_preference_ = true;
          }
        }
        return;
      case conditional_restriction_field::kTrailer:
        if (trailer_applies(params)) {
          if (is_forbidden(r.value_)) {
            state.accessible_ = false;
          } else if (is_designated(r.value_)) {
            state.designated_preference_ = true;
          }
        }
        return;
      default: return;
    }
  }

  static void apply_numeric_condition(conditional_numeric_restriction const& r,
                                      parameters const& params,
                                      direction const dir,
                                      way_state& state) {
    if (!conditional_direction_applies(r.direction_, dir) ||
        !mode_applies(r.mode_)) {
      return;
    }
    if (r.field_ == conditional_restriction_field::kMaxSpeed &&
        r.value_.state_ == conditional_numeric_state::kValue) {
      state.max_speed_km_h_ =
          static_cast<std::uint16_t>(std::min<std::uint32_t>(
              r.value_.value_, std::numeric_limits<std::uint16_t>::max()));
      return;
    }
    if (exceeds_limit(params, r.field_, r.value_)) {
      state.accessible_ = false;
    }
  }

  static void apply_oneway_condition(conditional_oneway_restriction const& r,
                                     direction const dir,
                                     way_state& state) {
    if (conditional_direction_applies(r.direction_, dir) &&
        mode_applies(r.mode_)) {
      state.oneway_ = r.value_;
    }
  }

  static void apply_conditional_restrictions(
      parameters const& params,
      ways::routing const& w,
      way_idx_t const way,
      direction const dir,
      std::optional<routing_time_t> const current_time,
      way_state& state) {
    auto const* conditionals = w.get_conditional_restrictions(way);
    if (conditionals == nullptr) {
      return;
    }
    for (auto i = conditionals->access_.begin_; i != conditionals->access_.end_;
         ++i) {
      auto const& r = w.conditional_access_[i];
      if (matches_profile_condition_set_utc(params, w, r.condition_set_,
                                            current_time)) {
        apply_access_condition(r, params, dir, state);
      }
    }
    for (auto i = conditionals->numeric_.begin_;
         i != conditionals->numeric_.end_; ++i) {
      auto const& r = w.conditional_numeric_[i];
      if (matches_profile_condition_set_utc(params, w, r.condition_set_,
                                            current_time)) {
        apply_numeric_condition(r, params, dir, state);
      }
    }
    for (auto i = conditionals->oneway_.begin_; i != conditionals->oneway_.end_;
         ++i) {
      auto const& r = w.conditional_oneway_[i];
      if (matches_profile_condition_set_utc(params, w, r.condition_set_,
                                            current_time)) {
        apply_oneway_condition(r, dir, state);
      }
    }
  }

  static constexpr std::uint16_t get_speed(
      parameters const& params,
      way_properties const& e,
      hgv_way_info const* info,
      std::optional<std::uint16_t> const max_speed = std::nullopt) {
    auto speed =
        std::min<std::uint16_t>(params.top_speed_km_h_, e.max_speed_km_per_h());
    if (info != nullptr && info->has(hgv_info_field::kMaxSpeed)) {
      speed = std::min<std::uint16_t>(speed, info->maxspeed_km_h_);
    }
    if (max_speed.has_value()) {
      speed = std::min<std::uint16_t>(speed, *max_speed);
    }
    return std::max<std::uint16_t>(speed, 1U);
  }

  static constexpr bool fits_vehicle(parameters const& params,
                                     hgv_way_info const& info) {
    if (info.has(hgv_info_field::kHazmat) &&
        (params.hazmat_ || params.hazmat_water_) &&
        is_forbidden(info.hazmat_access())) {
      return false;
    }
    if (info.has(hgv_info_field::kHazmatWater) && params.hazmat_water_ &&
        is_forbidden(info.hazmat_water_access())) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxLength) &&
        params.length_cm_ > info.maxlength_cm_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxWeightRating) &&
        params.weight_100kg_ > info.maxweightrating_100kg_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxHeight) &&
        params.height_cm_ > info.maxheight_cm_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxWidth) &&
        params.width_cm_ > info.maxwidth_cm_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxWeight) &&
        params.weight_100kg_ > info.maxweight_100kg_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxAxleLoad) &&
        params.axle_load_100kg_ > info.maxaxleload_100kg_) {
      return false;
    }
    if (info.has(hgv_info_field::kMaxAxles) &&
        params.axle_count_ > info.maxaxles_) {
      return false;
    }
    if (info.has(hgv_info_field::kTrailer) && params.trailer_ &&
        is_forbidden(info.trailer_access())) {
      return false;
    }
    return true;
  }

  static void apply_static_access_preferences(parameters const& params,
                                              hgv_way_info const& info,
                                              way_state& state) {
    if (info.has(hgv_info_field::kHazmat) && hazmat_applies(params) &&
        is_designated(info.hazmat_access())) {
      state.designated_preference_ = true;
    }
    if (info.has(hgv_info_field::kHazmatWater) &&
        hazmat_water_applies(params) &&
        is_designated(info.hazmat_water_access())) {
      state.designated_preference_ = true;
    }
    if (info.has(hgv_info_field::kTrailer) && trailer_applies(params) &&
        is_designated(info.trailer_access())) {
      state.designated_preference_ = true;
    }
  }

  static constexpr cost_t distance_cost(
      parameters const& params,
      way_properties const& e,
      hgv_way_info const* info,
      distance_t const dist,
      bool const destination_penalty,
      bool const designated_preference,
      access_value const hgv_access,
      std::optional<std::uint16_t> const max_speed = std::nullopt) {
    auto const speed = get_speed(params, e, info, max_speed);
    return static_cast<cost_t>(boost::math::ccmath::round(
               (e.is_detour() ? kDetourCostFactor : 1.0F) *
               hgv_access_factor(destination_penalty, designated_preference,
                                 hgv_access) *
               static_cast<float>(dist) * (3.6F / static_cast<float>(speed)))) +
           hgv_access_penalty(destination_penalty, hgv_access);
  }

  static constexpr duration_t distance_duration(
      parameters const& params,
      way_properties const& e,
      hgv_way_info const* info,
      distance_t const dist,
      std::optional<std::uint16_t> const max_speed = std::nullopt) {
    auto const speed = get_speed(params, e, info, max_speed);
    return duration_from_cost(static_cast<cost_t>(boost::math::ccmath::round(
        static_cast<float>(dist) * (3.6F / static_cast<float>(speed)))));
  }

  static constexpr float hgv_access_factor(bool const destination_penalty,
                                           bool const designated_preference,
                                           access_value const hgv_access) {
    switch (hgv_access) {
      case access_value::kDesignated: return 0.8F;
      case access_value::kDelivery: [[fallthrough]];
      case access_value::kDestination: return 5.0F;
      default:
        if (designated_preference) {
          return 0.8F;
        }
        return destination_penalty ? 5.0F : 1.0F;
    }
  }

  static constexpr cost_t hgv_access_penalty(bool const destination_penalty,
                                             access_value const hgv_access) {
    switch (hgv_access) {
      case access_value::kDelivery: [[fallthrough]];
      case access_value::kDestination: return 120U;
      default: return destination_penalty ? 120U : 0U;
    }
  }
};

}  // namespace osr
