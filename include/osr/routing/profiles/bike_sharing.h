#pragma once

#include <cassert>
#include <array>
#include <string_view>
#include <type_traits>

#include "boost/json.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/routing/additional_edge.h"
#include "osr/routing/mode.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

namespace osr {

struct bike_sharing {
  using footp = foot<false>;

  // initial foot -> bike
  static constexpr auto const kStartSwitchPenalty = cost_t{30U};
  // bike -> trailing foot
  static constexpr auto const kEndSwitchPenalty = cost_t{30U};

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
                     .from_level_ = 0,
                     .to_level_ = 0,
                     .is_platform_ = 0,
                     .is_parking_ = 0};

  static constexpr auto const kAdditionalNodeProperties =
      node_properties{.from_level_ = 0,
                      .is_foot_accessible_ = true,
                      .is_bike_accessible_ = true,
                      .is_car_accessible_ = false,
                      .is_elevator_ = false,
                      .is_entrance_ = false,
                      .is_multi_level_ = false,
                      .is_parking_ = false,
                      .to_level_ = 0};

  enum class node_type : std::uint8_t {
    kInitialFoot,
    kBike,
    kTrailingFoot,
    kInvalid,
  };

  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kInitialFoot: return "initial_foot";
      case node_type::kBike: return "bike";
      case node_type::kTrailingFoot: return "trailing_foot";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }

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
      return a.n_ == b.n_ && a.type_ == b.type_ &&
             (a.lvl_ == b.lvl_ || (is_zero(a.lvl_) && is_zero(b.lvl_)));
    }

    boost::json::object geojson_properties(ways const& w) const {
      auto properties =
          boost::json::object{{"osm_node_id", to_idx(w.node_to_osm_[n_])},
                              {"level", lvl_.to_float()},
                              {"type", node_type_to_str(type_)}};
      return properties;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node="
                 << (n_ >= w.n_nodes() ? osm_node_idx_t{to_idx(n_)}
                                       : w.node_to_osm_[n_])
                 << (n_ >= w.n_nodes() ? "*" : "") << ", level=" << lvl_
                 << ", type=" << node_type_to_str(type_) << ")";
    }

    static constexpr node invalid() noexcept { return {}; }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr key get_key() const noexcept { return {n_, lvl_}; }

    constexpr mode get_mode() const noexcept {
      return is_bike_node() ? mode::kBike : mode::kFoot;
    }

    constexpr bool is_initial_foot_node() const noexcept {
      return type_ == node_type::kInitialFoot;
    }

    constexpr bool is_bike_node() const noexcept {
      return type_ == node_type::kBike;
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
  };

  struct label {
    constexpr label(node const n, cost_t const c)
        : n_{n.n_}, type_{n.type_}, lvl_{n.lvl_}, cost_{c} {}

    constexpr node get_node() const noexcept {
      return {.n_ = n_, .type_ = type_, .lvl_ = lvl_};
    }

    constexpr cost_t cost() const noexcept { return cost_; }

    void track(label const&, ways::routing const&, way_idx_t, node_idx_t) {}

    node_idx_t n_;
    node_type type_;
    level_t lvl_;
    cost_t cost_;
  };

  struct entry {
    static constexpr auto const kN =
        static_cast<std::underlying_type_t<node_type>>(node_type::kInvalid);

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
                                      .lvl_ = pred_lvl_[idx]}};
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
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      return static_cast<std::size_t>(n.type_);
    }

    void write(node, path&) const {}

    std::array<node_idx_t, kN> pred_{};
    std::array<cost_t, kN> cost_{};
    std::array<level_t, kN> pred_lvl_{};
    std::array<node_type, kN> pred_type_{};
  };

  static footp::node to_foot(node const n) {
    return {.n_ = n.n_, .lvl_ = n.lvl_};
  }

  static bike::node to_bike(node const n) { return {.n_ = n.n_}; }

  static node to_node(footp::node const n, node_type const type) {
    return {.n_ = n.n_, .type_ = type, .lvl_ = n.lvl_};
  }

  static node to_node(bike::node const n, level_t const lvl) {
    return {.n_ = n.n_, .type_ = node_type::kBike, .lvl_ = lvl};
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
      f(to_node(neighbor, node_type::kBike));
    });
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* sharing,
                       Fn&& fn) {
    assert(sharing != nullptr);

    auto const& handle_additional_edge =
        [&](additional_edge const& ae, node_type const nt, cost_t const cost) {
          fn(node{.n_ = ae.node_,
                  .type_ = nt,
                  .lvl_ = nt == node_type::kBike ? kNoLevel : n.lvl_},
             cost, ae.distance_, way_idx_t::invalid(), 0, 1);
        };

    auto const& continue_on_foot = [&](node_type const nt,
                                       bool const include_additional_edges,
                                       cost_t const switch_penalty = 0) {
      footp::template adjacent<SearchDir, WithBlocked>(
          w, to_foot(n), blocked, nullptr,
          [&](footp::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to) {
            fn(to_node(neighbor, nt), cost + switch_penalty, dist, way, from,
               to);
          });
      if (include_additional_edges) {
        // walk to station or free-floating bike
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, nt,
                footp::way_cost(kAdditionalWayProperties, direction::kForward,
                                ae.distance_) +
                    switch_penalty);
          }
        }
      }
    };

    auto const& continue_on_bike = [&](bool const include_additional_edges,
                                       cost_t const switch_penalty = 0) {
      bike::adjacent<SearchDir, WithBlocked>(
          w, to_bike(n), blocked, nullptr,
          [&](bike::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to) {
            fn(to_node(neighbor, kNoLevel), cost + switch_penalty, dist, way,
               from, to);
          });
      if (include_additional_edges) {
        // drive to station
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            handle_additional_edge(
                ae, node_type::kBike,
                bike::way_cost(kAdditionalWayProperties, direction::kForward,
                               ae.distance_) +
                    switch_penalty);
          }
        }
      }
    };

    if (SearchDir == direction::kForward) {

      if (n.is_additional_node(sharing)) {
        // additional node - station or free-floating bike
        // switch mode and use additional edge
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            if (n.is_initial_foot_node() &&
                sharing->start_allowed_.test(n.n_)) {
              handle_additional_edge(
                  ae, node_type::kBike,
                  bike::way_cost(kAdditionalWayProperties, direction::kForward,
                                 ae.distance_) +
                      kStartSwitchPenalty);
            } else if (n.is_bike_node() && sharing->end_allowed_.test(n.n_)) {
              handle_additional_edge(
                  ae, node_type::kTrailingFoot,
                  footp::way_cost(kAdditionalWayProperties, direction::kForward,
                                  ae.distance_) +
                      kEndSwitchPenalty);
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_initial_foot_node());
        } else if (n.is_bike_node()) {
          continue_on_bike(true);
          if (sharing->end_allowed_.test(n.n_)) {
            // switch to foot
            continue_on_foot(node_type::kTrailingFoot, false,
                             kEndSwitchPenalty);
          }
        }
      }

    } else /* backward */ {

      if (n.is_additional_node(sharing)) {
        // additional node - station or free-floating bike
        // switch mode and use additional edge
        if (auto const it = sharing->additional_edges_.find(n.n_);
            it != end(sharing->additional_edges_)) {
          for (auto const& ae : it->second) {
            if (n.is_trailing_foot_node() && sharing->end_allowed_.test(n.n_)) {
              handle_additional_edge(
                  ae, node_type::kBike,
                  bike::way_cost(kAdditionalWayProperties, direction::kForward,
                                 ae.distance_) +
                      kEndSwitchPenalty);
            } else if (n.is_bike_node() && sharing->start_allowed_.test(n.n_)) {
              handle_additional_edge(
                  ae, node_type::kInitialFoot,
                  footp::way_cost(kAdditionalWayProperties, direction::kForward,
                                  ae.distance_) +
                      kStartSwitchPenalty);
            }
          }
        }
      } else {
        if (n.is_initial_foot_node() || n.is_trailing_foot_node()) {
          continue_on_foot(n.type_, n.is_trailing_foot_node());
          if (n.is_trailing_foot_node() && sharing->end_allowed_.test(n.n_)) {
            // switch to bike
            continue_on_bike(false, kEndSwitchPenalty);
          }
        } else if (n.is_bike_node()) {
          continue_on_bike(true);
        }
      }
    }
  }

  static bool is_dest_reachable(ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    return !n.is_bike_node() &&
           footp::is_dest_reachable(w, to_foot(n), way, way_dir, search_dir);
  }

  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    return footp::way_cost(e, dir, dist);
  }
};

}  // namespace osr
