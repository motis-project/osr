#pragma once

#include <variant>

#include "osr/ways.h"

namespace osr {

struct foot {
  struct node {
    CISTA_FRIEND_COMPARABLE(node)

    constexpr node_idx_t get_node() const noexcept { return n_; }

    node_idx_t n_;
    level_t lvl_;
  };

  struct entry {
    constexpr std::optional<node> pred() const noexcept {
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_lvl_}};
    }
    constexpr cost_t cost() const noexcept { return cost_; }
    constexpr bool update(cost_t const c) noexcept {
      if (c < cost_) {
        cost_ = c;
        return true;
      }
      return false;
    }

    node_idx_t pred_{node_idx_t::invalid()};
    level_t pred_lvl_;
    cost_t cost_{kInfeasible};
  };

  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, lvl_{n.lvl_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, lvl_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    level_t lvl_;
    cost_t cost_;
  };

  static constexpr auto const kMaxMatchDistance = 100U;

  struct hash {
    using is_avalanching = void;
    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::mix(
          wyhash::hash(static_cast<std::uint64_t>(to_idx(n.lvl_))),
          wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_))));
    }
  };

  template <typename Fn>
  static void resolve(ways const& w,
                      way_idx_t const way,
                      node_idx_t const n,
                      level_t const lvl,
                      Fn&& f) {
    if (lvl == level_t::invalid() ||
        w.way_properties_[way].get_level() == lvl) {
      f(node{n, lvl});
    }
  }

  template <direction SearchDir, typename Fn>
  static void adjacent(ways const& w, node const n, Fn&& fn) {
    auto const from_node_prop = w.node_properties_[n.n_];

    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        auto const target_node = w.way_nodes_[way][to];
        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        auto const target_lvl = target_way_prop.get_level();
        auto const level_ok =
            n.lvl_ == target_lvl ||
            (from_node_prop.can_use_elevator(n.lvl_, target_lvl)) ||
            (target_way_prop.can_use_steps(n.lvl_, target_lvl));
        if (!level_ok) {
          return;
        }

        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        fn(node{target_node, target_way_prop.get_level()},
           way_cost(target_way_prop, way_dir, dist) +
               node_cost(target_node_prop));
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  static constexpr cost_t way_cost(way_properties const e,
                                   direction,
                                   std::uint16_t const dist) {
    if (e.is_foot_accessible()) {
      return static_cast<cost_t>(std::round(dist / 1.2F));
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const n) {
    return (n.is_walk_accessible() ? (n.is_elevator_ ? 90U : 0U) : kInfeasible);
  }
};

/*
struct bike {
  using node = node_idx_t;

  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_bike_accessible() &&
        (dir == direction::kForward || !e.is_oneway_bike())) {
      return static_cast<dist_t>(std::round(dist / 3.5F));
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_bike_accessible() ? 0U : kInfeasible;
  }
};

struct car {
  struct node {
    node_idx_t node_;
    way_pos_t way_;
    direction dir_;
  };

  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s());
    } else {
      return kInfeasible;
    }
  }

  dist_t operator()(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

struct generic {
  explicit generic(search_profile const p) : fn_{get(p)} {}

  static std::variant<foot, bike, car> get(search_profile const p) {
    switch (p) {
      case search_profile::kFoot: return foot{};
      case search_profile::kBike: return bike{};
      case search_profile::kCar: return car{};
    }
    std::unreachable();
  }

  dist_t operator()(way_properties const& e,
                    direction const dir,
                    std::uint16_t const dist) {
    return std::visit([&](auto&& f) { return f(e, dir, dist); }, fn_);
  }

  dist_t operator()(node_properties const& n) {
    return std::visit([&](auto&& f) { return f(n); }, fn_);
  }

  std::variant<foot, bike, car> fn_;
};
*/

}  // namespace osr