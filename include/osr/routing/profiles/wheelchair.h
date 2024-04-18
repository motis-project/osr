#pragma once

#include "osr/ways.h"

namespace osr {

struct wheelchair {
  static constexpr auto const kMaxMatchDistance = 100U;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(), .lvl_{level_t::invalid()}};
    }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node get_key() const noexcept { return *this; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_]
                 << ", level=" << to_float(lvl_) << ")";
    }

    node_idx_t n_;
    level_t lvl_;
  };

  using key = node;

  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_lvl_}};
    }
    constexpr cost_t cost(node) const noexcept { return cost_; }
    constexpr bool update(node, cost_t const c, node const pred) noexcept {
      if (c < cost_) {
        cost_ = c;
        pred_ = pred.n_;
        pred_lvl_ = pred.lvl_;
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
                      Fn&& f) {
    f(node{n, w.way_properties_[way].get_level()});
  }

  template <typename Fn>
  static void resolve_all(ways const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    auto levels = hash_set<level_t>{};
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      auto const way = w.node_ways_[n][i];
      auto const way_prop = w.way_properties_[way];
      if (way_prop.is_elevator() &&
          (lvl == level_t::invalid() || way_prop.can_use_elevator(lvl))) {
        if (levels.emplace(lvl).second) {
          f(node{n, lvl});
        }
        //        fmt::println("creating elevator start for {}",
        //        w.way_osm_idx_[way]); for (auto l = way_prop.get_level(); l <=
        //        way_prop.get_to_level(); ++l) {
        //          if (levels.emplace(l).second) {
        //            f(node{n, l});
        //          }
        //        }
      } else {
        auto const lvl = way_prop.get_level();
        if (levels.emplace(lvl).second) {
          f(node{n, lvl});
        }
      }
    }
  }

  static bool is_reachable(ways const& w,
                           node const n,
                           way_idx_t const way,
                           direction const way_dir) {
    auto const from_node_prop = w.node_properties_[n.n_];

    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    auto const target_lvl = target_way_prop.get_level();
    auto const level_ok = n.lvl_ == target_lvl ||
                          (from_node_prop.can_use_elevator(n.lvl_, target_lvl));
    if (!level_ok) {
      return false;
    }

    return true;
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
        if (target_way_prop.is_steps_ ||
            way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if (target_way_prop.can_use_elevator(n.lvl_)) {
          fmt::println("using elevator way {}: curr={}, elevator=[{}, {}]",
                       w.way_osm_idx_[way], to_float(n.lvl_),
                       to_float(target_way_prop.get_level()),
                       to_float(target_way_prop.get_to_level()));
          auto const cost = way_cost(target_way_prop, way_dir, 0U) +
                            node_cost(target_node_prop);
          auto const dist = w.way_node_dist_[way][std::min(from, to)];
          for (auto l = target_way_prop.get_level();
               l <= target_way_prop.get_to_level(); ++l) {
            if (n.lvl_ != l) {
              fn(node{target_node, l}, cost, 0U, way, from, to);
            }
          }
          return;
        } else if (target_way_prop.is_elevator()) {
          fmt::println("not using elevator way {}: curr={}, elevator=[{}, {}]",
                       w.way_osm_idx_[way], n.lvl_,
                       to_float(target_way_prop.get_level()),
                       to_float(target_way_prop.get_to_level()));
        }

        auto const target_lvl = target_way_prop.get_level();
        auto const level_ok =
            n.lvl_ == target_lvl || target_node_prop.is_entrance() ||
            (from_node_prop.can_use_elevator(n.lvl_, target_lvl));
        if (!level_ok) {
          return;
        }

        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto const cost = way_cost(target_way_prop, way_dir, dist) +
                          node_cost(target_node_prop);
        fn(node{target_node, target_way_prop.get_level()}, cost, dist, way,
           from, to);
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

}  // namespace osr