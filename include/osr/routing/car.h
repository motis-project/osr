#pragma once

#include "osr/ways.h"

namespace osr {

struct car {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{120U};

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(),
              .way_{level_t::invalid()},
              .dir_ = direction::kForward};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
  };

  struct entry {
    constexpr std::optional<node> pred() const noexcept {
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_way_, pred_dir_}};
    }
    constexpr cost_t cost() const noexcept { return cost_; }
    constexpr bool update(cost_t const c, node const pred) noexcept {
      if (c < cost_) {
        cost_ = c;
        pred_ = pred.n_;
        pred_way_ = pred.way_;
        pred_dir_ = pred.dir_;
        return true;
      }
      return false;
    }

    node_idx_t pred_{node_idx_t::invalid()};
    cost_t cost_{kInfeasible};
    way_pos_t pred_way_;
    direction pred_dir_;
  };

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_}, way_{n.way_}, dir_{n.dir_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, way_, dir_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    way_pos_t way_;
    direction dir_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::mix(
          wyhash::hash(static_cast<std::uint64_t>(n.dir_)),
          wyhash::mix(wyhash::hash(static_cast<std::uint64_t>(n.way_)),
                      wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_)))));
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
      f(node{n, w.get_way_pos(n, way), direction::kForward});
      f(node{n, w.get_way_pos(n, way), direction::kBackward});
    }
  }

  template <direction SearchDir, typename Fn>
  static void adjacent(ways const& w, node const n, Fn&& fn) {
    auto const from_node_prop = w.node_properties_[n.n_];

    auto way_pos = way_pos_t{0U};
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

        auto const restricted = w.node_is_restricted_[n.n_] &&
                                w.is_restricted(n.n_, n.way_, way_pos);
        if (restricted) {
          return;
        }

        auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        fn(node{target_node, w.get_way_pos(target_node, way), way_dir},
           way_cost(target_way_prop, way_dir, dist) +
               node_cost(target_node_prop) + (is_u_turn ? kUturnPenalty : 0U));
      };

      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }

      ++way_pos;
    }
  }

  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s());
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

}  // namespace osr