#pragma once

#include <bitset>

#include "boost/json/object.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/mode.h"
#include "osr/ways.h"
#include "osr/preprocessing/contraction_hierarchies/storage.h"

namespace osr {
struct path;

namespace ch {

struct shortcut_data;
struct shortcut_storage;

} // namespace osr::ch

struct sharing_data;

struct car {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{120U};

  using key = node_idx_t;

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return node{
          .n_ = node_idx_t::invalid(), .way_ = 0U, .dir_ = direction::kForward};
    }

    boost::json::object geojson_properties(ways const&) const {
      return boost::json::object{{"node_id", n_.v_}, {"type", "car"}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    static constexpr mode get_mode() noexcept { return mode::kCar; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", dir=" << to_str(dir_)
                 << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                 << ")";
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
      cost_.resize(kN,kInfeasible);
      pred_.resize(kN, node_idx_t::invalid());
      pred_dir_.resize(kN);
      pred_way_.resize(kN);
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      if (idx >= cost_.size()) {
        return std::nullopt;
      }
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_[idx], pred_way_[idx],
                                      to_dir(pred_dir_[idx])}};
    }

    constexpr cost_t cost(node const n) const noexcept {
      auto const idx = get_index(n);
      return idx >= cost_.size() ? kInfeasible : cost_[idx];
    }

    constexpr bool update(label const&,
                          node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);
      if (idx >= cost_.size()) {
        size_t new_size = (idx + 1) * 2;
        cost_.resize(new_size, kInfeasible);
        pred_way_.resize(new_size);
        pred_.resize(new_size, node_idx_t::invalid());
        pred_dir_.resize(new_size);
      }
      if (c < cost_[idx]) {
        cost_[idx] = c;
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
      return node{n, static_cast<way_pos_t>((index - (index % 2) / 2)),
                  to_dir((index % 2) != 0U)};
    }

    static constexpr std::size_t get_index(node const n) {
      return (n.dir_ == direction::kForward ? 0U : 1U) + 2 * n.way_;
      return (n.dir_ == direction::kForward ? 0U : 1U) * kMaxWays + n.way_;
    }

    static constexpr direction to_dir(bool const b) {
      return b == false ? direction::kForward : direction::kBackward;
    }

    static constexpr bool to_bool(direction const d) {
      return d == direction::kForward ? false : true;
    }

    std::vector<node_idx_t> pred_;
    std::vector<way_pos_t> pred_way_;
    std::vector<bool> pred_dir_;
    std::vector<cost_t> cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

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
  static void adjacent(ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const*,
                       ch::shortcut_storage const* shortcuts,
                       Fn&& fn) {
    auto way_pos = way_pos_t{0U};
    auto const WITH_SHORTCUT = shortcuts != nullptr;

    auto node_resolved_way = w.node_ways_[n.n_][n.way_];
    auto node_resolved_dir = n.dir_;
    //fmt::print("start node resolving way: {} dir: {}\n",node_resolved_way, to_str(node_resolved_dir));
    if (n.n_ == node_idx_t{254U}) {
      //fmt::print("node: {} way: {} dir: {}\n", n.n_, n.way_, to_str(n.dir_));
    }
    if (WITH_SHORTCUT) {
      auto n_way = w.node_ways_[n.n_][n.way_];
      if(shortcuts->is_shortcut(n_way)) {
        auto resolved = SearchDir == direction::kForward ? shortcuts->resolve_last_way_and_dir(n_way,n.dir_): shortcuts->resolve_first_way_and_dir(n_way,n.dir_);
        node_resolved_way = resolved.way;
        node_resolved_dir = resolved.dir;
      }
    }
    //fmt::print("Resolved node way: {} dir: {}\n",node_resolved_way, to_str(node_resolved_dir));
    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const way_is_shortcut = WITH_SHORTCUT ? shortcuts->is_shortcut(way) : false;


      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {

        auto resolved_way = way;
        auto resolved_dir = way_dir;

        if (way_is_shortcut) {
          auto resolved = SearchDir == direction::kForward? shortcuts->resolve_first_way_and_dir(way,way_dir) : shortcuts->resolve_last_way_and_dir(way,way_dir);
          resolved_way = resolved.way;
          resolved_dir = resolved.dir;
        }

        // NOLINTNEXTLINE(clang-analyzer-core.CallAndMessage)
        auto const target_node = w.way_nodes_[way][to];
        if constexpr (WithBlocked) {
          if (blocked->test(target_node)) {
            return;
          }
        }

        auto const target_node_prop = w.node_properties_[target_node];
        if (node_cost(target_node_prop) == kInfeasible) {
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (!way_is_shortcut && way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if (w.is_restricted<SearchDir>(n.n_, n.way_, way_pos)) {
          return;
        }

        //auto const is_u_turn = way_pos == n.way_ && way_dir == opposite(n.dir_);
        auto const is_u_turn = resolved_way == node_resolved_way && resolved_dir == opposite(node_resolved_dir);
        auto const dist = w.way_node_dist_[way][std::min(from, to)];
        auto pos = way_pos_t{0U};
        if (way_is_shortcut && false) {
          if (to == 0) {
            pos = shortcuts->get_way_pos_at_from_node(way);
          } else {
            pos = shortcuts->get_way_pos_at_to_node(way);
          }
        } else {
          pos = w.get_way_pos(target_node, way, to);
        }

        auto const target =
            node{target_node, pos, way_dir};
        auto const cost = (way_is_shortcut ? way_cost_s(target_way_prop, way_dir, dist,shortcuts->get_shortcut(way)) : way_cost(target_way_prop, way_dir, dist)) +
                          node_cost(target_node_prop) +
                          (is_u_turn ? kUturnPenalty : 0U);
        fn(target, cost, dist, way, from, to, elevation_storage::elevation{},
           false);
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
  static constexpr cost_t way_cost_s(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist,
                                   ch::shortcut_data const* shortcut) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      cost_t cost = 0;
      if (shortcut != nullptr) {
        cost = shortcut->cost;
      } else {
        cost = dist / e.max_speed_m_per_s();
      }
      return (cost) * (e.is_destination() ? 5U : 1U) +
             (e.is_destination() ? 120U : 0U);
        } else {
          return kInfeasible;
        }
  }
  static bool is_dest_reachable(ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];
    if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return false;
    }

    if (w.is_restricted(n.n_, n.way_, w.get_way_pos(n.n_, way), search_dir)) {
      return false;
    }

    return true;
  }

  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_car_accessible() &&
        (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s()) * (e.is_destination() ? 5U : 1U) +
             (e.is_destination() ? 120U : 0U);
    } else {
      return kInfeasible;
    }
  }

  static constexpr cost_t node_cost(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }

  static constexpr double heuristic(double const dist) {
    return dist / (130U / 3.6);
  }
  static constexpr node get_reverse(node const n) {
    return {n.n_, n.way_, opposite(n.dir_)};
  }
};

}  // namespace osr
