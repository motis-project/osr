#pragma once

#include <bitset>

#include "boost/graph/graph_traits.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/ways.h"

namespace osr {

struct hybrid {
  static constexpr auto const kMaxMatchDistance = 200U;
  static constexpr auto const kUturnPenalty = cost_t{120U};
  static constexpr auto const kSwitchPenalty = cost_t{200U};
  static constexpr auto const kElevatorPenalty = cost_t{90U};

  using key = node_idx_t;

  enum class node_type : std::uint8_t { kCar, kFoot, kInvalid };


  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kCar: return "car";
      case node_type::kFoot: return "foot";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }
  /**
   * We have x nodes for each real node.
   * Where x is calculated via:
   * (number of ways the node is conntected to) * (number of possible directions
   * = 2) + (foot node = 1)
   *
   * E.g. if a ndoes has 3 ways connected to it, we have 7 nodes for this node.
   */
struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return node{};
    }

    constexpr node_idx_t get_node() const noexcept {
      return idx_;
    }

    boost::json::object geojson_properties(ways const& w) const {
      auto properties =  boost::json::object{
        {"node_id", idx_.v_},
        {"level", to_float(lvl_)},
        {"type", node_type_to_str(type_)}
      };

      if (is_car_node()) {
        properties.emplace("direction", to_str(dir_));
      }

      return properties;
    }

    constexpr node_idx_t get_key() const noexcept {
      return idx_;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[idx_]
          << ", level=" << to_float(lvl_)
          << ", dir=" << to_str(dir_)
          << ", way=" << w.way_osm_idx_[w.r_->node_ways_[idx_][node_way_index_]]
          << ")";
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

    node_type type_{node_type::kInvalid};
    node_idx_t idx_{node_idx_t::invalid()};

    level_t lvl_{level_t::invalid()};
    bool is_parking_;

    direction dir_{};
    way_pos_t node_way_index_{};
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U + 1 /* FWD+BWD + foot */;

    entry() {
      utl::fill(cost_, kInfeasible);
      utl::fill(pred_, node_idx_t::invalid());
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);

      if (pred_[idx] == node_idx_t::invalid()) {
        return std::nullopt;
      }

      return node{
        to_node_type(pred_type_[idx]),
        pred_[idx],
        pred_lvl_[idx],
        pred_parking_[idx],
        to_dir(pred_dir_[idx]),
        pred_way_[idx],
      };
    }

    constexpr cost_t cost(node const n) const noexcept {
      return cost_[get_index(n)];
    }

    constexpr bool update(node const n,
                          cost_t const c,
                          node const pred) noexcept {
      auto const idx = get_index(n);

      if (c < cost_[idx]) {
        cost_[idx] = c;
        pred_[idx] = pred.idx_;
        pred_lvl_[idx] = pred.lvl_;
        pred_parking_[idx] = pred.is_parking_;
        pred_type_[idx] = to_bool(pred.type_);
        pred_way_[idx] = pred.node_way_index_;
        pred_dir_[idx] = to_bool(pred.dir_);

        return true;
      }

      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      if (n.is_foot_node()) {
          return 0;
      }

      return 1 + (n.dir_ == direction::kForward ? 0 : 1) * kMaxWays + n.node_way_index_;
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

    std::array<node_idx_t, kN> pred_{};
    std::array<cost_t, kN> cost_{};
    std::array<way_pos_t, kN> pred_way_{};
    std::array<level_t, kN> pred_lvl_{};
    std::bitset<kN> pred_dir_{};
    std::bitset<kN> pred_type_{};
    std::bitset<kN> pred_parking_{};
  };

  struct label {
    label(node const n, cost_t const c)
        : idx_{n.idx_}, type_{n.type_}, lvl_{n.lvl_}, dir_{n.dir_}, node_way_index_(n.node_way_index_), cost_{c}, is_parking_{n.is_parking_} {}

    constexpr node get_node() const noexcept { return node{type_, idx_, lvl_, is_parking_, dir_, node_way_index_}; }
    constexpr cost_t cost() const noexcept { return cost_; }


    node_idx_t idx_;

    node_type type_;
    level_t lvl_;
    direction dir_;
    way_pos_t node_way_index_;

    cost_t cost_;

    bool is_parking_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  /**
   * resolve is only used to determine the start nodes. We start in car or foot
   * mode depending on search direction.
   */
  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                      way_idx_t const way,
                      node_idx_t const n,
                      level_t const lvl,
                      direction const search_dir,
                      Fn&& f) {
    auto const way_properties = w.way_properties_[way];
    auto const ways = w.node_ways_[n];


    level_t const node_level = lvl == level_t::invalid() ? way_properties.from_level() : lvl;
    bool const is_parking = w.node_properties_[n].is_parking_;

    if (search_dir == direction::kBackward) {
      // when seraching backward, we have to start with the foot node
      if (lvl == level_t::invalid() || (way_properties.from_level() == lvl || way_properties.to_level() == lvl || can_use_elevator(w, n, lvl))) {
        f(node{node_type::kFoot, n, node_level, is_parking});
      }

      return;
    }

    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      if (ways[i] == way) {
        f(node{node_type::kCar, n, node_level, is_parking, direction::kForward, i});
        f(node{node_type::kCar, n, node_level, is_parking, direction::kBackward, i});
      }
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];;
    auto const node_properties = w.node_properties_[n];

    auto levels = hash_set<level_t>{};

    auto const is_parking = node_properties.is_parking_;

    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      auto const p = w.way_properties_[w.node_ways_[n][i]];

      auto const node_level = lvl == level_t::invalid() ? p.from_level() : lvl;

      f(node{node_type::kCar, n, node_level, is_parking, direction::kForward, i});
      f(node{node_type::kCar, n, node_level, is_parking, direction::kBackward, i});

      if (lvl == level_t::invalid()) {
        if (levels.emplace(p.from_level()).second) {
          f(node{node_type::kFoot, n, p.from_level(), is_parking});
        }
        if (levels.emplace(p.to_level()).second) {
          f(node{node_type::kFoot, n, p.to_level(), is_parking});
        }
      } else if ((p.from_level() == lvl || p.to_level() == lvl ||
                  can_use_elevator(w, n, lvl)) &&
                 levels.emplace(lvl).second) {
        f(node{node_type::kFoot, n, lvl, is_parking});
      }
    }
  }

  template <direction SearchDir, typename Fn>
  static void adjacent_foot_nodes(
    ways::routing const& w,
    node const& n,
    direction const way_dir,
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to,
    Fn&& fn
  ) {
    if (SearchDir == direction::kBackward && n.is_car_node()) {
      // we will transition to foot via the adjacents_by_car function when going backwards,
      // since we are computing the path in reverse order
      return;
    }

    auto const target_node = w.way_nodes_[way][to];
    auto const target_node_prop = w.node_properties_[target_node];
    cost_t target_node_cost = foot_node_cost(target_node_prop);

    if (target_node_cost == kInfeasible) {
      return;
    }

    auto const target_way_prop = w.way_properties_[way];

    if (foot_way_cost(target_way_prop, 0U) == kInfeasible) {
      return;
    }

    if (n.is_car_node()) {
      if (!n.is_parking_) {
        return;
      }

      target_node_cost += kSwitchPenalty;
    }

    if (can_use_elevator(w, target_node, n.lvl_)) {
      for_each_elevator_level(
          w, target_node, [&](level_t const target_lvl) {
            auto const dist = w.way_node_dist_[way][std::min(from, to)];
            auto const cost = foot_way_cost(target_way_prop, dist) + target_node_cost;
            fn(node{node_type::kFoot, target_node, target_lvl, target_node_prop.is_parking_}, cost, dist, way, from, to);
          });

      return; // node is an elevator
    }

    auto const target_lvl = get_target_level(w, n, way);

    if (!target_lvl.has_value()) {
      return;
    }

    auto const dist = w.way_node_dist_[way][std::min(from, to)];
    auto const cost = foot_way_cost(target_way_prop, dist) + target_node_cost;
    fn(node{node_type::kFoot, target_node, *target_lvl, target_node_prop.is_parking_}, cost, dist, way, from, to);
  }

  template <direction SearchDir, typename Fn>
  static void adjacent_car_nodes(
    ways::routing const& w,
    node const& n,
    direction const way_dir,
    way_pos_t const way_pos,
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to,
    Fn&& fn
    ) {

    if(SearchDir == direction::kForward && n.is_foot_node()) {
      // we will transition to car via the adjacents_by_foot function when going forward.
      return;
    }

    auto const target_node = w.way_nodes_[way][to];
    auto const target_node_prop = w.node_properties_[target_node];
    auto const target_node_cost = car_node_cost(target_node_prop);

    if (target_node_cost == kInfeasible) {
      return;
    }

    auto const target_way_prop = w.way_properties_[way];

    if (car_way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
      return;
    }

    auto const way_prop = w.way_properties_[way];
    auto const target_level = way_prop.from_level() == n.lvl_ ? way_prop.to_level() : way_prop.from_level();

    auto const target =
        node{node_type::kCar, target_node, target_level, target_node_prop.is_parking_, way_dir, w.get_way_pos(target_node, way)};

    if(n.is_foot_node()) {
      if (!n.is_parking_) {
        return;
      }

      auto const dist = w.way_node_dist_[way][std::min(from, to)];
      cost_t const cost = car_way_cost(target_way_prop, way_dir, dist) + target_node_cost + kSwitchPenalty;
      fn(target, cost, dist, way, from, to);
    }

    if (w.is_restricted<SearchDir>(n.idx_, n.node_way_index_, way_pos)) {
      return;
    }

    auto const is_u_turn = way_pos == n.node_way_index_ && way_dir == opposite(n.dir_);
    auto const dist = w.way_node_dist_[way][std::min(from, to)];
    cost_t const cost = car_way_cost(target_way_prop, way_dir, dist)
                            + target_node_cost
                            + (is_u_turn ? kUturnPenalty : 0U);
    fn(target, cost, dist, way, from, to);
  }

  template <direction SearchDir, typename Fn>
  static void adjacent(ways::routing const& w, node const n, Fn&& fn) {
    auto way_pos = way_pos_t{0U};

    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.idx_], w.node_in_way_idx_[n.idx_])) {

      if (i != 0U) {
        adjacent_foot_nodes<SearchDir, Fn>(w, n, flip<SearchDir>(direction::kBackward), way, i, i - 1, std::forward<Fn>(fn));
        adjacent_car_nodes<SearchDir, Fn>(w, n, flip<SearchDir>(direction::kBackward), way_pos, way, i, i - 1, std::forward<Fn>(fn));
      }

      if (i != w.way_nodes_[way].size() - 1U) {
        adjacent_foot_nodes<SearchDir, Fn>(w, n, flip<SearchDir>(direction::kForward), way, i, i + 1, std::forward<Fn>(fn));
        adjacent_car_nodes<SearchDir, Fn>(w, n, flip<SearchDir>(direction::kForward), way_pos, way, i, i + 1, std::forward<Fn>(fn));
      }

      ++way_pos;
    }
  }

  /**
   * This function is only used to determine if the destination node
   * is reachable. We want to reach the destination node in foot mode,
   * so we check for that.
   */
  static bool is_reachable(ways::routing const& w,
                           node const n,
                           way_idx_t const way,
                           direction const way_dir,
                           direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];

    if (search_dir == direction::kBackward) {
      if ((n.is_parking_ || n.is_foot_node())
          && foot_way_cost(target_way_prop, 0U) != kInfeasible
          && get_target_level(w, n, way).has_value()) {
        return true;
      }

      return false;
    }

    if ((n.is_parking_ || n.is_car_node())
      && car_way_cost(target_way_prop, way_dir, 0U) != kInfeasible
      && !w.is_restricted(n.idx_, n.node_way_index_, w.get_way_pos(n.idx_, way), search_dir)) {
      return true;
    }

    return false;
  }

  static constexpr cost_t car_way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {
    if (e.is_car_accessible() && (dir == direction::kForward || !e.is_oneway_car())) {
      return (dist / e.max_speed_m_per_s()) * (e.is_destination() ? 5U : 1U) +
             (e.is_destination() ? 120U : 0U);
    }

    return kInfeasible;
  }

  static constexpr cost_t foot_way_cost(way_properties const& e,
                                   std::uint16_t const dist) {
    if (e.is_foot_accessible() || e.is_bike_accessible()) {
      return static_cast<cost_t>(std::round(static_cast<float>(dist) / 1.1F));
    }

    return kInfeasible;
  }

  /**
   * In the hybrid profile, this function is only used to determine cost
   * for start and end nodes. Since we start in car mode and end in foot mode,
   * we have to calculate the cost accordingly.
   */
  static constexpr cost_t way_cost(way_properties const& e,
                                   direction const dir,
                                   std::uint16_t const dist) {

    return foot_way_cost(e, dist);
  }

  static constexpr cost_t car_node_cost(node_properties const& n) {
    if (n.is_car_accessible()) {
      return 0U;
    }

    return kInfeasible;
  }

  static constexpr cost_t foot_node_cost(node_properties const& n) {
    if (n.is_walk_accessible()) {
      return n.is_elevator() ? kElevatorPenalty : 0U;
    }

    return kInfeasible;
  }

  // foot level stuff

  static std::optional<level_t> get_target_level(ways::routing const& w,
                                               node const& from_node,
                                               way_idx_t const to_way) {
    auto const way_prop = w.way_properties_[to_way];
    level_t const from_level = from_node.lvl_;

    if (way_prop.is_steps()) {
      if (way_prop.from_level() == from_level) {
        return way_prop.to_level();
      }

      if (way_prop.to_level() == from_level) {
        return way_prop.from_level();
      }

      return std::nullopt;
    }

    if (can_use_elevator(w, to_way, from_level)) {
      return from_level;
    }

    if (can_use_elevator(w, from_node.idx_, way_prop.from_level(), from_level)) {
      return way_prop.from_level();
    }

    if (way_prop.from_level() == from_level) {
      return from_level;
    }

    return std::nullopt;
  }


  static bool can_use_elevator(ways::routing const& w,
                               way_idx_t const way,
                               level_t const a,
                               level_t const b = level_t::invalid()) {
    return w.way_properties_[way].is_elevator() &&
           can_use_elevator(w, w.way_nodes_[way][0], a, b);
  }

  template <typename Fn>
  static void for_each_elevator_level(ways::routing const& w,
                                      node_idx_t const n,
                                      Fn&& f) {
    auto const p = w.node_properties_[n];
    if (p.is_multi_level()) {
      for_each_set_bit(get_elevator_multi_levels(w, n),
                       [&](auto&& l) { f(level_t{l}); });
    } else {
      f(p.from_level());
      f(p.to_level());
    }
  }

  static bool can_use_elevator(ways::routing const& w,
                               node_idx_t const n,
                               level_t const a,
                               level_t const b = level_t::invalid()) {
    auto const p = w.node_properties_[n];
    if (!p.is_elevator()) {
      return false;
    }

    if (p.is_multi_level()) {
      auto const levels = get_elevator_multi_levels(w, n);
      return has_bit_set(levels, to_idx(a)) &&
             (b == level_t::invalid() || has_bit_set(levels, to_idx(b)));
    } else {
      return (a == p.from_level() || a == p.to_level()) &&
             (b == level_t::invalid() || b == p.from_level() ||
              b == p.to_level());
    }
  }

  static level_bits_t get_elevator_multi_levels(ways::routing const& w,
                                                node_idx_t const n) {
    auto const it = std::lower_bound(
        begin(w.multi_level_elevators_), end(w.multi_level_elevators_), n,
        [](auto&& x, auto&& y) { return x.first < y; });
    assert(it != end(w.multi_level_elevators_) && it->first == n);
    return it->second;
  }

};

}  // namespace osr