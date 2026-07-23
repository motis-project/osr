#pragma once

#include <bitset>
#include <optional>

#include "boost/json.hpp"

#include "utl/helpers/algorithm.h"

#include "osr/elevation_storage.h"
#include "osr/routing/for_each_parking_edge.h"
#include "osr/routing/mode.h"
#include "osr/routing/path.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

namespace osr {

struct sharing_data;

// inline node_idx_t to_node_idx(parking_edge_idx_t const parking_edge_idx) {
//   return node_idx_t{to_idx(parking_edge_idx)};
// }
//
// inline parking_edge_idx_t to_parking_edge_idx(node_idx_t const node_idx) {
//   return parking_edge_idx_t{to_idx(node_idx)};
// }

template <bool IsWheelchair, bool UseParking = true>
struct car_parking {
  using footp = foot<IsWheelchair>;

  static constexpr auto const kSwitchPenalty = cost_t{200U};
  static constexpr auto const kMaxMatchDistance = car::kMaxMatchDistance;

  using key = node_idx_t;

  enum class node_type : std::uint8_t { kCar, kFoot, kParking, kInvalid };

  static constexpr std::string_view node_type_to_str(node_type const type) {
    switch (type) {
      case node_type::kCar: return "car";
      case node_type::kFoot: return "foot";
      case node_type::kParking: return "parking";
      case node_type::kInvalid: return "invalid";
    }
    std::unreachable();
  }

  struct parameters {
    using profile_t = car_parking<IsWheelchair, UseParking>;
    car::parameters const car_{};
    footp::parameters const foot_{};
  };

  struct node {
    friend bool operator==(node const& a, node const& b) {
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
      if (is_car_node()) {
        properties.emplace("direction", to_str(dir_));
      }
      return properties;
    }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_] << ", level=" << lvl_
                 << ", dir=" << to_str(dir_)
                 << ", way=" << w.way_osm_idx_[w.r_->node_ways_[n_][way_]]
                 << ", type=" << node_type_to_str(type_) << ")";
    }

    static constexpr node invalid() noexcept { return node{}; }
    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node_idx_t get_key() const noexcept { return n_; }

    constexpr std::optional<direction> get_direction() const noexcept {
      return dir_;
    }

    constexpr mode get_mode() const noexcept {
      return is_car_node() ? mode::kCar : mode::kFoot;
    }

    constexpr bool is_car_node() const noexcept {
      return type_ == node_type::kCar;
    }

    constexpr bool is_foot_node() const noexcept {
      return type_ == node_type::kFoot;
    }

    constexpr bool is_parking_node() const noexcept {
      return type_ == node_type::kParking;
    }

    constexpr bool is_invalid_node() const noexcept {
      return type_ == node_type::kInvalid;
    }

    node_idx_t n_{node_idx_t::invalid()};
    node_type type_{node_type::kInvalid};
    level_t lvl_;
    direction dir_;
    way_pos_t way_;
  };
  static_assert(sizeof(std::declval<node>()) == 8);

  struct label {
    label(node const n, cost_t const c)
        : n_{n.n_},
          cost_{c},
          type_{n.type_},
          lvl_{n.lvl_},
          dir_{n.dir_},
          way_(n.way_) {}

    constexpr node get_node() const noexcept {
      return {
          .n_ = n_, .type_ = type_, .lvl_ = lvl_, .dir_ = dir_, .way_ = way_};
    }

    constexpr cost_t cost() const noexcept { return cost_; }

    void track(
        label const&, ways::routing const&, way_idx_t, node_idx_t, bool) {}

    node_idx_t n_;
    cost_t cost_;
    node_type type_;
    level_t lvl_;
    direction dir_;
    way_pos_t way_;
  };

  struct entry {
    static constexpr auto const kMaxWays = way_pos_t{16U};
    static constexpr auto const kN = kMaxWays * 2U + 1 /* FWD+BWD + foot */;

    entry() {
      utl::fill(cost_, kInfeasible);
      utl::fill(pred_, node_idx_t::invalid());
      utl::fill(pred_way_, way_pos_t{0});
      utl::fill(pred_lvl_, kNoLevel);
    }

    constexpr std::optional<node> pred(node const n) const noexcept {
      auto const idx = get_index(n);
      return pred_[idx] == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{
                       node{.n_ = pred_[idx],
                            .type_ = pred_parking_[idx]
                                         ? node_type::kParking
                                         : to_node_type(pred_type_[idx]),
                            .lvl_ = pred_lvl_[idx],
                            .dir_ = to_dir(pred_dir_[idx]),
                            .way_ = pred_way_[idx]}};
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
        pred_type_[idx] = to_bool(pred.type_);
        pred_way_[idx] = pred.way_;
        pred_dir_[idx] = to_bool(pred.dir_);
        pred_parking_[idx] = pred.type_ == node_type::kParking;
        return true;
      }
      return false;
    }

    static constexpr std::size_t get_index(node const n) {
      return n.is_foot_node()
                 ? 0U
                 : 1U + (n.dir_ == direction::kForward ? 0U : 1U) * kMaxWays +
                       n.way_;
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

    void write(node, path&) const {}

    std::array<node_idx_t, kN> pred_;
    std::array<cost_t, kN> cost_;
    std::array<way_pos_t, kN> pred_way_;
    std::array<level_t, kN> pred_lvl_;
    std::bitset<kN> pred_dir_;
    std::bitset<kN> pred_type_;
    std::bitset<kN> pred_parking_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(key const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::hash(static_cast<std::uint64_t>(to_idx(n)));
    }
  };

  static car::node to_car(node const n) {
    return {.n_ = n.n_, .way_ = n.way_, .dir_ = n.dir_};
  }

  static footp::node to_foot(node const n) {
    return {.n_ = n.n_, .lvl_ = n.lvl_};
  }

  static node to_node(car::node const n, level_t const lvl) {
    return {.n_ = n.n_,
            .type_ = node_type::kCar,
            .lvl_ = lvl,
            .dir_ = n.dir_,
            .way_ = n.way_};
  }

  static node to_node(footp::node const n) {
    return {.n_ = n.n_,
            .type_ = node_type::kFoot,
            .lvl_ = n.lvl_,
            .dir_ = direction::kForward,
            .way_ = 0};
  }

  static node create_node(node_idx_t const n,
                          level_t const lvl,
                          way_pos_t const way,
                          direction const dir) {
    return node{n, node_type::kInvalid, lvl, dir, way};
  }

  static node encode_parking_node(parking_edge_idx_t const parking_edge_idx,
                                  level_t const lvl,
                                  direction const dir,
                                  unsigned const car_offset,
                                  unsigned const foot_offset) {
    utl::verify(car_offset < 2, "Invalid car offset {}", car_offset);
    utl::verify(foot_offset < 2, "Invalid foot offset {}", foot_offset);
    return {node_idx_t{to_idx(parking_edge_idx)}, node_type::kParking, lvl, dir,
            static_cast<way_pos_t>((2 * car_offset) + foot_offset)};
  }
  static node decode_parking_node(ways::routing const& w, node const& n) {
    auto const parking_edge =
        w.parking_edges_[parking_edge_idx_t{to_idx(n.n_)}];
    if (n.dir_ == direction::kForward) {
      auto const foot_offset = n.way_ & 0x01;
      auto const node_idx =
          foot_offset == 0 ? parking_edge.foot_left_ : parking_edge.foot_right_;
      fmt::println(
          "Decode Forward: parking_edge: {}, foot_offset: {}, node: {}", n.n_,
          foot_offset, node_idx);
      return {node_idx, node_type::kFoot, n.lvl_, n.dir_, 0U};
    } else {
      auto const car_offset = (n.way_ >> 1) & 0x01;
      auto const node_idx =
          car_offset == 0 ? parking_edge.car_left_ : parking_edge.car_right_;
      fmt::println(
          "Decode Backwards: parking_edge: {}, car_offset: {}, node: {}", n.n_,
          car_offset, node_idx);
      return {node_idx, node_type::kCar, n.lvl_, n.dir_, 0U};
    }
  }

  template <typename Fn>
  static void resolve_all(ways::routing const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    footp::resolve_all(
        w, n, lvl, [&](footp::node const neighbor) { f(to_node(neighbor)); });
    car::resolve_all(w, n, lvl, [&](car::node const neighbor) {
      auto const p = w.way_properties_[w.node_ways_[n][neighbor.way_]];
      auto const node_level = lvl == kNoLevel ? p.from_level() : lvl;
      f(to_node(neighbor, node_level));
    });
  }

  template <direction SearchDir, bool WithBlocked, typename Fn>
  static void adjacent(parameters const& params,
                       ways::routing const& w,
                       node const n,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const*,
                       elevation_storage const* elevations,
                       Fn&& fn) {
    static constexpr auto const kFwd = SearchDir == direction::kForward;
    static constexpr auto const kBwd = SearchDir == direction::kBackward;

    if (n.is_parking_node()) {
      auto const n2 = decode_parking_node(w, n);
      fmt::println("DECODED: (n: {} way: {})  ->  (n: {})", n.n_, n.way_,
                   n2.n_);
      adjacent<SearchDir, WithBlocked>(params, w, n2, blocked, nullptr,
                                       elevations, fn);
      return;
      // auto const& parking_edge = w.parking_edges_[to_parking_edge_idx(n.n_)];
      // if (kFwd) {
      //   adjacent<SearchDir, WithBlocked>(params, w, n.from_parking_edge(),
      //                                    blocked, nullptr, elevations, fn);
      //   // // TODO: costs - Add offset costs
      //   // for (auto const foot_node :
      //   //      {parking_edge.foot_left_, parking_edge.foot_right_}) {
      //   //   if (foot_node == node_idx_t::invalid()) {
      //   //     continue;
      //   //   }
      //   //   fn(node{foot_node, node_type::kFoot,
      //   //           w.node_properties_[foot_node].from_level(),
      //   //           direction::kForward, 0},
      //   //      kSwitchPenalty, 0, way_idx_t::invalid(), 0, 0,
      //   //      elevation_storage::elevation{}, false);
      //   // }
      // } else if (kBwd) {
      //   adjacent<SearchDir, WithBlocked>(params, w, n.from_parking_edge(),
      //                                    blocked, nullptr, elevations, fn);
      // }
      // return;
    }

    auto const is_parking =
        !UseParking || w.node_properties_[n.n_].is_parking() ||
        utl::any_of(w.node_ways_[n.n_], [&](way_idx_t const way) {
          return w.way_properties_[way].is_parking();
        });

    if (n.is_foot_node() || (kFwd && n.is_car_node() && is_parking)) {
      footp::template adjacent<SearchDir, WithBlocked>(
          params.foot_, w, to_foot(n), blocked, nullptr, elevations,
          [&](footp::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            fn(to_node(neighbor),
               cost + (n.is_foot_node() ? 0 : kSwitchPenalty), dist, way, from,
               to, elevation, false);
          });
    }

    if (n.is_car_node() || (kBwd && n.is_foot_node() && is_parking)) {
      car::template adjacent<SearchDir, WithBlocked>(
          params.car_, w, to_car(n), blocked, nullptr, elevations,
          [&](car::node const neighbor, std::uint32_t const cost,
              distance_t const dist, way_idx_t const way,
              std::uint16_t const from, std::uint16_t const to,
              elevation_storage::elevation const elevation, bool) {
            auto const way_prop = w.way_properties_[way];
            fn(to_node(neighbor, way_prop.from_level()),
               cost + (n.is_car_node() ? 0 : kSwitchPenalty), dist, way, from,
               to, elevation, false);
          });
    }

    if constexpr (UseParking) {
      if (kFwd && n.is_car_node()) {
        if (w.has_parking_edges_.test(n.n_)) {
          for_each_parking_edge(
              w, n.n_, [&](parking_edge_idx_t const parking_edge_idx) {
                auto const& parking_edge = w.parking_edges_[parking_edge_idx];
                for (auto car_offset = 0U; car_offset < 2U; ++car_offset) {
                  auto const car_node = car_offset == 0
                                            ? parking_edge.car_left_
                                            : parking_edge.car_right_;
                  if (car_node != n.n_) {
                    continue;
                  }
                  for (auto foot_offset = 0U; foot_offset < 2U; ++foot_offset) {
                    auto const foot_node = foot_offset == 0
                                               ? parking_edge.foot_left_
                                               : parking_edge.foot_right_;
                    if (foot_node == node_idx_t::invalid()) {
                      continue;
                    }
                    auto const encoded = encode_parking_node(
                        parking_edge_idx,
                        w.node_properties_[foot_node].from_level(), SearchDir,
                        car_offset, foot_offset);
                    fmt::println(
                        "ENCODED: (n: {} car: {} / {}, foot: {} / {})  ->  (n: "
                        "{} way: {})",
                        n.n_, car_node, car_offset, foot_node, foot_offset,
                        encoded.n_, encoded.way_);
                    auto const cost = kSwitchPenalty;  // TODO:offset- Add costs
                    fn(std::move(encoded), cost, 0, way_idx_t::invalid(), 0, 0,
                       elevation_storage::elevation{}, false);
                  }
                }
                // // for (auto const [car_offset, car_node] :
                // //      {{0, parking_edge.car_left_},
                // //       {1, parking_edge.car_right_}}) {
                // // }
                // // TODO Keep start check here
                // if (parking_edge.car_left_ != n.n_ &&
                //     parking_edge.car_right_ != n.n_) {
                //   return;
                // }
                // for (auto const foot_node :
                //      {parking_edge.foot_left_, parking_edge.foot_right_}) {
                //   if (foot_node == node_idx_t::invalid()) {
                //     continue;
                //   }
                //   auto const way = way_idx_t::invalid();  // TODO Use way_idx
                //                                           // of parking area
                //                                           ?
                //   fn(node{foot_node, node_type::kFoot,
                //           w.node_properties_[foot_node].from_level(),
                //           direction::kForward, 0},
                //      kSwitchPenalty, 0, way, 0, 0,
                //      elevation_storage::elevation{}, false);
                // }
              });
        }
      }
      if (kBwd && n.is_foot_node()) {
        fmt::println("DEBUG DEBUG -- BACKWARDS");
        if (w.has_parking_edges_.test(n.n_)) {
          for_each_parking_edge(
              w, n.n_, [&](parking_edge_idx_t const parking_edge_idx) {
                auto const& parking_edge = w.parking_edges_[parking_edge_idx];
                if (parking_edge.foot_left_ != n.n_ &&
                    parking_edge.foot_right_ != n.n_) {
                  return;
                }
                for (auto const car_node :
                     {parking_edge.car_left_, parking_edge.car_right_}) {
                  if (car_node == node_idx_t::invalid()) {
                    continue;
                  }
                  auto const way = way_idx_t::invalid();  // TODO Use way_idx
                                                          // of parking area ?
                  fn(node{car_node, node_type::kCar, level_t{0.0F},
                          direction::kBackward, 0},
                     kSwitchPenalty, 0, way, 0, 0,
                     elevation_storage::elevation{}, false);
                }
              });
        }
      }
      // TODO
      // If forward && is_extra_node && extra_node.car_* == n
      // => Add edge n -> extra -> foot_* (<= 2 paths)
    }
  }

  template <typename Fn>
  static void resolve_start_node(ways::routing const& w,
                                 way_idx_t const way,
                                 node_idx_t const n,
                                 level_t lvl,
                                 direction search_dir,
                                 Fn&& f) {
    auto const way_properties = w.way_properties_[way];
    search_dir == direction::kForward
        ? car::resolve_start_node(
              w, way, n, lvl, search_dir,
              [&](car::node const cn) {
                auto const node_level =
                    lvl == kNoLevel ? way_properties.from_level() : lvl;
                f(to_node(cn, node_level));
              })
        : footp::resolve_start_node(
              w, way, n, lvl, search_dir,
              [&](footp::node const fn) { f(to_node(fn)); });
  }

  static bool is_dest_reachable(parameters const& params,
                                ways::routing const& w,
                                node const n,
                                way_idx_t const way,
                                direction const way_dir,
                                direction const search_dir) {
    return !UseParking || w.way_properties_[way].is_parking() ||
           (search_dir == direction::kForward
                ? n.is_foot_node() &&
                      footp::is_dest_reachable(params.foot_, w, to_foot(n), way,
                                               way_dir, search_dir)
                : n.is_car_node() &&
                      car::is_dest_reachable(params.car_, w, to_car(n), way,
                                             way_dir, search_dir));
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
    return car::lower_bound_heuristic(params.car_, dist);
  }

  static constexpr double upper_bound_heuristic(parameters const& params,
                                                double const dist) {
    return car::upper_bound_heuristic(params.car_, dist);
  }

  static constexpr node get_reverse(node n) {
    return {n.n_, n.type_, n.lvl_, opposite(n.dir_), n.way_};
  }
};

}  // namespace osr
