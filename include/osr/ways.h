#pragma once

#if defined(_WIN32) || defined(_WIN64)
#include "windows.h"

#include "Memoryapi.h"
#define mlock(addr, size) VirtualLock((LPVOID)addr, (SIZE_T)size)
#else
#include <sys/mman.h>
#endif
#include <filesystem>
#include <ranges>

#include "fmt/ranges.h"
#include "fmt/std.h"

#include "osmium/osm/way.hpp"

#include "cista/memory_holder.h"
#include "cista/reflection/comparable.h"

#include "utl/enumerate.h"
#include "utl/equal_ranges_linear.h"
#include "utl/helpers/algorithm.h"
#include "utl/progress_tracker.h"
#include "utl/timer.h"
#include "utl/verify.h"
#include "utl/zip.h"

#include "osr/point.h"
#include "osr/types.h"
#include "osr/util/multi_counter.h"

namespace osr {

struct resolved_restriction {
  enum class type { kNo, kOnly } type_;
  way_idx_t from_, to_;
  node_idx_t via_;
};

struct restriction {
  friend bool operator==(restriction, restriction) = default;
  way_pos_t from_, to_;
};

struct way_properties {
  constexpr bool is_accessible() const {
    return is_car_accessible() || is_bike_accessible() ||
           is_foot_accessible() || is_bus_accessible() ||
           is_bus_accessible_with_penalty() || is_railway_accessible() ||
           is_ferry_accessible();
  }
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_foot_accessible() const { return is_foot_accessible_; }
  constexpr bool is_bus_accessible() const { return is_bus_accessible_; }
  constexpr bool is_bus_accessible_with_penalty() const {
    return is_bus_accessible_with_penalty_;
  }
  constexpr bool is_railway_accessible() const {
    return is_railway_accessible_;
  }
  constexpr bool is_ferry_accessible() const { return is_ferry_accessible_; }
  constexpr bool is_big_street() const { return is_big_street_; }
  constexpr bool is_destination() const { return is_destination_; }
  constexpr bool is_oneway_car() const { return is_oneway_car_; }
  constexpr bool is_oneway_bike() const { return is_oneway_bike_; }
  constexpr bool is_oneway_psv() const { return is_oneway_psv_; }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_steps() const { return is_steps_; }
  constexpr bool is_ramp() const { return is_ramp_; }
  constexpr bool is_parking() const { return is_parking_; }
  constexpr bool has_toll() const { return has_toll_; }
  constexpr bool is_sidewalk_separate() const { return is_sidewalk_separate_; }
  constexpr bool in_route() const { return in_route_; }
  constexpr std::uint16_t max_speed_m_per_s() const {
    return to_meters_per_second(static_cast<speed_limit>(speed_limit_));
  }
  constexpr std::uint16_t max_speed_km_per_h() const {
    return to_kmh(static_cast<speed_limit>(speed_limit_));
  }
  constexpr level_t from_level() const { return level_t{from_level_}; }
  constexpr level_t to_level() const { return level_t{to_level_}; }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      way_properties const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("way_properties v1.1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, way_properties const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, way_properties*) {}

  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_destination_ : 1;
  std::uint8_t is_oneway_car_ : 1;
  std::uint8_t is_oneway_bike_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_steps_ : 1;

  std::uint8_t speed_limit_ : 3;
  std::uint8_t is_platform_ : 1;  // only used during extract
  std::uint8_t is_parking_ : 1;
  std::uint8_t is_ramp_ : 1;
  std::uint8_t is_sidewalk_separate_ : 1;
  std::uint8_t motor_vehicle_no_ : 1;

  std::uint8_t from_level_ : 6;
  std::uint8_t has_toll_ : 1;
  std::uint8_t is_big_street_ : 1;

  std::uint8_t to_level_ : 6;
  std::uint8_t is_bus_accessible_ : 1;
  std::uint8_t in_route_ : 1;

  std::uint8_t is_railway_accessible_ : 1;
  std::uint8_t is_oneway_psv_ : 1;
  std::uint8_t is_incline_down_ : 1;
  std::uint8_t is_bus_accessible_with_penalty_ : 1;
  std::uint8_t is_ferry_accessible_ : 1;
};

static_assert(sizeof(way_properties) == 5);

struct node_properties {
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_walk_accessible() const { return is_foot_accessible_; }
  constexpr bool is_bus_accessible() const { return is_bus_accessible_; }
  constexpr bool is_bus_accessible_with_penalty() const {
    return is_bus_accessible_with_penalty_;
  }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_multi_level() const { return is_multi_level_; }
  constexpr bool is_entrance() const { return is_entrance_; }
  constexpr bool is_parking() const { return is_parking_; }

  constexpr level_t from_level() const { return level_t{from_level_}; }
  constexpr level_t to_level() const { return level_t{to_level_}; }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      node_properties const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("node_properties v1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, node_properties const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, node_properties*) {}

  std::uint8_t from_level_ : 6;
  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;

  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_bus_accessible_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_entrance_ : 1;
  std::uint8_t is_multi_level_ : 1;
  std::uint8_t is_parking_ : 1;

  std::uint8_t to_level_ : 6;
  std::uint8_t is_bus_accessible_with_penalty_ : 1;
};

static_assert(sizeof(node_properties) == 3);

struct ways {
  ways(std::filesystem::path, cista::mmap::protection);

  void add_restriction(std::vector<resolved_restriction>&);
  void compute_big_street_neighbors();
  void connect_ways();
  void build_components();

  std::optional<way_idx_t> find_way(osm_way_idx_t const i) {
    auto const it = std::lower_bound(begin(way_osm_idx_), end(way_osm_idx_), i);
    return it != end(way_osm_idx_) && *it == i
               ? std::optional{way_idx_t{
                     std::distance(begin(way_osm_idx_), it)}}
               : std::nullopt;
  }

  bool is_additional_node(osr::node_idx_t const n) const {
    return n != node_idx_t::invalid() && n >= n_nodes();
  }

  std::optional<node_idx_t> find_node_idx(osm_node_idx_t const i) const {
    auto const it = std::lower_bound(begin(node_to_osm_), end(node_to_osm_), i,
                                     [](auto&& a, auto&& b) { return a < b; });
    if (it == end(node_to_osm_) || *it != i) {
      return std::nullopt;
    }
    return {node_idx_t{static_cast<node_idx_t::value_t>(
        std::distance(begin(node_to_osm_), it))}};
  }

  node_idx_t get_node_idx(osm_node_idx_t const i) const {
    auto const j = find_node_idx(i);
    utl::verify(j.has_value(), "osm node {} not found", i);
    return *j;
  }

  point get_node_pos(node_idx_t const i) const {
    return r_->node_positions_.at(i);
  }

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  void sync();

  way_idx_t::value_t n_ways() const { return way_osm_idx_.size(); }
  node_idx_t::value_t n_nodes() const { return node_to_osm_.size(); }

  std::optional<std::string_view> get_access_restriction(way_idx_t) const;

  std::filesystem::path p_;
  cista::mmap::protection mode_;

  struct routing {
    static constexpr auto const kMode =
        cista::mode::WITH_INTEGRITY | cista::mode::WITH_STATIC_VERSION;

    way_pos_t get_way_pos(node_idx_t const node, way_idx_t const way) const {
      auto const ways = node_ways_[node];
      for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
        if (ways[i] == way) {
          return i;
        }
      }
      return 0U;
    }

    way_pos_t get_way_pos(node_idx_t const node,
                          way_idx_t const way,
                          std::uint16_t const node_in_way_idx) const {
      auto const ways = node_ways_[node];
      for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
        if (ways[i] == way &&
            (i + 1U == ways.size() || ways[i] != ways[i + 1U] ||
             node_in_way_idx_[node][i] == node_in_way_idx)) {
          return i;
        }
      }
      return 0U;
    }

    template <direction SearchDir>
    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to) const {
      if (!node_is_restricted_[n]) {
        return false;
      }
      auto const r = node_restrictions_[n];
      auto const needle = SearchDir == direction::kForward
                              ? restriction{from, to}
                              : restriction{to, from};
      return utl::find(r, needle) != end(r);
    }

    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to,
                       direction const search_dir) const {
      return search_dir == direction::kForward
                 ? is_restricted<direction::kForward>(n, from, to)
                 : is_restricted<direction::kBackward>(n, from, to);
    }

    bool is_loop(way_idx_t const w) const {
      return way_nodes_[w].back() == way_nodes_[w].front();
    }

    distance_t get_way_node_distance(way_idx_t const way,
                                     std::uint16_t const node) const {
      auto const v = way_node_dist_[way][node];
      if (v != std::numeric_limits<std::uint16_t>::max()) {
        [[likely]] return distance_t{v};
      } else {
        auto const it = std::lower_bound(begin(long_way_node_dist_),
                                         end(long_way_node_dist_),
                                         long_distance{way, node, 0U});
        if (it != end(long_way_node_dist_) && it->way_ == way &&
            it->node_ == node) {
          return it->distance_;
        }
        throw utl::fail("long distance not found for way {} node {}",
                        to_idx(way), node);
      }
    }

    static cista::wrapped<routing> read(std::filesystem::path const&);
    void write(std::filesystem::path const&) const;

    struct long_distance {
      CISTA_COMPARABLE()

      way_idx_t way_{};
      std::uint16_t node_{};
      distance_t distance_{};
    };

    vec_map<node_idx_t, node_properties> node_properties_;
    vec_map<way_idx_t, way_properties> way_properties_;

    vecvec<way_idx_t, node_idx_t> way_nodes_;
    vecvec<way_idx_t, std::uint16_t> way_node_dist_;
    vec<long_distance> long_way_node_dist_;

    vecvec<node_idx_t, way_idx_t> node_ways_;
    vecvec<node_idx_t, std::uint16_t> node_in_way_idx_;

    bitvec<node_idx_t> node_is_restricted_;
    vecvec<node_idx_t, restriction> node_restrictions_;

    vec_map<node_idx_t, point> node_positions_;

    vec<pair<node_idx_t, level_bits_t>> multi_level_elevators_;

    vec_map<way_idx_t, component_idx_t> way_component_;
  };

  cista::wrapped<routing> r_;

  mm_vec_map<node_idx_t, osm_node_idx_t> node_to_osm_;
  mm_vec_map<way_idx_t, osm_way_idx_t> way_osm_idx_;
  mm_vecvec<way_idx_t, point, std::uint64_t> way_polylines_;
  mm_vecvec<way_idx_t, osm_node_idx_t, std::uint64_t> way_osm_nodes_;
  mm_vecvec<string_idx_t, char, std::uint64_t> strings_;
  mm_vec_map<way_idx_t, string_idx_t> way_names_;

  mm_bitvec<way_idx_t> way_has_conditional_access_no_;
  mm_vec<pair<way_idx_t, string_idx_t>> way_conditional_access_no_;

  multi_counter node_way_counter_;
};

}  // namespace osr
