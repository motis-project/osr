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

#include "osr/conditional.h"
#include "osr/point.h"
#include "osr/routing/turns.h"
#include "osr/types.h"
#include "osr/util/multi_counter.h"

namespace osr {

struct resolved_restriction {
  enum class type { kNo, kOnly } type_;
  way_idx_t from_, to_;
  node_idx_t via_;
  bool applies_to_default_{true};
  bool applies_to_bus_{true};
  bool applies_to_hgv_{true};
  conditional_condition_set_idx_t condition_set_{
      conditional_condition_set_idx_t::invalid()};
};

struct restriction {
  friend bool operator==(restriction, restriction) = default;

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      restriction const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("restriction v1.1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, restriction const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, restriction*) {}

  way_pos_t from_ : 4;
  way_pos_t to_ : 4;
  way_pos_t applies_to_default_ : 1;
  way_pos_t applies_to_bus_ : 1;
  way_pos_t applies_to_hgv_ : 1;
  conditional_condition_set_idx_t condition_set_{
      conditional_condition_set_idx_t::invalid()};
};

enum class hgv_info_field : std::uint16_t {
  kAccessFwd = 1U << 0U,
  kAccessBwd = 1U << 1U,
  kMaxSpeed = 1U << 2U,
  kMaxLength = 1U << 3U,
  kMaxWeightRating = 1U << 4U,
  kMaxHeight = 1U << 5U,
  kMaxWidth = 1U << 6U,
  kMaxWeight = 1U << 7U,
  kMaxAxleLoad = 1U << 8U,
  kMaxAxles = 1U << 9U,
  kHazmat = 1U << 10U,
  kHazmatWater = 1U << 11U,
  kTrailer = 1U << 12U,
};

constexpr std::uint16_t to_mask(hgv_info_field const field) {
  return static_cast<std::uint16_t>(field);
}

struct hgv_way_info {
  friend bool operator==(hgv_way_info, hgv_way_info) = default;

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      hgv_way_info const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("hgv_way_info v1"));
  }

  template <typename Ctx>
  friend void serialize(Ctx&, hgv_way_info const*, cista::offset_t) {}

  template <typename Ctx>
  friend void deserialize(Ctx const&, hgv_way_info*) {}

  constexpr bool has(hgv_info_field const field) const {
    return (fields_ & to_mask(field)) != 0U;
  }

  constexpr access_value hgv_access_fwd() const {
    return static_cast<access_value>(hgv_access_fwd_);
  }

  constexpr access_value hgv_access_bwd() const {
    return static_cast<access_value>(hgv_access_bwd_);
  }

  constexpr access_value hazmat_access() const {
    return static_cast<access_value>(hazmat_access_);
  }

  constexpr access_value hazmat_water_access() const {
    return static_cast<access_value>(hazmat_water_access_);
  }

  constexpr access_value trailer_access() const {
    return static_cast<access_value>(trailer_access_);
  }

  std::uint16_t fields_{0U};
  std::uint8_t hgv_access_fwd_{
      static_cast<std::uint8_t>(access_value::kUnknown)};
  std::uint8_t hgv_access_bwd_{
      static_cast<std::uint8_t>(access_value::kUnknown)};
  std::uint8_t hazmat_access_{
      static_cast<std::uint8_t>(access_value::kUnknown)};
  std::uint8_t hazmat_water_access_{
      static_cast<std::uint8_t>(access_value::kUnknown)};
  std::uint8_t trailer_access_{
      static_cast<std::uint8_t>(access_value::kUnknown)};
  std::uint8_t maxspeed_km_h_{0U};
  std::uint8_t maxaxles_{0U};
  std::uint16_t maxlength_cm_{0U};
  std::uint16_t maxweightrating_100kg_{0U};
  std::uint16_t maxheight_cm_{0U};
  std::uint16_t maxwidth_cm_{0U};
  std::uint16_t maxweight_100kg_{0U};
  std::uint16_t maxaxleload_100kg_{0U};
};

struct way_properties {
  constexpr bool is_accessible() const {
    return is_car_accessible() || is_bike_accessible() ||
           is_foot_accessible() || is_bus_accessible() ||
           is_bus_accessible_with_penalty() || is_railway_accessible() ||
           is_railway_accessible_with_penalty() || is_ferry_accessible() ||
           has_hgv_info() || has_conditionals();
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
  constexpr bool is_railway_accessible_with_penalty() const {
    return is_railway_accessible_with_penalty_;
  }
  constexpr bool is_ferry_accessible() const { return is_ferry_accessible_; }
  constexpr bool is_big_street() const { return is_big_street_; }
  constexpr bool is_destination() const { return is_destination_; }
  constexpr bool is_oneway_car() const { return is_oneway_car_; }
  constexpr bool is_oneway_bike() const { return is_oneway_bike_; }
  constexpr bool is_oneway_bus_psv() const { return is_oneway_bus_psv_; }
  constexpr bool is_oneway_reverse() const { return is_oneway_reverse_; }
  constexpr bool is_oneway_direction_allowed(bool const is_oneway,
                                             direction const dir) const {
    return !is_oneway || dir == (is_oneway_reverse_ ? direction::kBackward
                                                    : direction::kForward);
  }
  constexpr bool is_car_direction_allowed(direction const dir) const {
    return is_oneway_direction_allowed(is_oneway_car(), dir);
  }
  constexpr bool is_bike_direction_allowed(direction const dir) const {
    return is_oneway_direction_allowed(is_oneway_bike(), dir);
  }
  constexpr bool is_bus_psv_direction_allowed(direction const dir) const {
    return is_oneway_direction_allowed(is_oneway_bus_psv(), dir);
  }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_steps() const { return is_steps_; }
  constexpr bool is_ramp() const { return is_ramp_; }
  constexpr bool is_parking() const { return is_parking_; }
  constexpr bool has_toll() const { return has_toll_; }
  constexpr bool is_sidewalk_separate() const { return is_sidewalk_separate_; }
  constexpr bool in_route() const { return in_route_; }
  constexpr bool has_hgv_info() const { return has_hgv_info_; }
  constexpr bool has_conditionals() const { return has_conditionals_; }
  constexpr bool is_in_low_emission_zone() const {
    return is_in_low_emission_zone_;
  }
  constexpr bool is_detour() const { return is_detour_; }
  constexpr std::uint16_t max_speed_km_per_h() const {
    return to_kmh(static_cast<speed_limit>(speed_limit_));
  }
  constexpr float max_speed_s_per_m() const {
    return to_seconds_per_meter(static_cast<speed_limit>(speed_limit_));
  }
  constexpr level_t from_level() const { return level_t{from_level_}; }
  constexpr level_t to_level() const { return level_t{to_level_}; }

  template <std::size_t NMaxTypes>
  friend constexpr auto static_type_hash(
      way_properties const*, cista::hash_data<NMaxTypes> h) noexcept {
    return h.combine(cista::hash("way_properties v1.2"));
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
  std::uint8_t is_oneway_bus_psv_ : 1;
  std::uint8_t is_incline_down_ : 1;
  std::uint8_t is_bus_accessible_with_penalty_ : 1;
  std::uint8_t is_ferry_accessible_ : 1;
  std::uint8_t is_railway_accessible_with_penalty_ : 1;
  std::uint8_t has_hgv_info_ : 1;
  std::uint8_t has_conditionals_ : 1;
  std::uint8_t is_in_low_emission_zone_ : 1;
  std::uint8_t is_detour_ : 1;
  std::uint8_t is_oneway_reverse_ : 1;
};

static_assert(sizeof(way_properties) == 6);

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
  void compute_turn_bearings();
  void build_components();

  std::optional<way_idx_t> find_way(osm_way_idx_t const i) {
    auto const it = std::lower_bound(
        begin(way_osm_idx_), end(way_osm_idx_), i,
        [](auto const a, auto const b) { return osm_id_less(a, b); });
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

  std::size_t get_polyline_node_idx(
      way_idx_t const way, std::uint16_t const target_routing_idx) const;

  std::optional<std::int64_t> get_osm_node(node_idx_t const n) const {
    return n != node_idx_t::invalid() && n < n_nodes()
               ? std::optional{static_cast<std::int64_t>(
                     to_idx(node_to_osm_[n]))}
               : std::nullopt;
  }

  std::optional<std::int64_t> get_osm_way(way_idx_t const way) const {
    return way != way_idx_t::invalid()
               ? std::optional{static_cast<std::int64_t>(
                     to_idx(way_osm_idx_[way]))}
               : std::nullopt;
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

    template <direction SearchDir, bool IsBus = false, bool IsHgv = false>
    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to) const {
      if (!node_is_restricted_[n]) {
        return false;
      }

      auto const r = node_restrictions_[n];
      auto const from_way =
          SearchDir == direction::kForward ? way_pos_t{from} : way_pos_t{to};
      auto const to_way =
          SearchDir == direction::kForward ? way_pos_t{to} : way_pos_t{from};

      return utl::any_of(r, [&](restriction const& x) {
        if constexpr (IsBus) {
          return x.from_ == from_way && x.to_ == to_way && x.applies_to_bus_ &&
                 x.condition_set_ == conditional_condition_set_idx_t::invalid();
        } else if constexpr (IsHgv) {
          return x.from_ == from_way && x.to_ == to_way && x.applies_to_hgv_ &&
                 x.condition_set_ == conditional_condition_set_idx_t::invalid();
        } else {
          return x.from_ == from_way && x.to_ == to_way &&
                 x.applies_to_default_ &&
                 x.condition_set_ == conditional_condition_set_idx_t::invalid();
        }
      });
    }

    template <bool IsBus = false, bool IsHgv = false>
    bool is_restricted(node_idx_t const n,
                       std::uint8_t const from,
                       std::uint8_t const to,
                       direction const search_dir) const {
      return search_dir == direction::kForward
                 ? is_restricted<direction::kForward, IsBus, IsHgv>(n, from, to)
                 : is_restricted<direction::kBackward, IsBus, IsHgv>(n, from,
                                                                     to);
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

    quantized_angle_t get_turn_angle(node_idx_t const n,
                                     way_pos_t const from,
                                     direction const from_dir,
                                     way_pos_t const to,
                                     direction const to_dir) const {
      return osr::get_turn_angle(node_turn_bearings_[n][from], from_dir,
                                 node_turn_bearings_[n][to], to_dir);
    }

    static cista::wrapped<routing> read(std::filesystem::path const&);
    void write(std::filesystem::path const&) const;

    hgv_way_info const* get_hgv_info(way_idx_t const way) const {
      if (!way_properties_[way].has_hgv_info()) {
        return nullptr;
      }
      auto const it = std::lower_bound(
          begin(way_hgv_info_), end(way_hgv_info_), way,
          [](auto const& entry, auto const& key) { return entry.first < key; });
      utl::verify(it != end(way_hgv_info_) && it->first == way,
                  "missing HGV info for way {}", way);
      return &it->second;
    }

    way_conditional_restrictions const* get_conditional_restrictions(
        way_idx_t const way) const {
      if (!way_properties_[way].has_conditionals()) {
        return nullptr;
      }
      auto const it = std::lower_bound(
          begin(way_conditionals_), end(way_conditionals_), way,
          [](auto const& entry, auto const& key) { return entry.first < key; });
      utl::verify(it != end(way_conditionals_) && it->first == way,
                  "missing conditional restrictions for way {}", way);
      return &it->second;
    }

    struct long_distance {
      CISTA_COMPARABLE()

      way_idx_t way_{};
      std::uint16_t node_{};
      distance_t distance_{};
    };

    vec_map<node_idx_t, node_properties> node_properties_;
    vec_map<way_idx_t, way_properties> way_properties_;
    vec<pair<way_idx_t, hgv_way_info>> way_hgv_info_;

    vec<pair<way_idx_t, way_conditional_restrictions>> way_conditionals_;
    vec<conditional_access_restriction> conditional_access_;
    vec<conditional_oneway_restriction> conditional_oneway_;
    vec<conditional_numeric_restriction> conditional_numeric_;

    vec<conditional_condition_set> conditional_condition_sets_;
    vec<conditional_condition> conditional_conditions_;
    vec<opening_hours> opening_hours_;
    vec<opening_hours_rule> opening_hours_rules_;
    vec<opening_hours_year_range> opening_hours_year_ranges_;
    vec<opening_hours_week_range> opening_hours_week_ranges_;
    vec<opening_hours_monthday_range> opening_hours_monthday_ranges_;
    vec<opening_hours_weekday_range> opening_hours_weekday_ranges_;
    vec<opening_hours_time_span> opening_hours_time_spans_;
    vecvec<conditional_timezone_idx_t, char> timezones_;

    vecvec<way_idx_t, node_idx_t> way_nodes_;
    vecvec<way_idx_t, std::uint16_t> way_node_dist_;
    vec<long_distance> long_way_node_dist_;

    vecvec<node_idx_t, way_idx_t> node_ways_;
    vecvec<node_idx_t, std::uint16_t> node_in_way_idx_;
    vecvec<node_idx_t, turn_bearing> node_turn_bearings_;

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

  multi_counter<> node_way_counter_;
};

}  // namespace osr
