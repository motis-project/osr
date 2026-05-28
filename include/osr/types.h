#pragma once

#include <cinttypes>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <limits>
#include <optional>

#include "ankerl/cista_adapter.h"

#include "cista/containers/bitvec.h"
#include "cista/containers/mmap_vec.h"
#include "cista/containers/nvec.h"
#include "cista/containers/paged.h"
#include "cista/containers/paged_vecvec.h"
#include "cista/containers/vector.h"
#include "cista/containers/vecvec.h"
#include "cista/strong.h"

#include "utl/for_each_bit_set.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename T>
using vec = cista::raw::vector<T>;

template <typename K, typename V>
using vec_map = cista::raw::vector_map<K, V>;

template <typename K>
using bitvec = cista::basic_bitvec<vec<std::uint64_t>, K>;

template <typename K, typename V>
using mm_vec_map = cista::basic_mmap_vec<V, K>;

template <typename T>
using mm_vec = cista::basic_mmap_vec<T, std::uint64_t>;

template <typename T>
using mm_vec32 = cista::basic_mmap_vec<T, std::uint32_t>;

template <typename K>
using mm_bitvec = cista::basic_bitvec<mm_vec<std::uint64_t>, K>;

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using mm_vecvec = cista::basic_vecvec<K, mm_vec<V>, mm_vec<SizeType>>;

template <typename K, typename V, std::size_t N>
using mm_nvec =
    cista::basic_nvec<K, mm_vec<V>, mm_vec<std::uint64_t>, N, std::uint64_t>;

template <typename Key, typename T>
struct mmap_paged_vecvec_helper {
  using data_t = cista::paged<mm_vec32<T>>;
  using idx_t = mm_vec<typename data_t::page_t>;
  using type = cista::paged_vecvec<idx_t, data_t, Key>;
};

template <typename Key, typename T>
using mm_paged_vecvec = mmap_paged_vecvec_helper<Key, T>::type;

using bitvec64 = cista::basic_bitvec<
    cista::basic_vector<std::uint64_t, cista::raw::ptr, false, std::uint64_t>>;

template <typename First, typename Second>
using pair = cista::pair<First, Second>;

template <typename K,
          typename V,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_map = cista::raw::ankerl_map<K, V, Hash, Equality>;

template <typename K,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_set = cista::raw::ankerl_set<K, Hash, Equality>;

using string_idx_t = cista::strong<std::uint32_t, struct string_idx_>;

using osm_node_idx_t = cista::strong<std::uint64_t, struct osm_node_idx_>;
using osm_way_idx_t = cista::strong<std::uint64_t, struct osm_way_idx_>;

constexpr auto const kEncodedNegativeOsmIdBit = std::uint64_t{1U} << 63U;

constexpr bool is_negative_encoded_osm_id(std::uint64_t const id) {
  return (id & kEncodedNegativeOsmIdBit) != 0U;
}

constexpr std::uint64_t encode_osm_id(std::int64_t const id) {
  return id >= 0 ? static_cast<std::uint64_t>(id)
                 : (kEncodedNegativeOsmIdBit |
                    static_cast<std::uint64_t>(-(id + 1)));
}

constexpr osm_node_idx_t to_osm_node_idx(std::int64_t const id) {
  return osm_node_idx_t{encode_osm_id(id)};
}

constexpr osm_way_idx_t to_osm_way_idx(std::int64_t const id) {
  return osm_way_idx_t{encode_osm_id(id)};
}

constexpr bool encoded_osm_id_less(std::uint64_t const lhs,
                                   std::uint64_t const rhs) {
  // Negative OSM ids are stored in the upper half of the uint64_t range but
  // still need to compare before positive ids in their original signed order.
  auto const lhs_negative = is_negative_encoded_osm_id(lhs);
  auto const rhs_negative = is_negative_encoded_osm_id(rhs);
  if (lhs_negative != rhs_negative) {
    return lhs_negative;
  }

  if (!lhs_negative) {
    return lhs < rhs;
  }

  return (lhs & ~kEncodedNegativeOsmIdBit) > (rhs & ~kEncodedNegativeOsmIdBit);
}

template <typename OsmIdx>
constexpr bool osm_id_less(OsmIdx const lhs, OsmIdx const rhs) {
  return encoded_osm_id_less(to_idx(lhs), to_idx(rhs));
}

using way_idx_t = cista::strong<std::uint32_t, struct way_idx_>;
using node_idx_t = cista::strong<std::uint32_t, struct node_idx_>;

using component_idx_t = cista::strong<std::uint32_t, struct component_idx_>;

using platform_idx_t = cista::strong<std::uint32_t, struct platform_idx_>;

using multi_level_elevator_idx_t =
    cista::strong<std::uint32_t, struct multi_level_elevator_idx_>;

using distance_t = std::uint32_t;
using elevation_monotonic_t =
    cista::strong<std::uint16_t, struct elevation_monotonic_>;

using way_pos_t = std::uint8_t;

using cost_t = std::uint32_t;
using duration_t = std::chrono::duration<std::uint16_t>;  // sec -> max ~18h
using routing_time_t =
    std::chrono::time_point<std::chrono::system_clock, std::chrono::seconds>;

constexpr auto const kInfeasible = std::numeric_limits<cost_t>::max();
constexpr auto const kMaxDuration =
    duration_t{std::numeric_limits<duration_t::rep>::max()};

template <typename T>
constexpr cost_t clamp_cost(T const c) {
  return static_cast<cost_t>(std::min(c, static_cast<T>(kInfeasible)));
}

constexpr duration_t clamp_duration(std::uint64_t const seconds) {
  return duration_t{static_cast<duration_t::rep>(
      std::min(seconds, static_cast<std::uint64_t>(kMaxDuration.count())))};
}

constexpr duration_t clamp_add_duration(duration_t const a,
                                        duration_t const b) {
  return clamp_duration(static_cast<std::uint64_t>(a.count()) +
                        static_cast<std::uint64_t>(b.count()));
}

constexpr duration_t clamp_sub_duration(duration_t const a,
                                        duration_t const b) {
  return duration_t{static_cast<duration_t::rep>(
      a.count() > b.count() ? a.count() - b.count() : 0U)};
}

template <typename T>
constexpr duration_t duration_from_cost(T const c) {
  return clamp_duration(static_cast<std::uint64_t>(c));
}

struct cost_and_duration {
  cost_t cost_{0U};
  duration_t duration_{0U};

  constexpr bool feasible() const noexcept { return cost_ != kInfeasible; }
};

constexpr cost_and_duration infeasible_cost_and_duration() {
  return {.cost_ = kInfeasible, .duration_ = kMaxDuration};
}

constexpr cost_and_duration cost_and_duration_from_cost(cost_t const cost) {
  return cost == kInfeasible
             ? infeasible_cost_and_duration()
             : cost_and_duration{.cost_ = cost,
                                 .duration_ = duration_from_cost(cost)};
}

constexpr cost_and_duration clamp_add(cost_and_duration const a,
                                      cost_and_duration const b) {
  if (!a.feasible() || !b.feasible()) {
    return infeasible_cost_and_duration();
  }
  return {.cost_ = clamp_cost(static_cast<std::uint64_t>(a.cost_) + b.cost_),
          .duration_ = clamp_add_duration(a.duration_, b.duration_)};
}

constexpr cost_and_duration clamp_add(cost_and_duration const value,
                                      cost_t const extra_cost,
                                      duration_t const extra_duration) {
  if (!value.feasible() || extra_cost == kInfeasible) {
    return infeasible_cost_and_duration();
  }
  return {
      .cost_ = clamp_cost(static_cast<std::uint64_t>(value.cost_) + extra_cost),
      .duration_ = clamp_add_duration(value.duration_, extra_duration)};
}

constexpr cost_and_duration clamp_add(cost_and_duration const value,
                                      cost_t const extra_cost) {
  return clamp_add(value, extra_cost, duration_from_cost(extra_cost));
}

// direction
enum class direction : std::uint8_t {
  kForward,
  kBackward,
};

inline std::ostream& operator<<(std::ostream& out, direction const d) {
  return out << (d == direction::kBackward ? "bwd" : "fwd");
}

constexpr direction opposite(direction const dir) {
  return dir == direction::kForward ? direction::kBackward
                                    : direction::kForward;
}

template <direction Dir>
constexpr direction flip(direction const dir) {
  return Dir == direction::kForward ? dir : opposite(dir);
}

constexpr direction flip(direction const search_dir, direction const dir) {
  return search_dir == direction::kForward ? dir : opposite(dir);
}

constexpr std::string_view to_str(direction const d) {
  switch (d) {
    case direction::kForward: return "forward";
    case direction::kBackward: return "backward";
  }
  std::unreachable();
}

inline std::optional<routing_time_t> current_routing_time(
    std::optional<routing_time_t> const start_time,
    direction const dir,
    duration_t const duration) {
  if (!start_time.has_value()) {
    return std::nullopt;
  }
  auto const delta = std::chrono::seconds{duration.count()};
  return dir == direction::kForward ? *start_time + delta : *start_time - delta;
}

constexpr direction to_direction(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("forward"): return direction::kForward;
    case cista::hash("backward"): return direction::kBackward;
  }
  std::unreachable();
}

// level
constexpr auto const kMinLevel = -8.0F;
constexpr auto const kMaxLevel = 7.5F;

struct level_t {
  static constexpr auto kNoLevel = 0U;

  friend constexpr std::uint8_t to_idx(level_t l) { return l.v_; }

  friend std::ostream& operator<<(std::ostream& out, level_t const l) {
    return (l.v_ == kNoLevel) ? (out << "-") : (out << l.to_float());
  }

  explicit constexpr level_t(std::uint8_t const x) : v_{x} {}

  explicit constexpr level_t(float const f)
      : v_{static_cast<std::uint8_t>((f - kMinLevel) / 0.25F + 1U)} {}

  constexpr float to_float() const {
    return (v_ == kNoLevel) ? 0.0F : (kMinLevel + ((v_ - 1U) / 4.0F));
  }

  constexpr bool has_level() const { return v_ != kNoLevel; }

  constexpr level_t() = default;

  auto operator<=>(level_t const&) const = default;

  constexpr cista::hash_t hash() const { return v_; }

  std::uint8_t v_;
};

constexpr auto const kNoLevel = level_t{std::uint8_t{0U}};

constexpr auto const kLevelBits = cista::constexpr_trailing_zeros(
    cista::next_power_of_two(to_idx(level_t{kMaxLevel}) + 1U));

using level_bits_t = std::uint64_t;

constexpr std::tuple<level_t, level_t, bool> get_levels(
    bool const has_level, level_bits_t const levels) noexcept {
  if (!has_level) {
    return {level_t{kNoLevel}, level_t{kNoLevel}, false};
  }
  auto from = kNoLevel, to = kNoLevel;
  utl::for_each_set_bit(levels, [&](auto&& bit) {
    from == kNoLevel  //
        ? from = level_t{static_cast<std::uint8_t>(bit)}
        : to = level_t{static_cast<std::uint8_t>(bit)};
  });
  return {from, to == kNoLevel ? from : to, std::popcount(levels) > 2};
}

static_assert(kLevelBits == 6U);

// speed
enum speed_limit : std::uint8_t {
  kmh_10,
  kmh_20,
  kmh_30,
  kmh_50,
  kmh_60,
  kmh_80,
  kmh_100,
  kmh_120,
};

constexpr speed_limit get_speed_limit(unsigned const x) {
  if (x >= 120U) {
    return speed_limit::kmh_120;
  } else if (x >= 100) {
    return speed_limit::kmh_100;
  } else if (x >= 80) {
    return speed_limit::kmh_80;
  } else if (x >= 60) {
    return speed_limit::kmh_60;
  } else if (x >= 50) {
    return speed_limit::kmh_50;
  } else if (x >= 30) {
    return speed_limit::kmh_30;
  } else if (x >= 20) {
    return speed_limit::kmh_20;
  } else {
    return speed_limit::kmh_10;
  }
}

constexpr std::uint16_t to_kmh(speed_limit const l) {
  switch (l) {
    case speed_limit::kmh_10: return 10U;
    case speed_limit::kmh_20: return 20U;
    case speed_limit::kmh_30: return 30U;
    case speed_limit::kmh_50: return 50U;
    case speed_limit::kmh_60: return 60U;
    case speed_limit::kmh_80: return 80U;
    case speed_limit::kmh_100: return 100U;
    case speed_limit::kmh_120: return 120U;
  }
  std::unreachable();
}

constexpr float to_seconds_per_meter(speed_limit const l) {
  switch (l) {
    case speed_limit::kmh_10: return 3.6f / 10U;
    case speed_limit::kmh_20: return 3.6f / 20U;
    case speed_limit::kmh_30: return 3.6f / 30U;
    case speed_limit::kmh_50: return 3.6f / 50U;
    case speed_limit::kmh_60: return 3.6f / 60U;
    case speed_limit::kmh_80: return 3.6f / 80U;
    case speed_limit::kmh_100: return 3.6f / 100U;
    case speed_limit::kmh_120: return 3.6f / 120U;
  }
  std::unreachable();
}

}  // namespace osr
