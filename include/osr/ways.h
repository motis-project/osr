#pragma once

#if defined(_WIN32) || defined(_WIN64)
#include "Memoryapi.h"
#define mlock(addr, size) VirtualLock((LPVOID)addr, (SIZE_T)size)
#else
#include <sys/mman.h>
#endif
#include <filesystem>
#include <ranges>

#include "osmium/osm/way.hpp"

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

#define OSR_DEBUG_DIJKSTRA
#ifdef OSR_DEBUG_DIJKSTRA
#define trace(...) fmt::println(std::cerr, __VA_ARGS__)
#else
#define trace(...)
#endif

enum speed_limit : std::uint8_t {
  kmh_10,
  kmh_30,
  kmh_50,
  kmh_70,
  kmh_100,
  kmh_120,
};

constexpr auto const kMinLevel = -4.0F;
constexpr auto const kMaxLevel = 4.0F;

constexpr level_t to_level(float const f) {
  return level_t{static_cast<std::uint8_t>((f - kMinLevel) / 0.25F)};
}

constexpr float to_float(level_t const l) {
  return kMinLevel + (to_idx(l) / 4.0F);
}

constexpr auto const kLevelBits = cista::constexpr_trailing_zeros(
    cista::next_power_of_two(to_idx(to_level(kMaxLevel))));

static_assert(kLevelBits == 5U);

struct resolved_restriction {
  enum class type { kNo, kOnly } type_;
  way_idx_t from_, to_;
  node_idx_t via_;
};

struct restriction {
  CISTA_FRIEND_COMPARABLE(restriction)
  std::uint8_t from_, to_;
};

enum node_type : std::uint8_t {
  kEntry,
  kEntrance,
  kElevator,
};

constexpr std::uint16_t to_kmh(speed_limit const l) {
  switch (l) {
    case speed_limit::kmh_10: return 10U;
    case speed_limit::kmh_30: return 30U;
    case speed_limit::kmh_50: return 50U;
    case speed_limit::kmh_70: return 70U;
    case speed_limit::kmh_100: return 100U;
    case speed_limit::kmh_120: return 120U;
  }
  std::unreachable();
}

constexpr std::uint16_t to_meters_per_second(speed_limit const l) {
  return to_kmh(l) / 3.6;
}

struct way_properties {
  constexpr bool is_accessible() const {
    return is_car_accessible() || is_bike_accessible() || is_foot_accessible();
  }
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_foot_accessible() const { return is_foot_accessible_; }
  constexpr bool is_oneway_car() const { return is_oneway_car_; }
  constexpr bool is_oneway_bike() const { return is_oneway_bike_; }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_steps() const { return is_steps_; }
  constexpr std::uint16_t max_speed_m_per_s() const {
    return to_meters_per_second(static_cast<speed_limit>(speed_limit_));
  }
  constexpr std::uint16_t max_speed_km_per_h() const {
    return to_kmh(static_cast<speed_limit>(speed_limit_));
  }
  constexpr level_t get_level() const { return level_t{level_}; }
  constexpr level_t get_to_level() const { return level_t{to_level_}; }

  constexpr bool can_use_steps(level_t const a, level_t const b) const {
    return is_steps_ && std::min(a, b) == get_level() &&
           std::max(a, b) == get_to_level();
  }

  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_oneway_car_ : 1;
  std::uint8_t is_oneway_bike_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_steps_ : 1;

  std::uint8_t speed_limit_ : 3;

  std::uint8_t level_ : 5;

  std::uint8_t to_level_ : 5;
};

static_assert(sizeof(way_properties) == 3);

struct node_properties {
  constexpr bool is_car_accessible() const { return is_car_accessible_; }
  constexpr bool is_bike_accessible() const { return is_bike_accessible_; }
  constexpr bool is_walk_accessible() const { return is_foot_accessible_; }
  constexpr bool is_elevator() const { return is_elevator_; }
  constexpr bool is_entrance() const { return is_entrance_; }

  constexpr level_t get_from_level() const { return level_t{from_level_}; }
  constexpr level_t get_to_level() const { return level_t{to_level_}; }

  constexpr bool can_use_elevator(level_t const from, level_t const to) const {
    return is_elevator_ && is_in_range(from) && is_in_range(to);
  }

  constexpr bool is_in_range(level_t const l) const {
    return get_from_level() <= l && l <= get_to_level();
  }

  std::uint8_t from_level_ : 5;

  std::uint8_t is_foot_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_elevator_ : 1;
  std::uint8_t is_entrance_ : 1;

  std::uint8_t to_level_ : 5;
};

static_assert(sizeof(node_properties) == 2);

struct ways {
  ways(std::filesystem::path p, cista::mmap::protection const mode)
      : p_{std::move(p)},
        mode_{mode},
        node_to_osm_{mm("node_to_osm.bin")},
        node_properties_{mm("node_properties.bin")},
        way_osm_idx_{mm("way_osm_idx.bin")},
        way_properties_{mm("way_properties.bin")},
        way_polylines_{mm_vec<point>{mm("way_polylines_data.bin")},
                       mm_vec<std::uint64_t>{mm("way_polylines_index.bin")}},
        way_osm_nodes_{mm_vec<osm_node_idx_t>{mm("way_osm_nodes_data.bin")},
                       mm_vec<std::uint64_t>{mm("way_osm_nodes_index.bin")}},
        way_nodes_{mm_vec<node_idx_t>{mm("way_nodes_data.bin")},
                   mm_vec<std::uint64_t>{mm("way_nodes_index.bin")}},
        way_node_dist_{mm_vec<std::uint16_t>{mm("way_node_dist_data.bin")},
                       mm_vec<std::uint64_t>{mm("way_node_dist_index.bin")}},
        node_ways_{
            cista::paged<mm_vec<way_idx_t>>{
                mm_vec<way_idx_t>{mm("node_ways_data.bin")}},
            mm_vec<cista::page<std::uint64_t>>{mm("node_ways_index.bin")}},
        node_in_way_idx_{
            cista::paged<mm_vec<std::uint16_t>>{
                mm_vec<std::uint16_t>{mm("node_in_way_idx_data.bin")}},
            mm_vec<cista::page<std::uint64_t>>{
                mm("node_in_way_idx_index.bin")}},
        node_is_restricted_{mm_vec<std::uint64_t>{mm("node_restricted.bin")}},
        node_restrictions_{
            mm_vec<restriction>{mm("node_restrictions_data.bin")},
            mm_vec<std::uint32_t>{mm("node_restrictions_index.bin")}} {}

  void lock() const {
    auto const timer = utl::scoped_timer{"lock"};

    mlock(node_properties_.mmap_.data(), node_properties_.mmap_.size());
    mlock(way_properties_.mmap_.data(), way_properties_.mmap_.size());

    mlock(way_nodes_.data_.mmap_.data(), way_nodes_.data_.mmap_.size());
    mlock(way_nodes_.bucket_starts_.mmap_.data(),
          way_nodes_.bucket_starts_.mmap_.size());

    mlock(way_node_dist_.data_.mmap_.data(), way_node_dist_.data_.mmap_.size());
    mlock(way_node_dist_.bucket_starts_.mmap_.data(),
          way_node_dist_.bucket_starts_.mmap_.size());

    mlock(node_ways_.paged_.data_.mmap_.data(),
          node_ways_.paged_.data_.mmap_.size());
    mlock(node_ways_.idx_.mmap_.data(), node_ways_.idx_.mmap_.size());

    mlock(node_in_way_idx_.paged_.data_.mmap_.data(),
          node_in_way_idx_.paged_.data_.mmap_.size());
    mlock(node_in_way_idx_.idx_.mmap_.data(), node_ways_.idx_.mmap_.size());
  }

  way_pos_t get_way_pos(node_idx_t const node, way_idx_t const way) const {
    auto const ways = node_ways_[node];
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      if (ways[i] == way) {
        return i;
      }
    }
    trace("node {} not found in way {}", node_to_osm_[node], way_osm_idx_[way]);
    return 0U;
  }

  void add_restriction(std::vector<resolved_restriction>& rs) {
    using it_t = std::vector<resolved_restriction>::iterator;
    utl::sort(rs, [](auto&& a, auto&& b) { return a.via_ < b.via_; });
    utl::equal_ranges_linear(
        begin(rs), end(rs), [](auto&& a, auto&& b) { return a.via_ == b.via_; },
        [&](it_t const& lb, it_t const& ub) {
          auto const range = std::span{lb, ub};
          node_restrictions_.resize(to_idx(range.front().via_) + 1U);
          node_is_restricted_.set(range.front().via_, true);

          for (auto const& x : range) {
            if (x.type_ == resolved_restriction::type::kNo) {
              node_restrictions_[x.via_].push_back(restriction{
                  get_way_pos(x.via_, x.from_), get_way_pos(x.via_, x.to_)});
            } else /* kOnly */ {
              for (auto const [i, from] : utl::enumerate(node_ways_[x.via_])) {
                for (auto const [j, to] : utl::enumerate(node_ways_[x.via_])) {
                  if (x.from_ == from && x.to_ != to) {
                    node_restrictions_[x.via_].push_back(restriction{
                        static_cast<way_pos_t>(i), static_cast<way_pos_t>(j)});
                  }
                }
              }
            }
          }
        });
    node_restrictions_.resize(node_to_osm_.size());
  }

  void connect_ways() {
    auto pt = utl::get_active_progress_tracker_or_activate("osr");

    {  // Assign graph node ids to every node with >1 way.
      pt->status("Create graph nodes")
          .in_high(node_way_counter_.size())
          .out_bounds(50, 60);

      auto node_idx = node_idx_t{0U};
      node_way_counter_.multi_.for_each_set_bit([&](std::uint64_t const b_idx) {
        auto const i = osm_node_idx_t{b_idx};
        node_to_osm_.push_back(i);
        ++node_idx;
        pt->update(b_idx);
      });
      node_ways_.resize(to_idx(node_idx));
      node_in_way_idx_.resize(to_idx(node_idx));
      node_is_restricted_.resize(to_idx(node_idx));
    }

    // Build edges.
    pt->status("Connect ways")
        .in_high(way_osm_nodes_.size())
        .out_bounds(60, 90);
    for (auto const [osm_way_idx, osm_nodes, polyline] :
         utl::zip(way_osm_idx_, way_osm_nodes_, way_polylines_)) {
      auto pred_pos = std::make_optional<point>();
      auto from = node_idx_t::invalid();
      auto distance = 0.0;
      auto i = std::uint16_t{0U};
      auto way_idx = way_idx_t{way_nodes_.size()};
      auto dists = way_node_dist_.add_back_sized(0U);
      auto nodes = way_nodes_.add_back_sized(0U);
      for (auto const [osm_node_idx, pos] : utl::zip(osm_nodes, polyline)) {
        if (pred_pos.has_value()) {
          distance += geo::distance(pos, *pred_pos);
        }

        if (node_way_counter_.is_multi(to_idx(osm_node_idx))) {
          auto const to = get_node_idx(osm_node_idx);
          node_ways_[to].push_back(way_idx);
          node_in_way_idx_[to].push_back(i);
          nodes.push_back(to);

          if (from != node_idx_t::invalid()) {
            dists.push_back(static_cast<std::uint16_t>(std::round(distance)));
          }

          distance = 0.0;
          from = to;

          if (i == std::numeric_limits<std::uint16_t>::max()) {
            fmt::println("error: way with {} nodes", osm_way_idx);
          }

          ++i;
        }

        pred_pos = pos;
      }
      pt->increment();
    }
  }

  std::optional<way_idx_t> find_way(osm_way_idx_t const i) {
    auto const it = std::lower_bound(begin(way_osm_idx_), end(way_osm_idx_), i);
    return it != end(way_osm_idx_) && *it == i
               ? std::optional{way_idx_t{
                     std::distance(begin(way_osm_idx_), it)}}
               : std::nullopt;
  }

  std::optional<node_idx_t> find_node_idx(osm_node_idx_t const i) const {
    auto const it = std::lower_bound(begin(node_to_osm_), end(node_to_osm_), i,
                                     [](auto&& a, auto&& b) { return a < b; });
    if (it == end(node_to_osm_) || *it != i) {
      return std::nullopt;
    }
    return std::make_optional(node_idx_t{static_cast<node_idx_t::value_t>(
        std::distance(begin(node_to_osm_), it))});
  }

  node_idx_t get_node_idx(osm_node_idx_t const i) const {
    auto const j = find_node_idx(i);
    utl::verify(j.has_value(), "osm node {} not found", i);
    return *j;
  }

  bool is_loop(way_idx_t const w) const {
    return way_nodes_[w].back() == way_nodes_[w].front();
  }

  point get_node_pos(node_idx_t const i) const {
    auto const osm_idx = node_to_osm_[i];
    auto const way = node_ways_[i][0];
    for (auto const [o, p] :
         utl::zip(way_osm_nodes_[way], way_polylines_[way])) {
      if (o == osm_idx) {
        return p;
      }
    }
    throw utl::fail("unable to find node {} [osm={}] in way {} [osm]", i,
                    osm_idx, way, way_osm_idx_[way]);
  }

  bool is_restricted(node_idx_t const n,
                     std::uint8_t const from,
                     std::uint8_t const to) const {
    if (!node_is_restricted_[n]) {
      return false;
    }

    auto const from_prop = way_properties_[node_ways_[n][from]];
    auto const to_prop = way_properties_[node_ways_[n][to]];
    auto const node_prop = node_properties_[n];
    auto const level_change = !node_prop.is_elevator_ &&
                              !(to_prop.is_elevator_ || to_prop.is_steps_) &&
                              from_prop.get_level() != to_prop.get_level();
    if (level_change) {
      return true;
    }

    auto const r = node_restrictions_[n];
    return utl::find(r, restriction{from, to}) != end(r);
  }

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  way_idx_t::value_t n_ways() const { return way_osm_idx_.size(); }
  node_idx_t::value_t n_nodes() const { return node_to_osm_.size(); }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  mm_vec_map<node_idx_t, osm_node_idx_t> node_to_osm_;
  mm_vec_map<node_idx_t, node_properties> node_properties_;
  mm_vec_map<way_idx_t, osm_way_idx_t> way_osm_idx_;
  mm_vec_map<way_idx_t, way_properties> way_properties_;
  mm_vecvec<way_idx_t, point, std::uint64_t> way_polylines_;
  mm_vecvec<way_idx_t, osm_node_idx_t, std::uint64_t> way_osm_nodes_;
  mm_vecvec<way_idx_t, node_idx_t, std::uint64_t> way_nodes_;
  mm_vecvec<way_idx_t, std::uint16_t, std::uint64_t> way_node_dist_;
  mm_paged_vecvec<node_idx_t, way_idx_t> node_ways_;
  mm_paged_vecvec<node_idx_t, std::uint16_t> node_in_way_idx_;
  multi_counter node_way_counter_;
  mm_bitvec<node_idx_t> node_is_restricted_;
  mm_vecvec<node_idx_t, restriction> node_restrictions_;
};

}  // namespace osr