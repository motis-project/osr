#pragma once

#include <cmath>
#include <cstdint>
#include <algorithm>
#include <optional>
#include <utility>

#include "utl/helpers/algorithm.h"
#include "utl/verify.h"

#include "geo/latlng.h"

#include "osr/elevation_storage.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"
#include "osr/ways.h"

namespace osr {

struct connecting_way {
  constexpr bool valid() const noexcept { return way_ != way_idx_t::invalid(); }

  way_idx_t way_{way_idx_t::invalid()};
  std::uint16_t from_{};
  std::uint16_t to_{};
  bool is_loop_{};
  std::uint16_t distance_{};
  elevation_storage::elevation elevation_{};
};

template <direction SearchDir, bool WithBlocked, Profile P>
inline connecting_way find_connecting_way(typename P::parameters const& params,
                                          ways const& w,
                                          ways::routing const& r,
                                          bitvec<node_idx_t> const* blocked,
                                          sharing_data const* sharing,
                                          elevation_storage const* elevations,
                                          typename P::node const from,
                                          typename P::node const to,
                                          cost_t const expected_cost) {
  auto conn = std::optional<connecting_way>{};
  P::template adjacent<SearchDir, WithBlocked>(
      params, r, from, blocked, sharing, elevations,
      [&](typename P::node const target, std::uint32_t const cost,
          distance_t const dist, way_idx_t const way, std::uint16_t const a_idx,
          std::uint16_t const b_idx,
          elevation_storage::elevation const elevation, bool) {
        if (target == to && cost == expected_cost) {
          auto const is_loop = way != way_idx_t::invalid() && r.is_loop(way) &&
                               static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                   r.way_nodes_[way].size() - 2U;
          conn = {way, a_idx, b_idx, is_loop, dist, elevation};
        }
      });
  utl::verify(
      conn.has_value(), "no connecting way node/{} -> node/{} found {} {}",
      (sharing == nullptr || from.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[from.get_node()])
          : 0,
      (sharing == nullptr || to.get_node() < sharing->additional_node_offset_)
          ? to_idx(w.node_to_osm_[to.get_node()])
          : 0,
      expected_cost, expected_cost);
  return *conn;
}

template <Profile P>
inline connecting_way find_connecting_way(typename P::parameters const& params,
                                          ways const& w,
                                          bitvec<node_idx_t> const* blocked,
                                          sharing_data const* sharing,
                                          elevation_storage const* elevations,
                                          typename P::node const from,
                                          typename P::node const to,
                                          cost_t const expected_cost,
                                          direction const dir) {
  auto const call = [&]<bool WithBlocked>() {
    if (dir == direction::kForward) {
      return find_connecting_way<direction::kForward, WithBlocked, P>(
          params, w, *w.r_, blocked, sharing, elevations, from, to,
          expected_cost);
    } else {
      return find_connecting_way<direction::kBackward, WithBlocked, P>(
          params, w, *w.r_, blocked, sharing, elevations, from, to,
          expected_cost);
    }
  };

  if (blocked == nullptr) {
    return call.template operator()<false>();
  } else {
    return call.template operator()<true>();
  }
}

template <Profile P>
inline double add_path(typename P::parameters const& params,
                       ways const& w,
                       ways::routing const& r,
                       bitvec<node_idx_t> const* blocked,
                       sharing_data const* sharing,
                       elevation_storage const* elevations,
                       typename P::node const from,
                       typename P::node const to,
                       cost_t const expected_cost,
                       std::vector<path::segment>& path,
                       direction const dir) {
  auto const& [way, from_idx, to_idx, is_loop, distance, elevation] =
      find_connecting_way<P>(params, w, blocked, sharing, elevations, from, to,
                             expected_cost, dir);

  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.elevation_ = elevation;
  segment.mode_ = to.get_mode();

  if (way != way_idx_t::invalid()) {
    auto const start_idx = dir == direction::kBackward ? to_idx : from_idx;
    auto const end_idx = dir == direction::kBackward ? from_idx : to_idx;
    auto const is_reverse = (start_idx > end_idx) ^ is_loop;

    if (is_reverse) {
      segment.from_level_ = r.way_properties_[way].to_level();
      segment.to_level_ = r.way_properties_[way].from_level();
    } else {
      segment.from_level_ = r.way_properties_[way].from_level();
      segment.to_level_ = r.way_properties_[way].to_level();
    }
    segment.from_ = r.way_nodes_[way][start_idx];
    segment.to_ = r.way_nodes_[way][end_idx];

    for (auto const [osm_idx, coord] : infinite(
             reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                     is_reverse),
             is_loop)) {
      utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U,
                  "infinite loop");
      if (!active && w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
        active = true;
      }
      if (active) {
        if (w.node_to_osm_[r.way_nodes_[way][start_idx]] == osm_idx) {
          // Again "from" node, then it's shorter to start from here.
          segment.polyline_.clear();
        }

        segment.polyline_.emplace_back(coord);
        if (w.node_to_osm_[r.way_nodes_[way][end_idx]] == osm_idx) {
          break;
        }
      }
    }
  } else {
    auto const get_node_pos = [&](node_idx_t const n) -> geo::latlng {
      if (n == node_idx_t::invalid()) {
        return {};
      } else if (w.is_additional_node(n)) {
        return sharing->get_additional_node_coordinates(n);
      } else {
        return w.get_node_pos(n).as_latlng();
      }
    };

    segment.from_level_ = level_t{0.0F};
    segment.to_level_ = level_t{0.0F};
    segment.from_ =
        dir == direction::kBackward ? to.get_node() : from.get_node();
    segment.to_ = dir == direction::kBackward ? from.get_node() : to.get_node();
    segment.polyline_ = {get_node_pos(segment.from_),
                         get_node_pos(segment.to_)};
  }

  return distance;
}

}  // namespace osr
