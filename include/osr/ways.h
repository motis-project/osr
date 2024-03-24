#pragma once

#include <filesystem>
#include <ranges>

#include "osmium/osm/way.hpp"

#include "utl/progress_tracker.h"
#include "utl/verify.h"
#include "utl/zip.h"

#include "osr/multi_counter.h"
#include "osr/point.h"
#include "osr/types.h"

namespace osr {

enum speed_limit : std::uint8_t {
  kmh_10,
  kmh_30,
  kmh_50,
  kmh_70,
  kmh_100,
  kmh_120
};

struct way_properties {
  std::uint8_t is_car_accessible_ : 1;
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_walk_accessible_ : 1;
  std::uint8_t oneway_car_ : 1;
  std::uint8_t oneway_bike_ : 1;
  std::uint8_t speed_limit_ : 3;
};

struct node_properties {
  std::uint8_t is_bike_accessible_ : 1;
  std::uint8_t is_walk_accessible_ : 1;
  std::uint8_t is_car_accessible_ : 1;
};

struct ways {
  ways(std::filesystem::path p, cista::mmap::protection const mode)
      : p_{std::move(p)},
        mode_{mode},
        osm_to_node_{mm("osm_to_mode.bin")},
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
                mm("node_in_way_idx_index.bin")}} {}

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
        osm_to_node_.push_back({i, node_idx});
        ++node_idx;
        pt->update(b_idx);
      });
      node_ways_.resize(to_idx(node_idx));
      node_in_way_idx_.resize(to_idx(node_idx));
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
            dists.push_back(distance);
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

  std::optional<node_idx_t> find_node_idx(osm_node_idx_t const i) const {
    auto const it =
        std::lower_bound(begin(osm_to_node_), end(osm_to_node_), i,
                         [](auto&& a, auto&& b) { return a.first < b; });
    return it != end(osm_to_node_) && it->first == i ? std::optional{it->second}
                                                     : std::nullopt;
  }

  node_idx_t get_node_idx(osm_node_idx_t const i) const {
    auto const it =
        std::lower_bound(begin(osm_to_node_), end(osm_to_node_), i,
                         [](auto&& a, auto&& b) { return a.first < b; });
    utl::verify(it != end(osm_to_node_) && it->first == i,
                "osm node {} not found", i);
    return it->second;
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

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).c_str(), mode_};
  }

  way_idx_t::value_t n_ways() const { return way_osm_idx_.size(); }
  node_idx_t::value_t n_nodes() const { return node_to_osm_.size(); }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  mm_vec<pair<osm_node_idx_t, node_idx_t>> osm_to_node_;
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
};

}  // namespace osr