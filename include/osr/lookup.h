#pragma once

#include "osr/ways.h"

#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"

#include "rtree.h"

namespace osr {

enum class direction : std::uint8_t {
  kLeft,
  kRight,
};

struct start_dist {
  friend bool operator<(start_dist const& a, start_dist const& b) {
    return a.dist_ < b.dist_;
  }
  double min() const { return std::min(init_left_.second, init_right_.second); }
  double dist_;
  geo::latlng best_;
  std::size_t best_segment_idx_;
  way_idx_t way_;
  std::pair<node_idx_t, double> init_left_{node_idx_t::invalid(),
                                           std::numeric_limits<double>::max()};
  std::pair<node_idx_t, double> init_right_{node_idx_t::invalid(),
                                            std::numeric_limits<double>::max()};
  std::vector<geo::latlng> left_path_, right_path_;
};

template <typename PolyLine>
start_dist distance_to_way(geo::latlng const& x, PolyLine&& c) {
  auto min = std::numeric_limits<double>::max();
  auto best = geo::latlng{};
  auto best_segment_idx = 0U;
  auto segment_idx = 0U;
  for (auto const [a, b] : utl::pairwise(c)) {
    auto const candidate = geo::closest_on_segment(x, a, b);
    auto const dist = geo::distance(x, candidate);
    if (dist < min) {
      min = dist;
      best = candidate;
      best_segment_idx = segment_idx;
    }
    ++segment_idx;
  }
  return start_dist{
      .dist_ = min, .best_ = best, .best_segment_idx_ = best_segment_idx};
}

struct lookup {
  lookup(ways const& ways) : rtree_{rtree_new()}, ways_{ways} {
    utl::verify(rtree_ != nullptr, "rtree creation failed");
    for (auto i = way_idx_t{0U}; i != ways.n_ways(); ++i) {
      insert(i);
    }
  }

  ~lookup() { rtree_free(rtree_); }

  void find(geo::latlng const& query, std::vector<start_dist>& results) const {
    results.clear();
    find(query, [&](way_idx_t const way) {
      auto d = distance_to_way(query, ways_.way_polylines_[way]);
      d.way_ = way;
      results.emplace_back(d);
    });
    utl::sort(results);

    for (auto i = 0U; i != std::min(std::size_t{10U}, results.size()); ++i) {
      init(query, results[i]);
    }
  }

  void init(geo::latlng const& query, start_dist& d) const {
    init_right(query, d, d.best_segment_idx_ + 1U);
    init_left(query, d, d.best_segment_idx_);
  }

  void init_right(geo::latlng const& query,
                  start_dist& d,
                  std::size_t const idx) const {
    auto const polyline = ways_.way_polylines_[d.way_];
    auto const osm_idx = ways_.way_osm_nodes_[d.way_];
    auto pos = d.best_;
    auto dist = geo::distance(query, d.best_);
    d.right_path_.push_back(query);
    d.right_path_.push_back(d.best_);
    for (auto i = idx; i != polyline.size(); ++i) {
      dist += geo::distance(pos, polyline[i]);
      pos = polyline[i];
      d.right_path_.push_back(pos);
      if (auto const node = ways_.find_node_idx(osm_idx[i]); node.has_value()) {
        d.init_right_ = {*node, dist};
        break;
      }
    }
  }

  void init_left(geo::latlng const& query,
                 start_dist& d,
                 std::size_t const idx) const {
    auto const polyline = ways_.way_polylines_[d.way_];
    auto const osm_idx = ways_.way_osm_nodes_[d.way_];
    auto pos = d.best_;
    auto dist = geo::distance(query, d.best_);
    d.left_path_.push_back(query);
    d.left_path_.push_back(d.best_);
    for (auto j = 0U; j <= idx; ++j) {
      auto i = idx - j;
      dist += geo::distance(pos, polyline[i]);
      pos = polyline[i];
      d.left_path_.push_back(pos);
      if (auto const node = ways_.find_node_idx(osm_idx[i]); node.has_value()) {
        d.init_left_ = {*node, dist};
        break;
      }
    }
  }

  template <typename Fn>
  void find(geo::latlng const& query, Fn&& fn) const {
    auto const min =
        std::array<double, 2U>{query.lng_ - 0.01, query.lat_ - 0.01};
    auto const max =
        std::array<double, 2U>{query.lng_ + 0.01, query.lat_ + 0.01};
    rtree_search(
        rtree_, min.data(), max.data(),
        [](double const* /* min */, double const* /* max */, void const* item,
           void* udata) {
          (*reinterpret_cast<Fn*>(udata))(
              way_idx_t{static_cast<way_idx_t::value_t>(
                  reinterpret_cast<std::size_t>(item))});
          return true;
        },
        &fn);
  }

  void insert(way_idx_t const way) {
    auto b = osmium::Box{};
    for (auto const& c : ways_.way_polylines_[way]) {
      b.extend(osmium::Location{c.lat_, c.lng_});
    }

    auto const min_corner =
        std::array<double, 2U>{b.bottom_left().lon(), b.bottom_left().lat()};
    auto const max_corner =
        std::array<double, 2U>{b.top_right().lon(), b.top_right().lat()};

    rtree_insert(rtree_, min_corner.data(), max_corner.data(),
                 reinterpret_cast<void*>(to_idx(way)));
  }

  rtree* rtree_;
  ways const& ways_;
};

}  // namespace osr