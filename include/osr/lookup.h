#pragma once

#include <optional>
#include <ostream>

#include "cista/containers/rtree.h"
#include "cista/reflection/printable.h"

#include "geo/box.h"
#include "geo/latlng.h"
#include "geo/polyline.h"

#include "osr/types.h"
#include "osr/ways.h"

#include "utl/cflow.h"
#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"

#include "osr/location.h"
#include "osr/routing/profile.h"
#include "osr/types.h"

namespace osr {

template <typename T, typename Collection, typename Fn>
void till_the_end(T const& start,
                  Collection const& c,
                  direction const dir,
                  Fn&& fn) {
  if (dir == direction::kForward) {
    for (auto i = start; i != c.size(); ++i) {
      if (fn(c[i]) == utl::cflow::kBreak) {
        break;
      }
    }
  } else {
    for (auto j = 0U; j <= start; ++j) {
      auto i = start - j;
      if (fn(c[i]) == utl::cflow::kBreak) {
        break;
      }
    }
  }
}

struct node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  level_t lvl_{kNoLevel};
  direction way_dir_{direction::kForward};
  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
  cost_t cost_{0U};
  std::vector<geo::latlng> path_{};
};

struct raw_node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
};

struct way_candidate {
  friend bool operator<(way_candidate const& a, way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  node_candidate left_{}, right_{};
};

struct raw_way_candidate {
  friend bool operator<(raw_way_candidate const& a,
                        raw_way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  raw_node_candidate left_{}, right_{};
};

using match_t = std::vector<way_candidate>;
using match_view_t = std::span<way_candidate const>;

struct lookup {
  lookup(ways const&, std::filesystem::path, cista::mmap::protection);

  void build_rtree();

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  std::vector<raw_way_candidate> get_raw_match(location const&,
                                               double max_match_distance) const;

  template <typename Profile>
  match_t complete_match(
      location const& query,
      bool const reverse,
      direction const search_dir,
      double max_match_distance,
      bitvec<node_idx_t> const* blocked,
                routing_parameters const rp,
      std::span<raw_way_candidate const> raw_way_candidates) const {
    auto matches = std::vector<way_candidate>{};
    auto i = 0U;
    auto dist = max_match_distance;
    for (auto const& raw_wc : raw_way_candidates) {
      while (raw_wc.dist_to_way_ >= dist && matches.empty() && i++ < 4U) {
        dist *= 2U;
      }
      if (raw_wc.dist_to_way_ >= dist) {
        break;
      }
      auto wc =
          way_candidate{raw_wc.dist_to_way_,
                        raw_wc.way_,
                        {query.lvl_, direction::kBackward, raw_wc.left_.node_,
                         raw_wc.left_.dist_to_node_},
                        {query.lvl_, direction::kForward, raw_wc.right_.node_,
                         raw_wc.right_.dist_to_node_}};
      apply_next_node_cost<Profile>(wc, wc.left_, query, reverse, search_dir,
                                    blocked, rp);
      apply_next_node_cost<Profile>(wc, wc.right_, query, reverse, search_dir,
                                    blocked, rp);
      if (wc.left_.valid() || wc.right_.valid()) {
        matches.emplace_back(std::move(wc));
      }
    }
    if (i < 4 && matches.empty()) {
      return match<Profile>(query, reverse, search_dir, max_match_distance,
                            blocked, rp);
    }
    return matches;
  }

  template <typename Profile>
  std::vector<geo::latlng> get_node_candidate_path(
      way_candidate const& wc,
      node_candidate const& nc,
      bool const reverse,
      location const& query) const {
    if (!nc.path_.empty() || !nc.valid()) {
      return nc.path_;
    }
    auto const approx_distance_lng_degrees =
        geo::approx_distance_lng_degrees(query.pos_);
    auto const [squared_dist, best, segment_idx] =
        geo::approx_squared_distance_to_polyline<
            std::tuple<double, geo::latlng, size_t>>(
            query.pos_, ways_.way_polylines_[wc.way_],
            approx_distance_lng_degrees);
    auto const polyline = ways_.way_polylines_[wc.way_];
    auto const osm_nodes = ways_.way_osm_nodes_[wc.way_];
    auto path = std::vector<geo::latlng>{best};
    till_the_end(segment_idx + (nc.way_dir_ == direction::kForward ? 1U : 0U),
                 utl::zip(polyline, osm_nodes), nc.way_dir_, [&](auto&& x) {
                   auto const& [pos, osm_node_idx] = x;
                   path.push_back(pos);
                   if (ways_.node_to_osm_[nc.node_] == osm_node_idx) {
                     return utl::cflow::kBreak;
                   }
                   return utl::cflow::kContinue;
                 });

    if (reverse) {
      std::reverse(begin(path), end(path));
    }
    return path;
  }

  match_t match(location const& query,
                bool const reverse,
                direction const search_dir,
                double const max_match_distance,
                bitvec<node_idx_t> const* blocked,
                search_profile,
                routing_parameters const rp,
                std::optional<std::span<raw_way_candidate const>>
                    raw_way_candidates = std::nullopt) const;

  template <typename Profile>
  match_t match(location const& query,
                bool const reverse,
                direction const search_dir,
                double max_match_distance,
                bitvec<node_idx_t> const* blocked,
                routing_parameters const rp,
                std::optional<std::span<raw_way_candidate const>>
                    raw_way_candidates = std::nullopt) const {
    if (raw_way_candidates.has_value()) {
      return complete_match<Profile>(query, reverse, search_dir,
                                     max_match_distance, blocked,
                                     rp, *raw_way_candidates);
    }
    auto way_candidates = get_way_candidates<Profile>(
        query, reverse, search_dir, max_match_distance, blocked, rp);
    auto i = 0U;
    while (way_candidates.empty() && i++ < 4U) {
      max_match_distance *= 2U;
      way_candidates = get_way_candidates<Profile>(query, reverse, search_dir,
                                                   max_match_distance, blocked, rp);
    }
    return way_candidates;
  }

  template <typename Fn>
  void find(geo::box const& b, Fn&& fn) const {
    auto const min = b.min_.lnglat_float();
    auto const max = b.max_.lnglat_float();
    rtree_.search(min, max, [&](auto, auto, way_idx_t const w) {
      fn(w);
      return true;
    });
  }

  hash_set<node_idx_t> find_elevators(geo::box const& b) const;

  void insert(way_idx_t);

private:
  template <typename Profile>
  match_t get_way_candidates(location const& query,
                             bool const reverse,
                             direction const search_dir,
                             double const max_match_distance,
                             bitvec<node_idx_t> const* blocked, routing_parameters const rp) const {
    auto way_candidates = std::vector<way_candidate>{};
    auto const approx_distance_lng_degrees =
        geo::approx_distance_lng_degrees(query.pos_);
    auto const squared_max_dist = std::pow(max_match_distance, 2);
    find(geo::box{query.pos_, max_match_distance}, [&](way_idx_t const way) {
      auto const [squared_dist, best, segment_idx] =
          geo::approx_squared_distance_to_polyline<
              std::tuple<double, geo::latlng, size_t>>(
              query.pos_, ways_.way_polylines_[way],
              approx_distance_lng_degrees);
      if (squared_dist < squared_max_dist) {
        auto wc = way_candidate{std::sqrt(squared_dist), way};
        wc.left_ = find_next_node<Profile>(
            wc, query, direction::kBackward, query.lvl_, reverse, search_dir,
            blocked, approx_distance_lng_degrees, best, segment_idx, rp);
        wc.right_ = find_next_node<Profile>(
            wc, query, direction::kForward, query.lvl_, reverse, search_dir,
            blocked, approx_distance_lng_degrees, best, segment_idx, rp);
        if (wc.left_.valid() || wc.right_.valid()) {
          way_candidates.emplace_back(std::move(wc));
        }
      }
    });
    utl::sort(way_candidates);
    return way_candidates;
  }

  template <typename Profile>
  bool is_way_node_feasible(way_candidate const& wc,
                            node_idx_t const node_idx,
                            location const& query,
                            bool const reverse,
                            direction const search_dir) const {
    auto const node_prop = ways_.r_->node_properties_[node_idx];
    if (Profile::node_cost(node_prop) == kInfeasible) {
      return false;
    }
    auto found = false;
    Profile::resolve_start_node(*ways_.r_, wc.way_, node_idx, query.lvl_,
                                reverse ? opposite(search_dir) : search_dir,
                                [&](auto const) { found = true; });
    return found;
  }

  template <typename Profile>
  node_candidate find_next_node(way_candidate const& wc,
                                location const& query,
                                direction const dir,
                                level_t const lvl,
                                bool const reverse,
                                direction const search_dir,
                                bitvec<node_idx_t> const* blocked,
                                double approx_distance_lng_degrees,
                                geo::latlng const best,
                                size_t segment_idx, routing_parameters const rp) const {
    auto const way_prop = ways_.r_->way_properties_[wc.way_];
    auto const edge_dir = reverse ? opposite(dir) : dir;
    if (Profile::way_cost(way_prop, flip(search_dir, edge_dir), 0U, rp) ==
        kInfeasible) {
      return node_candidate{};
    }

    auto c = node_candidate{.lvl_ = lvl,
                            .way_dir_ = dir,
                            .dist_to_node_ = wc.dist_to_way_,
                            .cost_ = 0,
                            .path_ = {best}};
    auto const polyline = ways_.way_polylines_[wc.way_];
    auto const osm_nodes = ways_.way_osm_nodes_[wc.way_];

    till_the_end(segment_idx + (dir == direction::kForward ? 1U : 0U),
                 utl::zip(polyline, osm_nodes), dir, [&](auto&& x) {
                   auto const& [pos, osm_node_idx] = x;

                   auto const segment_dist =
                       std::sqrt(geo::approx_squared_distance(
                           c.path_.back(), pos, approx_distance_lng_degrees));
                   c.dist_to_node_ += segment_dist;
                   c.path_.push_back(pos);

                   auto const way_node = ways_.find_node_idx(osm_node_idx);
                   if (way_node.has_value()) {
                     if (is_way_node_feasible<Profile>(wc, *way_node, query,
                                                       reverse, search_dir) &&
                         (blocked == nullptr || !blocked->test(*way_node))) {
                       c.node_ = *way_node;
                       c.cost_ = Profile::way_cost(
                           way_prop, flip(search_dir, edge_dir),
                           static_cast<distance_t>(c.dist_to_node_), rp);
                     }
                     return utl::cflow::kBreak;
                   }

                   return utl::cflow::kContinue;
                 });

    if (reverse ^ (search_dir == direction::kBackward)) {
      std::reverse(begin(c.path_), end(c.path_));
    }
    return c;
  }

  std::vector<raw_way_candidate> get_raw_way_candidates(
      location const& query, double const max_match_distance) const;

  raw_node_candidate find_raw_next_node(raw_way_candidate const&,
                                        direction const,
                                        double,
                                        geo::latlng const,
                                        size_t) const;

  template <typename Profile>
  void apply_next_node_cost(way_candidate const& wc,
                            node_candidate& nc,
                            location const& query,
                            bool const reverse,
                            direction const search_dir,
                            bitvec<node_idx_t> const* blocked, routing_parameters const rp) const {
    if (!nc.valid()) {
      return;
    }
    if (!is_way_node_feasible<Profile>(wc, nc.node_, query, reverse,
                                       search_dir)) {
      nc.node_ = node_idx_t::invalid();
      return;
    }
    auto const way_prop = ways_.r_->way_properties_[wc.way_];

    auto const edge_dir = reverse ? opposite(nc.way_dir_) : nc.way_dir_;
    auto const cost =
        Profile::way_cost(way_prop, flip(search_dir, edge_dir),
                          static_cast<distance_t>(nc.dist_to_node_), rp);

    if (cost != kInfeasible &&
        (blocked == nullptr || !blocked->test(nc.node_))) {
      nc.cost_ = cost;
    } else {
      nc.node_ = node_idx_t::invalid();
    }
  }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  cista::mm_rtree<way_idx_t> rtree_;
  ways const& ways_;
};

}  // namespace osr
