#pragma once

#include <optional>
#include <ostream>

#include "cista/containers/rtree.h"
#include "cista/reflection/printable.h"

#include "geo/box.h"
#include "geo/latlng.h"
#include "geo/polyline.h"

#include "utl/cflow.h"
#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"

#include "osr/location.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/common.h"
#include "osr/types.h"
#include "osr/ways.h"

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
  float dist_to_node_{0.0};
};

struct way_candidate {
  friend bool operator<(way_candidate const& a, way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  double dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  node_candidate left_{}, right_{};
  geo::latlng closest_point_on_way_{};
  unsigned segment_idx_{};
};

struct raw_way_candidate {
  friend bool operator<(raw_way_candidate const& a,
                        raw_way_candidate const& b) {
    return a.dist_to_way_ < b.dist_to_way_;
  }

  float dist_to_way_;
  way_idx_t way_{way_idx_t::invalid()};
  raw_node_candidate left_{}, right_{};
};

using match_t = std::vector<way_candidate>;
using match_view_t = std::span<way_candidate const>;

using way_candidate_idx_t = cista::strong<std::uint32_t, struct way_candidate_idx_>;
using match_idx_t = cista::strong<std::uint32_t, struct match_idx_>;

// Structure of arrays replacement for `std::vector<match_t>`: all matches of
// one request live in flat, trivially copyable arrays, bucketed per match.
// `node_candidate::path_` is intentionally absent - it is only needed for path
// reconstruction, which recomputes it via `get_node_candidate_path()`.
struct match_result {
  struct node {
    bool valid() const { return node_ != node_idx_t::invalid(); }

    node_idx_t node_{node_idx_t::invalid()};
    float dist_to_node_{0.0F};
    cost_t cost_{0U};
  };

  struct nodes {
    node left_{}, right_{};
  };

  // View of a single match (= all way candidates of one query location).
  struct view {
    bool empty() const { return way_.empty(); }
    std::size_t size() const { return way_.size(); }

    std::span<float const> dist_to_way_{};
    std::span<way_idx_t const> way_{};
    std::span<nodes const> nodes_{};
    level_t lvl_{kNoLevel};
  };

  match_result() { begin_.emplace_back(way_candidate_idx_t{0U}); }

  void clear() {
    begin_.clear();
    begin_.emplace_back(way_candidate_idx_t{0U});
    lvl_.clear();
    dist_to_way_.clear();
    way_.clear();
    nodes_.clear();
  }

  std::size_t size() const { return lvl_.size(); }
  bool empty() const { return lvl_.empty(); }

  // Appending a match: start() -> add()* -> finish().
  void start(level_t const lvl) { lvl_.emplace_back(lvl); }

  void add(float const dist_to_way, way_idx_t const w, nodes const& n) {
    dist_to_way_.emplace_back(dist_to_way);
    way_.emplace_back(w);
    nodes_.emplace_back(n);
  }

  void finish() {
    begin_.emplace_back(
        way_candidate_idx_t{static_cast<way_candidate_idx_t::value_t>(
            way_.size())});
  }

  view operator[](match_idx_t const i) const {
    auto const from = to_idx(begin_[i]);
    auto const to = to_idx(begin_[match_idx_t{to_idx(i) + 1U}]);
    auto const n = static_cast<std::size_t>(to - from);
    return view{
        .dist_to_way_ = std::span{&dist_to_way_[way_candidate_idx_t{from}], n},
        .way_ = std::span{&way_[way_candidate_idx_t{from}], n},
        .nodes_ = std::span{&nodes_[way_candidate_idx_t{from}], n},
        .lvl_ = lvl_[i]};
  }

  vec_map<match_idx_t, way_candidate_idx_t> begin_{};  // size() + 1 entries
  vec_map<match_idx_t, level_t> lvl_{};
  vec_map<way_candidate_idx_t, float> dist_to_way_{};
  vec_map<way_candidate_idx_t, way_idx_t> way_{};
  vec_map<way_candidate_idx_t, nodes> nodes_{};
};

// Uniform view of one matched way node, independent of AoS/SoA storage.
struct candidate_node {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
  cost_t cost_{0U};
  direction way_dir_{direction::kForward};
  level_t lvl_{kNoLevel};
};

// Accessors letting routing code iterate AoS (`match_t`) and SoA
// (`match_result`) matches with the same syntax. `left_` is always the
// backward node, `right_` the forward one, so the SoA form derives the
// direction from the position instead of storing it.
inline std::size_t n_candidates(match_view_t const m) { return m.size(); }
inline way_idx_t candidate_way(match_view_t const m, std::size_t const j) {
  return m[j].way_;
}
inline double candidate_dist_to_way(match_view_t const m, std::size_t const j) {
  return m[j].dist_to_way_;
}
inline candidate_node candidate_left(match_view_t const m, std::size_t const j) {
  auto const& n = m[j].left_;
  return {n.node_, n.dist_to_node_, n.cost_, n.way_dir_, n.lvl_};
}
inline candidate_node candidate_right(match_view_t const m,
                                      std::size_t const j) {
  auto const& n = m[j].right_;
  return {n.node_, n.dist_to_node_, n.cost_, n.way_dir_, n.lvl_};
}

inline std::size_t n_candidates(match_result::view const& m) {
  return m.size();
}
inline way_idx_t candidate_way(match_result::view const& m,
                               std::size_t const j) {
  return m.way_[j];
}
inline double candidate_dist_to_way(match_result::view const& m,
                                    std::size_t const j) {
  return m.dist_to_way_[j];
}
inline candidate_node candidate_left(match_result::view const& m,
                                     std::size_t const j) {
  auto const& n = m.nodes_[j].left_;
  return {n.node_, n.dist_to_node_, n.cost_, direction::kBackward, m.lvl_};
}
inline candidate_node candidate_right(match_result::view const& m,
                                      std::size_t const j) {
  auto const& n = m.nodes_[j].right_;
  return {n.node_, n.dist_to_node_, n.cost_, direction::kForward, m.lvl_};
}

// Same for the container of matches passed to the one-to-many `route()`.
inline std::size_t n_matches(std::vector<match_t> const& m) { return m.size(); }
inline match_view_t match_at(std::vector<match_t> const& m,
                             std::size_t const i) {
  return m[i];
}
inline std::size_t n_matches(match_result const& m) { return m.size(); }
inline match_result::view match_at(match_result const& m, std::size_t const i) {
  return m[match_idx_t{static_cast<match_idx_t::value_t>(i)}];
}

struct lookup {
  lookup(ways const&, std::filesystem::path, cista::mmap::protection);

  void build_rtree();

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  std::vector<raw_way_candidate> get_raw_match(location const&,
                                               double max_match_distance) const;

  template <Profile P>
  match_t complete_match(
      P::parameters const& params,
      location const& query,
      bool const reverse,
      direction const search_dir,
      double max_match_distance,
      bitvec<node_idx_t> const* blocked,
      std::optional<routing_time_t> const start_time,
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
      apply_next_node_cost<P>(params, wc, wc.left_, query, reverse, search_dir,
                              blocked, start_time);
      apply_next_node_cost<P>(params, wc, wc.right_, query, reverse, search_dir,
                              blocked, start_time);
      if (wc.left_.valid() || wc.right_.valid()) {
        matches.emplace_back(std::move(wc));
      }
    }
    if (i < 4 && matches.empty()) {
      return match<P>(params, query, reverse, search_dir, max_match_distance,
                      blocked, start_time);
    }
    return matches;
  }

  // SoA variant of `complete_match()`: appends one match to `out`. Falls back
  // to a fresh geometric match if the precomputed candidates yield nothing.
  template <Profile P>
  void complete_match(P::parameters const& params,
                      location const& query,
                      bool const reverse,
                      direction const search_dir,
                      double max_match_distance,
                      bitvec<node_idx_t> const* blocked,
                      std::optional<routing_time_t> const start_time,
                      std::span<raw_way_candidate const> raw_way_candidates,
                      match_result& out) const {
    out.start(query.lvl_);
    auto doublings = 0U;
    auto const added =
        append_raw<P>(params, query, reverse, search_dir, max_match_distance,
                      blocked, start_time, raw_way_candidates, doublings, out);
    // Cold path: no precomputed candidate was usable. Reuse the AoS matcher
    // verbatim so the (subtle) widening semantics stay identical, and convert
    // its result. Rare enough that the temporary AoS vector does not matter.
    if (added == 0U && doublings < 4U) {
      for (auto const& wc :
           match<P>(params, query, reverse, search_dir, max_match_distance,
                    blocked, start_time)) {
        out.add(static_cast<float>(wc.dist_to_way_), wc.way_,
                match_result::nodes{
                    .left_ = {.node_ = wc.left_.node_,
                              .dist_to_node_ =
                                  static_cast<float>(wc.left_.dist_to_node_),
                              .cost_ = wc.left_.cost_},
                    .right_ = {.node_ = wc.right_.node_,
                               .dist_to_node_ =
                                   static_cast<float>(wc.right_.dist_to_node_),
                               .cost_ = wc.right_.cost_}});
      }
    }
    out.finish();
  }

  // Converts raw (geometric, profile independent) candidates into profile
  // aware ones, appending them to the match currently being built.
  template <Profile P>
  unsigned append_raw(P::parameters const& params,
                      location const& query,
                      bool const reverse,
                      direction const search_dir,
                      double const max_match_distance,
                      bitvec<node_idx_t> const* blocked,
                      std::optional<routing_time_t> const start_time,
                      std::span<raw_way_candidate const> raw_way_candidates,
                      unsigned& doublings,
                      match_result& out) const {
    auto dist = max_match_distance;
    auto added = 0U;
    for (auto const& raw_wc : raw_way_candidates) {
      while (raw_wc.dist_to_way_ >= dist && added == 0U && doublings++ < 4U) {
        dist *= 2U;
      }
      if (raw_wc.dist_to_way_ >= dist) {
        break;
      }
      auto n = match_result::nodes{
          .left_ = {.node_ = raw_wc.left_.node_,
                    .dist_to_node_ = raw_wc.left_.dist_to_node_},
          .right_ = {.node_ = raw_wc.right_.node_,
                     .dist_to_node_ = raw_wc.right_.dist_to_node_}};
      apply_node_cost<P>(params, raw_wc.way_, n.left_, direction::kBackward,
                         query, reverse, search_dir, blocked, start_time);
      apply_node_cost<P>(params, raw_wc.way_, n.right_, direction::kForward,
                         query, reverse, search_dir, blocked, start_time);
      if (n.left_.valid() || n.right_.valid()) {
        out.add(raw_wc.dist_to_way_, raw_wc.way_, n);
        ++added;
      }
    }
    return added;
  }

  // `apply_next_node_cost()` keyed on the way index instead of a
  // `way_candidate`, so it works on the SoA representation.
  template <Profile P>
  void apply_node_cost(P::parameters const& params,
                       way_idx_t const way,
                       match_result::node& nc,
                       direction const way_dir,
                       location const& query,
                       bool const reverse,
                       direction const search_dir,
                       bitvec<node_idx_t> const* blocked,
                       std::optional<routing_time_t> const start_time) const {
    if (!nc.valid()) {
      return;
    }
    if (!is_way_node_feasible<P>(params, way, nc.node_, query, reverse,
                                 search_dir)) {
      nc.node_ = node_idx_t::invalid();
      return;
    }
    auto const way_prop = ways_.r_->way_properties_[way];
    auto const edge_dir = reverse ? opposite(way_dir) : way_dir;
    auto const cost = P::way_cost(params, *ways_.r_, ways_.timezones_, way,
                                  way_prop, flip(search_dir, edge_dir),
                                  static_cast<distance_t>(nc.dist_to_node_),
                                  start_time, duration_t{0}, search_dir);
    if (cost.cost_ != kInfeasible &&
        (blocked == nullptr || !blocked->test(nc.node_))) {
      nc.cost_ = cost.cost_;
    } else {
      nc.node_ = node_idx_t::invalid();
    }
  }

  // Recomputes the geometry between the query position and the candidate
  // node. Used by path reconstruction; the SoA match representation never
  // caches it.
  std::vector<geo::latlng> get_node_candidate_path(
      way_idx_t const way,
      node_idx_t const node,
      direction const way_dir,
      bool const reverse,
      location const& query) const {
    auto const approx_distance_lng_degrees =
        geo::approx_distance_lng_degrees(query.pos_);
    auto const [squared_dist, best, segment_idx] =
        geo::approx_squared_distance_to_polyline<
            std::tuple<double, geo::latlng, size_t>>(
            query.pos_, ways_.way_polylines_[way],
            approx_distance_lng_degrees);
    auto const polyline = ways_.way_polylines_[way];
    auto const osm_nodes = ways_.way_osm_nodes_[way];
    auto path = std::vector<geo::latlng>{best};
    till_the_end(segment_idx + (way_dir == direction::kForward ? 1U : 0U),
                 utl::zip(polyline, osm_nodes), way_dir, [&](auto&& x) {
                   auto const& [pos, osm_node_idx] = x;
                   path.push_back(pos);
                   if (ways_.node_to_osm_[node] == osm_node_idx) {
                     return utl::cflow::kBreak;
                   }
                   return utl::cflow::kContinue;
                 });

    if (reverse) {
      std::reverse(begin(path), end(path));
    }
    return path;
  }

  std::vector<geo::latlng> get_node_candidate_path(
      way_candidate const& wc,
      node_candidate const& nc,
      bool const reverse,
      location const& query) const {
    if (!nc.path_.empty() || !nc.valid()) {
      return nc.path_;
    }
    return get_node_candidate_path(wc.way_, nc.node_, nc.way_dir_, reverse,
                                   query);
  }

  match_t match(profile_parameters const& params,
                location const& query,
                bool const reverse,
                direction const search_dir,
                double const max_match_distance,
                bitvec<node_idx_t> const* blocked,
                search_profile,
                std::optional<std::span<raw_way_candidate const>>
                    raw_way_candidates = std::nullopt) const;

  void match(profile_parameters const& params,
             location const& query,
             bool reverse,
             direction search_dir,
             double max_match_distance,
             bitvec<node_idx_t> const* blocked,
             search_profile,
             std::span<raw_way_candidate const> raw_way_candidates,
             match_result& out) const;

  template <Profile P>
  match_t match(P::parameters const& params,
                location const& query,
                bool const reverse,
                direction const search_dir,
                double max_match_distance,
                bitvec<node_idx_t> const* blocked,
                std::optional<routing_time_t> const start_time = std::nullopt,
                std::optional<std::span<raw_way_candidate const>>
                    raw_way_candidates = std::nullopt) const {
    if (raw_way_candidates.has_value()) {
      return complete_match<P>(params, query, reverse, search_dir,
                               max_match_distance, blocked, start_time,
                               *raw_way_candidates);
    }
    auto way_candidates =
        get_way_candidates<P>(params, query, reverse, search_dir,
                              max_match_distance, blocked, start_time);
    auto i = 0U;
    while (way_candidates.empty() && i++ < 4U) {
      max_match_distance *= 2U;
      way_candidates =
          get_way_candidates<P>(params, query, reverse, search_dir,
                                max_match_distance, blocked, start_time);
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

  template <Profile P>
  match_t get_way_candidates(
      P::parameters const& params,
      location const& query,
      bool const reverse,
      direction const search_dir,
      double const max_match_distance,
      bitvec<node_idx_t> const* blocked,
      std::optional<routing_time_t> const start_time = std::nullopt) const {
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
        auto wc =
            way_candidate{.dist_to_way_ = std::sqrt(squared_dist),
                          .way_ = way,
                          .closest_point_on_way_ = best,
                          .segment_idx_ = static_cast<unsigned>(segment_idx)};
        wc.left_ = find_next_node<P>(params, wc, query, direction::kBackward,
                                     query.lvl_, reverse, search_dir, blocked,
                                     approx_distance_lng_degrees, best,
                                     segment_idx, start_time);
        wc.right_ = find_next_node<P>(params, wc, query, direction::kForward,
                                      query.lvl_, reverse, search_dir, blocked,
                                      approx_distance_lng_degrees, best,
                                      segment_idx, start_time);
        if (wc.left_.valid() || wc.right_.valid()) {
          way_candidates.emplace_back(std::move(wc));
        }
      }
    });
    utl::sort(way_candidates);
    return way_candidates;
  }

  template <Profile P>
  bool is_way_node_feasible(P::parameters const& params,
                            way_idx_t const way,
                            node_idx_t const node_idx,
                            location const& query,
                            bool const reverse,
                            direction const search_dir) const {
    auto const node_prop = ways_.r_->node_properties_[node_idx];
    if (P::node_cost(params, node_prop).cost_ == kInfeasible) {
      return false;
    }
    auto found = false;
    P::resolve_start_node(*ways_.r_, way, node_idx, query.lvl_,
                          reverse ? opposite(search_dir) : search_dir,
                          [&](auto const) { found = true; });
    return found;
  }

  template <Profile P>
  bool is_way_node_feasible(P::parameters const& params,
                            way_candidate const& wc,
                            node_idx_t const node_idx,
                            location const& query,
                            bool const reverse,
                            direction const search_dir) const {
    return is_way_node_feasible<P>(params, wc.way_, node_idx, query, reverse,
                                   search_dir);
  }

  template <Profile P>
  node_candidate find_next_node(
      P::parameters const& params,
      way_candidate const& wc,
      location const& query,
      direction const dir,
      level_t const lvl,
      bool const reverse,
      direction const search_dir,
      bitvec<node_idx_t> const* blocked,
      double approx_distance_lng_degrees,
      geo::latlng const best,
      size_t segment_idx,
      std::optional<routing_time_t> const start_time = std::nullopt,
      // Only map matching consumes `node_candidate::path_` directly. Routing
      // recomputes the geometry for the one candidate it actually uses via
      // `get_node_candidate_path()`, so filling it for every candidate here is
      // wasted work.
      bool const with_path = false) const {
    auto const way_prop = ways_.r_->way_properties_[wc.way_];
    auto const edge_dir = reverse ? opposite(dir) : dir;
    if (P::way_cost(params, *ways_.r_, ways_.timezones_, wc.way_, way_prop,
                    flip(search_dir, edge_dir), 0U, start_time, duration_t{0},
                    search_dir)
            .cost_ == kInfeasible) {
      return node_candidate{};
    }

    auto c = node_candidate{.lvl_ = lvl,
                            .way_dir_ = dir,
                            .dist_to_node_ = wc.dist_to_way_,
                            .cost_ = 0};
    if (with_path) {
      c.path_ = {best};
    }
    auto last_path_pos = best;
    auto const polyline = ways_.way_polylines_[wc.way_];
    auto const osm_nodes = ways_.way_osm_nodes_[wc.way_];

    till_the_end(
        segment_idx + (dir == direction::kForward ? 1U : 0U),
        utl::zip(polyline, osm_nodes), dir, [&](auto&& x) {
          auto const& [pos, osm_node_idx] = x;

          auto const segment_dist = std::sqrt(geo::approx_squared_distance(
              last_path_pos, pos, approx_distance_lng_degrees));
          c.dist_to_node_ += segment_dist;
          last_path_pos = pos;
          if (with_path) {
            c.path_.push_back(pos);
          }

          auto const way_node = ways_.find_node_idx(osm_node_idx);
          if (way_node.has_value()) {
            if (is_way_node_feasible<P>(params, wc, *way_node, query, reverse,
                                        search_dir) &&
                (blocked == nullptr || !blocked->test(*way_node))) {
              c.node_ = *way_node;
              c.cost_ =
                  P::way_cost(params, *ways_.r_, ways_.timezones_, wc.way_,
                              way_prop, flip(search_dir, edge_dir),
                              static_cast<distance_t>(c.dist_to_node_),
                              start_time, duration_t{0}, search_dir)
                      .cost_;
            }
            return utl::cflow::kBreak;
          }

          return utl::cflow::kContinue;
        });

    if (with_path && (reverse ^ (search_dir == direction::kBackward))) {
      std::reverse(begin(c.path_), end(c.path_));
    }
    return c;
  }

private:
  std::vector<raw_way_candidate> get_raw_way_candidates(
      location const& query, double const max_match_distance) const;

  raw_node_candidate find_raw_next_node(raw_way_candidate const&,
                                        direction const,
                                        double,
                                        geo::latlng const,
                                        size_t) const;

  template <Profile P>
  void apply_next_node_cost(
      P::parameters const& params,
      way_candidate const& wc,
      node_candidate& nc,
      location const& query,
      bool const reverse,
      direction const search_dir,
      bitvec<node_idx_t> const* blocked,
      std::optional<routing_time_t> const start_time) const {
    if (!nc.valid()) {
      return;
    }
    if (!is_way_node_feasible<P>(params, wc, nc.node_, query, reverse,
                                 search_dir)) {
      nc.node_ = node_idx_t::invalid();
      return;
    }
    auto const way_prop = ways_.r_->way_properties_[wc.way_];

    auto const edge_dir = reverse ? opposite(nc.way_dir_) : nc.way_dir_;
    auto const cost = P::way_cost(params, *ways_.r_, ways_.timezones_, wc.way_,
                                  way_prop, flip(search_dir, edge_dir),
                                  static_cast<distance_t>(nc.dist_to_node_),
                                  start_time, duration_t{0}, search_dir);

    if (cost.cost_ != kInfeasible &&
        (blocked == nullptr || !blocked->test(nc.node_))) {
      nc.cost_ = cost.cost_;
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
