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
#include "utl/verify.h"

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

struct raw_node_candidate {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  node_idx_t node_{node_idx_t::invalid()};
  float dist_to_node_{0.0};
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

using match_idx_t = cista::strong<std::uint32_t, struct match_idx_>;

// One matched way node, expanded from its stored form.
struct candidate_node {
  bool valid() const { return node_ != node_idx_t::invalid(); }

  node_idx_t node_{node_idx_t::invalid()};
  double dist_to_node_{0.0};
  cost_t cost_{0U};
  direction way_dir_{direction::kForward};
  level_t lvl_{kNoLevel};
};

// All matches of one request, in flat trivially copyable arrays bucketed per
// match. Geometry is deliberately absent - only path reconstruction needs it,
// and it recomputes it via `lookup::get_node_candidate_path()`.
struct match_result {
  // Index into the flat candidate arrays. Internal: callers address whole
  // matches via `match_idx_t`, never individual candidates.
  using way_candidate_idx_t =
      cista::strong<std::uint32_t, struct way_candidate_idx_>;

  struct node {
    bool valid() const { return node_ != node_idx_t::invalid(); }

    node_idx_t node_{node_idx_t::invalid()};
    float dist_to_node_{0.0F};
    cost_t cost_{0U};
  };

  struct nodes {
    node left_{}, right_{};
  };

  struct flags {
    bool vehicle_match_{};
    bool exact_return_{};
  };

  // View of a single match (= all way candidates of one query location).
  struct view {
    bool empty() const { return way_.empty(); }
    std::size_t size() const { return way_.size(); }

    candidate_node left(std::size_t const j) const {
      auto const& n = nodes_[j].left_;
      return {n.node_, n.dist_to_node_, n.cost_, direction::kBackward, lvl_};
    }
    candidate_node right(std::size_t const j) const {
      auto const& n = nodes_[j].right_;
      return {n.node_, n.dist_to_node_, n.cost_, direction::kForward, lvl_};
    }

    bool vehicle_match(std::size_t const j) const {
      return flags_[j].vehicle_match_;
    }
    bool exact_return(std::size_t const j) const {
      return flags_[j].exact_return_;
    }

    std::span<float const> dist_to_way_{};
    std::span<way_idx_t const> way_{};
    std::span<nodes const> nodes_{};
    std::span<flags const> flags_{};
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
    flags_.clear();
  }

  std::size_t size() const { return lvl_.size(); }
  bool empty() const { return lvl_.empty(); }

  // Appending a match: start() -> add()* -> finish().
  void start(level_t const lvl) { lvl_.emplace_back(lvl); }

  void add(float const dist_to_way, way_idx_t const w, nodes const& n) {
    add(dist_to_way, w, n, flags{false, false});
  }

  void add(float const dist_to_way,
           way_idx_t const w,
           nodes const& n,
           flags const f) {
    dist_to_way_.emplace_back(dist_to_way);
    way_.emplace_back(w);
    nodes_.emplace_back(n);
    flags_.emplace_back(f);
  }

  void finish() { begin_.emplace_back(way_candidate_idx_t{(way_.size())}); }

  // Appends a copy of one bucket of `src`. Used to gather a scattered subset
  // of precomputed matches into the contiguous form `route()` consumes.
  void append(match_result const& src, match_idx_t const i) {
    auto const v = src[i];
    start(v.lvl_);
    for (auto j = std::size_t{0U}; j != v.size(); ++j) {
      add(v.dist_to_way_[j], v.way_[j], v.nodes_[j], v.flags_[j]);
    }
    finish();
  }

  view operator[](match_idx_t const i) const {
    utl::verify(to_idx(i) + 1U < begin_.size(),
                "match_result: match {} out of range (size={})", to_idx(i),
                size());

    auto const from = to_idx(begin_[i]);
    auto const to = to_idx(begin_[match_idx_t{to_idx(i) + 1U}]);
    auto const n = static_cast<std::size_t>(to - from);
    auto const at = [&](auto const& v) {
      using value_t = std::decay_t<decltype(v[way_candidate_idx_t{0U}])>;
      return n == 0U ? std::span<value_t const>{}
                     : std::span{&v[way_candidate_idx_t{from}], n};
    };
    return view{.dist_to_way_ = at(dist_to_way_),
                .way_ = at(way_),
                .nodes_ = at(nodes_),
                .flags_ = at(flags_),
                .lvl_ = lvl_[i]};
  }

  vec_map<match_idx_t, way_candidate_idx_t> begin_{};  // size() + 1 entries
  vec_map<match_idx_t, level_t> lvl_{};
  vec_map<way_candidate_idx_t, float> dist_to_way_{};
  vec_map<way_candidate_idx_t, way_idx_t> way_{};
  vec_map<way_candidate_idx_t, nodes> nodes_{};
  vec_map<way_candidate_idx_t, flags> flags_{};
};

// One match, borrowed from the `match_result` that owns it.
using match_view_t = match_result::view;

struct lookup {
  lookup(ways const&, std::filesystem::path, cista::mmap::protection);

  void build_rtree();

  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  std::vector<raw_way_candidate> get_raw_match(location const&,
                                               double max_match_distance) const;

  // Turns precomputed (geometric) candidates into profile aware ones and
  // appends them to `out` as one match. Falls back to a fresh geometric match
  // if none of the precomputed candidates is usable.
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
    // Cold path: no precomputed candidate was usable, so run the full match.
    if (!added && doublings < 4U) {
      auto dist = max_match_distance;
      auto found = get_way_candidates<P>(params, query, reverse, search_dir,
                                         dist, blocked, out, start_time);
      auto i = 0U;
      while (!found && i++ < 4U) {
        dist *= 2U;
        found = get_way_candidates<P>(params, query, reverse, search_dir, dist,
                                      blocked, out, start_time);
      }
    }
    out.finish();
  }

  // Converts raw (geometric, profile independent) candidates into profile
  // aware ones, appending them to the match currently being built. Returns
  // whether any candidate was usable.
  template <Profile P>
  bool append_raw(P::parameters const& params,
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
    auto added = false;
    for (auto const& raw_wc : raw_way_candidates) {
      while (raw_wc.dist_to_way_ >= dist && !added && doublings++ < 4U) {
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
        added = true;
      }
    }
    return added;
  }

  // Applies profile feasibility and cost to a matched way node.
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
  // node. Matches do not cache it: only path reconstruction needs it, and only
  // for the one candidate it actually uses.
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
            query.pos_, ways_.way_polylines_[way], approx_distance_lng_degrees);
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

  void match(profile_parameters const& params,
             location const& query,
             bool reverse,
             direction search_dir,
             double max_match_distance,
             bitvec<node_idx_t> const* blocked,
             search_profile,
             std::span<raw_way_candidate const> raw_way_candidates,
             match_result& out) const;

  void match_endpoint(
      profile_parameters const& params,
      location const& query,
      bool reverse,
      direction search_dir,
      double max_match_distance,
      bitvec<node_idx_t> const* blocked,
      search_profile,
      bool exact_return_allowed,
      std::optional<std::span<raw_way_candidate const>> raw_way_candidates,
      bool include_non_exact_vehicle_matches,
      match_result& out) const;

  // Matches `query` against the street network and appends the result to
  // `out` as one match. If nothing is found, the search is retried at twice
  // the distance, up to four times.
  template <Profile P>
  void match(
      P::parameters const& params,
      location const& query,
      bool const reverse,
      direction const search_dir,
      double max_match_distance,
      bitvec<node_idx_t> const* blocked,
      match_result& out,
      std::optional<routing_time_t> const start_time = std::nullopt) const {
    out.start(query.lvl_);
    auto found =
        get_way_candidates<P>(params, query, reverse, search_dir,
                              max_match_distance, blocked, out, start_time);
    auto i = 0U;
    while (!found && i++ < 4U) {
      max_match_distance *= 2U;
      found =
          get_way_candidates<P>(params, query, reverse, search_dir,
                                max_match_distance, blocked, out, start_time);
    }
    out.finish();
  }

  template <Profile P>
  void match_endpoint(
      P::parameters const& params,
      location const& query,
      bool const reverse,
      direction const search_dir,
      double const max_match_distance,
      bitvec<node_idx_t> const* blocked,
      bool const exact_return_allowed,
      match_result& out,
      std::optional<routing_time_t> const start_time = std::nullopt,
      std::optional<std::span<raw_way_candidate const>> raw_way_candidates =
          std::nullopt,
      bool const include_non_exact_vehicle_matches = true) const {
    auto raw_way_candidates_storage = std::vector<raw_way_candidate>{};
    if constexpr (SharingProfile<P>) {
      if (exact_return_allowed && !raw_way_candidates.has_value()) {
        // Reuse one geometric/R-tree match for both the regular foot endpoint
        // and the exact vehicle return endpoint.
        raw_way_candidates_storage = get_raw_match(query, max_match_distance);
        raw_way_candidates = raw_way_candidates_storage;
      }
    }

    auto regular_result = match_result{};
    if (raw_way_candidates.has_value()) {
      complete_match<P>(params, query, reverse, search_dir, max_match_distance,
                        blocked, start_time, *raw_way_candidates,
                        regular_result);
    } else {
      match<P>(params, query, reverse, search_dir, max_match_distance, blocked,
               regular_result, start_time);
    }
    auto const regular = regular_result[match_idx_t{0U}];

    auto vehicle_result = match_result{};
    if constexpr (SharingProfile<P>) {
      if (exact_return_allowed) {
        if (raw_way_candidates.has_value()) {
          complete_match<typename P::vehiclep>(
              P::vehicle_parameters(params), query, reverse, search_dir,
              max_match_distance, blocked, start_time, *raw_way_candidates,
              vehicle_result);
        } else {
          match<typename P::vehiclep>(P::vehicle_parameters(params), query,
                                      reverse, search_dir, max_match_distance,
                                      blocked, vehicle_result, start_time);
        }
      } else if (include_non_exact_vehicle_matches) {
        if (raw_way_candidates.has_value()) {
          complete_match<typename P::vehiclep>(
              P::vehicle_parameters(params), query, reverse, search_dir,
              max_match_distance, blocked, start_time, *raw_way_candidates,
              vehicle_result);
        } else {
          vehicle_result.start(query.lvl_);
          auto const approx_distance_lng_degrees =
              geo::approx_distance_lng_degrees(query.pos_);
          for (auto i = std::size_t{0U}; i != regular.size(); ++i) {
            auto const way = regular.way_[i];
            auto const [squared_dist, best, segment_idx] =
                geo::approx_squared_distance_to_polyline<
                    std::tuple<double, geo::latlng, size_t>>(
                    query.pos_, ways_.way_polylines_[way],
                    approx_distance_lng_degrees);
            auto const dist_to_way = std::sqrt(squared_dist);
            auto const left = find_next_node<typename P::vehiclep>(
                P::vehicle_parameters(params), way, dist_to_way, query,
                direction::kBackward, query.lvl_, reverse, search_dir, blocked,
                approx_distance_lng_degrees, best, segment_idx, start_time);
            auto const right = find_next_node<typename P::vehiclep>(
                P::vehicle_parameters(params), way, dist_to_way, query,
                direction::kForward, query.lvl_, reverse, search_dir, blocked,
                approx_distance_lng_degrees, best, segment_idx, start_time);
            if (left.valid() || right.valid()) {
              vehicle_result.add(
                  static_cast<float>(dist_to_way), way,
                  match_result::nodes{
                      .left_ = {.node_ = left.node_,
                                .dist_to_node_ =
                                    static_cast<float>(left.dist_to_node_),
                                .cost_ = left.cost_},
                      .right_ = {.node_ = right.node_,
                                 .dist_to_node_ =
                                     static_cast<float>(right.dist_to_node_),
                                 .cost_ = right.cost_}});
            }
          }
          vehicle_result.finish();
        }
      }
    }

    out.start(query.lvl_);
    if constexpr (SharingProfile<P>) {
      auto const vehicle = vehicle_result.empty()
                               ? match_view_t{}
                               : vehicle_result[match_idx_t{0U}];
      auto regular_idx = std::size_t{0U};
      auto vehicle_idx = std::size_t{0U};
      while (regular_idx != regular.size() || vehicle_idx != vehicle.size()) {
        while (vehicle_idx != vehicle.size() && !exact_return_allowed &&
               raw_way_candidates.has_value()) {
          auto const vehicle_way = vehicle.way_[vehicle_idx];
          auto const matches_regular = [&] {
            for (auto i = std::size_t{0U}; i != regular.size(); ++i) {
              if (regular.way_[i] == vehicle_way) {
                return true;
              }
            }
            return false;
          }();
          if (matches_regular) {
            break;
          }
          ++vehicle_idx;
        }
        if (regular_idx == regular.size() && vehicle_idx == vehicle.size()) {
          break;
        }
        auto const take_regular = vehicle_idx == vehicle.size() ||
                                  (regular_idx != regular.size() &&
                                   regular.dist_to_way_[regular_idx] <=
                                       vehicle.dist_to_way_[vehicle_idx]);
        if (take_regular) {
          out.add(regular.dist_to_way_[regular_idx], regular.way_[regular_idx],
                  regular.nodes_[regular_idx]);
          ++regular_idx;
        } else {
          out.add(
              vehicle.dist_to_way_[vehicle_idx], vehicle.way_[vehicle_idx],
              vehicle.nodes_[vehicle_idx],
              {.vehicle_match_ = true, .exact_return_ = exact_return_allowed});
          ++vehicle_idx;
        }
      }
    } else {
      for (auto i = std::size_t{0U}; i != regular.size(); ++i) {
        out.add(regular.dist_to_way_[i], regular.way_[i], regular.nodes_[i]);
      }
    }
    out.finish();
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

  // Appends the geometric+profile matches for `query` to `out` (between start()
  // and finish()) and returns whether any was found.
  template <Profile P>
  bool get_way_candidates(
      P::parameters const& params,
      location const& query,
      bool const reverse,
      direction const search_dir,
      double const max_match_distance,
      bitvec<node_idx_t> const* blocked,
      match_result& out,
      std::optional<routing_time_t> const start_time = std::nullopt) const {
    struct tmp_candidate {
      double dist_to_way_;
      way_idx_t way_;
      candidate_node left_, right_;
    };
    auto way_candidates = std::vector<tmp_candidate>{};
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
        auto const dist_to_way = std::sqrt(squared_dist);
        auto const left = find_next_node<P>(
            params, way, dist_to_way, query, direction::kBackward, query.lvl_,
            reverse, search_dir, blocked, approx_distance_lng_degrees, best,
            segment_idx, start_time);
        auto const right = find_next_node<P>(
            params, way, dist_to_way, query, direction::kForward, query.lvl_,
            reverse, search_dir, blocked, approx_distance_lng_degrees, best,
            segment_idx, start_time);
        if (left.valid() || right.valid()) {
          way_candidates.emplace_back(
              tmp_candidate{dist_to_way, way, left, right});
        }
      }
    });
    utl::sort(way_candidates,
              [](tmp_candidate const& a, tmp_candidate const& b) {
                return a.dist_to_way_ < b.dist_to_way_;
              });
    for (auto const& wc : way_candidates) {
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
    return !way_candidates.empty();
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
  candidate_node find_next_node(
      P::parameters const& params,
      way_idx_t const way,
      double const dist_to_way,
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
      std::vector<geo::latlng>* path = nullptr) const {
    auto const way_prop = ways_.r_->way_properties_[way];
    auto const edge_dir = reverse ? opposite(dir) : dir;
    if (P::way_cost(params, *ways_.r_, ways_.timezones_, way, way_prop,
                    flip(search_dir, edge_dir), 0U, start_time, duration_t{0},
                    search_dir)
            .cost_ == kInfeasible) {
      return candidate_node{};
    }

    auto c = candidate_node{
        .dist_to_node_ = dist_to_way, .cost_ = 0, .way_dir_ = dir, .lvl_ = lvl};
    if (path != nullptr) {
      *path = {best};
    }
    auto last_path_pos = best;
    auto const polyline = ways_.way_polylines_[way];
    auto const osm_nodes = ways_.way_osm_nodes_[way];

    till_the_end(
        segment_idx + (dir == direction::kForward ? 1U : 0U),
        utl::zip(polyline, osm_nodes), dir, [&](auto&& x) {
          auto const& [pos, osm_node_idx] = x;

          auto const segment_dist = std::sqrt(geo::approx_squared_distance(
              last_path_pos, pos, approx_distance_lng_degrees));
          c.dist_to_node_ += segment_dist;
          last_path_pos = pos;
          if (path != nullptr) {
            path->push_back(pos);
          }

          auto const way_node = ways_.find_node_idx(osm_node_idx);
          if (way_node.has_value()) {
            if (is_way_node_feasible<P>(params, way, *way_node, query, reverse,
                                        search_dir) &&
                (blocked == nullptr || !blocked->test(*way_node))) {
              c.node_ = *way_node;
              c.cost_ = P::way_cost(params, *ways_.r_, ways_.timezones_, way,
                                    way_prop, flip(search_dir, edge_dir),
                                    static_cast<distance_t>(c.dist_to_node_),
                                    start_time, duration_t{0}, search_dir)
                            .cost_;
            }
            return utl::cflow::kBreak;
          }

          return utl::cflow::kContinue;
        });

    if (path != nullptr && (reverse ^ (search_dir == direction::kBackward))) {
      std::reverse(begin(*path), end(*path));
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

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  cista::mm_rtree<way_idx_t> rtree_;
  ways const& ways_;
};

}  // namespace osr
