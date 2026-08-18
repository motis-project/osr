#include "osr/lookup.h"

#include <cmath>
#include <array>
#include <numbers>
#include <optional>
#include <vector>

#include "utl/helpers/algorithm.h"
#include "utl/parallel_for.h"

#include "osr/routing/parameters.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/with_profile.h"

namespace osr {

namespace {

// Near-ties to the closest candidate of a component are kept as well.
constexpr auto const kTieTolerance = 1.0F;  // meters
constexpr auto const kTieFactor = 0.25F;  // of the closest distance

struct kept_candidate {
  float dist_{};
  component_classes kept_by_{};
  component_classes unpenalized_{};
  component_classes oneway_{};
  std::array<std::optional<std::uint32_t>, kNumComponentClasses> component_{};
  bool on_target_level_{};
  level_t min_level_{kNoLevel}, max_level_{kNoLevel};
  std::array<double, 2> heading_{};  // travel direction, oneways only
};

std::array<double, 2> get_travel_heading(ways const& w,
                                         way_idx_t const way,
                                         way_properties const& props,
                                         geo::latlng const& pos) {
  auto const polyline = w.way_polylines_[way];
  auto const [squared_dist, best, segment_idx] =
      geo::approx_squared_distance_to_polyline<
          std::tuple<double, geo::latlng, size_t>>(
          pos, polyline, geo::approx_distance_lng_degrees(pos));
  auto const i = std::min<std::size_t>(segment_idx, polyline.size() - 2U);
  geo::latlng const a = polyline[i];
  geo::latlng const b = polyline[i + 1U];
  auto const sign = props.is_oneway_reverse() ? -1.0 : 1.0;
  auto const cos_lat = std::cos(pos.lat() * std::numbers::pi / 180.0);
  return {sign * (b.lng() - a.lng()) * cos_lat, sign * (b.lat() - a.lat())};
}

// Candidates arrive sorted by distance. Each class decides on its own and a
// candidate is kept if any class keeps it (or no class can use its way). A
// class drops a candidate if a kept candidate in the same component is
// clearly closer and at least as good:
// - not penalised if this one is not (e.g. foot on a `foot=no` cycleway)
// - on the target level (the query level, or the ground without one), or both
//   are off it on overlapping levels
// - for oneways: heading the same way (not the other carriageway)
std::optional<kept_candidate> keep_candidate(
    ways const& w,
    std::vector<kept_candidate> const& kept,
    component_classes const classes,
    location const& query,
    way_idx_t const way,
    float const dist) {
  auto const& props = w.r_->way_properties_[way];
  auto k = kept_candidate{
      .dist_ = dist,
      .on_target_level_ = query.lvl_.has_level()
                              ? is_on_level(props, query.lvl_)
                              : touches_ground(props),
      .min_level_ = std::min(props.from_level(), props.to_level()),
      .max_level_ = std::max(props.from_level(), props.to_level())};
  if (utl::any_of(kComponentClasses, [&](auto const& e) {
        return classes.contains(e.first) && is_oneway(e.first, props);
      })) {
    k.heading_ = get_travel_heading(w, way, props, query.pos_);
  }

  auto voted = false;
  for (auto const& [c, name] : kComponentClasses) {
    if (!classes.contains(c) || !is_accessible(c, props)) {
      continue;
    }
    voted = true;
    auto const ci = static_cast<std::size_t>(c);
    auto const id = w.r_->get_class_components(c).get(way);
    auto const unpenalized = is_accessible_without_penalty(c, props);
    auto const oneway = is_oneway(c, props);
    auto const shadows = [&](kept_candidate const& x) {
      return x.kept_by_.contains(c) && x.component_[ci] == id &&
             (x.unpenalized_.contains(c) || !unpenalized) &&
             (!x.oneway_.contains(c) ||
              (oneway &&
               x.heading_[0] * k.heading_[0] + x.heading_[1] * k.heading_[1] >
                   0.0)) &&
             (x.on_target_level_ ||
              (!k.on_target_level_ && x.min_level_ <= k.max_level_ &&
               k.min_level_ <= x.max_level_));
    };
    auto const shadow =
        id.has_value() ? utl::find_if(kept, shadows) : end(kept);
    if (shadow != end(kept) &&
        dist - shadow->dist_ >
            std::max(kTieTolerance, kTieFactor * shadow->dist_)) {
      continue;
    }
    k.kept_by_.insert(c);
    k.component_[ci] = id;
    if (unpenalized) {
      k.unpenalized_.insert(c);
    }
    if (oneway) {
      k.oneway_.insert(c);
    }
  }
  return !voted || !k.kept_by_.empty() ? std::optional{k} : std::nullopt;
}

}  // namespace

lookup::lookup(ways const& ways,
               std::filesystem::path p,
               cista::mmap::protection mode)
    : p_{std::move(p)},
      mode_{mode},
      rtree_{mode == cista::mmap::protection::READ
                 ? *cista::read<cista::mm_rtree<way_idx_t>::meta>(
                       p_ / "rtree_meta.bin")
                 : cista::mm_rtree<way_idx_t>::meta{},
             cista::mm_rtree<way_idx_t>::vector_t{mm("rtree_data.bin")}},
      ways_{ways} {}

void lookup::build_rtree() {
  auto sorted_ways = std::vector<way_idx_t>();
  sorted_ways.resize(ways_.n_ways());
  std::iota(sorted_ways.begin(), sorted_ways.end(), way_idx_t{0});
  auto curve = vec_map<way_idx_t, std::uint64_t>{};
  curve.resize(ways_.n_ways());

  utl::parallel_for(sorted_ways, [&](way_idx_t const way) {
    curve[way] = geo::morton_encode(
        ways_.way_polylines_[way][ways_.way_polylines_[way].size() / 2]);
  });

  std::stable_sort(sorted_ways.begin(), sorted_ways.end(),
                   [&](way_idx_t const a, way_idx_t const b) {
                     return curve[a] < curve[b];
                   });

  for (auto way : sorted_ways) {
    auto b = geo::box{};
    for (auto const& c : ways_.way_polylines_[way]) {
      b.extend(c);
    }
    rtree_.insert(b.min_.lnglat_float(), b.max_.lnglat_float(), way);
  }
  rtree_.write_meta(p_ / "rtree_meta.bin");
}

std::vector<raw_way_candidate> lookup::get_raw_way_candidates(
    location const& query, double const max_match_distance) const {
  auto way_candidates = std::vector<raw_way_candidate>{};
  auto const approx_distance_lng_degrees =
      geo::approx_distance_lng_degrees(query.pos_);
  auto const squared_max_dist = std::pow(max_match_distance, 2);
  find(geo::box{query.pos_, max_match_distance}, [&](way_idx_t const way) {
    auto const [squared_dist, best, segment_idx] =
        geo::approx_squared_distance_to_polyline<
            std::tuple<double, geo::latlng, size_t>>(
            query.pos_, ways_.way_polylines_[way], approx_distance_lng_degrees);
    if (squared_dist < squared_max_dist) {
      auto raw_wc = get_raw_way_candidate(
          way, squared_dist, approx_distance_lng_degrees, best, segment_idx);
      if (raw_wc.left_.valid() || raw_wc.right_.valid()) {
        way_candidates.emplace_back(std::move(raw_wc));
      }
    }
  });
  utl::sort(way_candidates);
  return way_candidates;
}

lookup::way_stretch lookup::get_way_stretch(way_idx_t const way,
                                            geo::latlng const& pos) const {
  auto const approx_distance_lng_degrees =
      geo::approx_distance_lng_degrees(pos);
  auto const [squared_dist, best, segment_idx] =
      geo::approx_squared_distance_to_polyline<
          std::tuple<double, geo::latlng, size_t>>(
          pos, ways_.way_polylines_[way], approx_distance_lng_degrees);
  auto const wc = get_raw_way_candidate(
      way, squared_dist, approx_distance_lng_degrees, best, segment_idx);
  auto const offset =
      static_cast<double>(wc.left_.dist_to_node_ - wc.dist_to_way_);
  return {.candidate_ = wc,
          .projection_ = best,
          .segment_idx_ = segment_idx,
          .offset_ = offset};
}

void lookup::filter_by_component(match_result& out,
                                 location const& query,
                                 component_classes const classes) const {
  using idx_t = match_result::way_candidate_idx_t;

  auto const from = to_idx(out.begin_.back());
  auto const to = static_cast<std::uint32_t>(out.way_.size());
  if (classes.empty() || to - from < 2U) {
    return;
  }

  auto kept = std::vector<kept_candidate>{};
  kept.reserve(to - from);
  auto write = from;
  for (auto read = from; read != to; ++read) {
    auto const k =
        keep_candidate(ways_, kept, classes, query, out.way_[idx_t{read}],
                       out.dist_to_way_[idx_t{read}]);
    if (!k.has_value()) {
      continue;
    }
    if (write != read) {
      out.dist_to_way_[idx_t{write}] = out.dist_to_way_[idx_t{read}];
      out.way_[idx_t{write}] = out.way_[idx_t{read}];
      out.nodes_[idx_t{write}] = out.nodes_[idx_t{read}];
    }
    kept.emplace_back(*k);
    ++write;
  }

  out.dist_to_way_.resize(write);
  out.way_.resize(write);
  out.nodes_.resize(write);
}

void lookup::set_penalty_reference(match_result& out,
                                   level_t const query_lvl) const {
  using idx_t = match_result::way_candidate_idx_t;

  if (query_lvl.has_level()) {
    return;
  }
  auto const to = static_cast<std::uint32_t>(out.way_.size());
  for (auto i = to_idx(out.begin_.back()); i != to; ++i) {
    if (touches_ground(ways_.r_->way_properties_[out.way_[idx_t{i}]])) {
      out.penalty_ref_.back() = out.dist_to_way_[idx_t{i}];
      return;
    }
  }
}

std::vector<raw_way_candidate> lookup::get_raw_match(
    location const& query, double max_match_distance) const {
  auto const covers_all_base_profiles =
      [&](std::vector<raw_way_candidate> const& candidates) {
        return utl::all_of(
            std::array{search_profile::kFoot, search_profile::kBike,
                       search_profile::kCar},
            [&](search_profile const p) {
              auto const params = get_parameters(p);
              return with_profile(p, [&]<Profile P>(P&&) {
                return utl::any_of(candidates, [&](auto const& wc) {
                  return is_raw_usable<P>(
                      std::get<typename P::parameters>(params), wc, query);
                });
              });
            });
      };

  auto way_candidates = get_raw_way_candidates(query, max_match_distance);
  auto i = 0U;
  while (!covers_all_base_profiles(way_candidates) && i++ < 4U) {
    max_match_distance *= 2U;
    way_candidates = get_raw_way_candidates(query, max_match_distance);
  }
  return way_candidates;
}

raw_way_candidate lookup::get_raw_way_candidate(
    way_idx_t const way,
    double const squared_dist,
    double const approx_distance_lng_degrees,
    geo::latlng const best,
    std::size_t const segment_idx) const {
  auto wc = raw_way_candidate{static_cast<float>(std::sqrt(squared_dist)), way};
  wc.left_ = find_raw_next_node(wc, direction::kBackward,
                                approx_distance_lng_degrees, best, segment_idx);
  wc.right_ = find_raw_next_node(
      wc, direction::kForward, approx_distance_lng_degrees, best, segment_idx);
  return wc;
}

raw_node_candidate lookup::find_raw_next_node(
    raw_way_candidate const& wc,
    direction const dir,
    double approx_distance_lng_degrees,
    geo::latlng const best,
    size_t segment_idx) const {
  auto c = raw_node_candidate{.dist_to_node_ = wc.dist_to_way_};
  auto const polyline = ways_.way_polylines_[wc.way_];
  auto const osm_nodes = ways_.way_osm_nodes_[wc.way_];

  auto last_path_pos = best;
  till_the_end(segment_idx + (dir == direction::kForward ? 1U : 0U),
               utl::zip(polyline, osm_nodes), dir, [&](auto&& x) {
                 auto const& [pos, osm_node_idx] = x;

                 auto const segment_dist =
                     std::sqrt(geo::approx_squared_distance(
                         last_path_pos, pos, approx_distance_lng_degrees));
                 c.dist_to_node_ += static_cast<float>(segment_dist);
                 last_path_pos = pos;

                 auto const way_node = ways_.find_node_idx(osm_node_idx);
                 if (way_node.has_value()) {
                   c.node_ = *way_node;
                   return utl::cflow::kBreak;
                 }
                 return utl::cflow::kContinue;
               });
  return c;
}

void lookup::match(profile_parameters const& params,
                   location const& query,
                   bool const reverse,
                   direction const search_dir,
                   double const max_match_distance,
                   bitvec<node_idx_t> const* blocked,
                   bool const exact_return_allowed,
                   search_profile const p,
                   std::span<raw_way_candidate const> const raw_way_candidates,
                   match_result& out) const {
  with_profile(p, [&]<Profile P>(P&&) {
    complete_match<P>(std::get<typename P::parameters>(params), query, reverse,
                      search_dir, max_match_distance, blocked,
                      exact_return_allowed, std::nullopt, raw_way_candidates,
                      out);
  });
}

hash_set<node_idx_t> lookup::find_elevators(geo::box const& b) const {
  auto elevators = hash_set<node_idx_t>{};
  find(b, [&](way_idx_t const way) {
    for (auto const n : ways_.r_->way_nodes_[way]) {
      if (ways_.r_->node_properties_[n].is_elevator()) {
        elevators.emplace(n);
      }
    }
  });
  return elevators;
}

}  // namespace osr
