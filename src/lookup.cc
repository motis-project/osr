#include "osr/lookup.h"

#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/parameters.h"
#include "osr/routing/with_profile.h"

namespace osr {

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
  for (auto way = way_idx_t{0U}; way != ways_.n_ways(); ++way) {
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
      auto raw_wc = raw_way_candidate{std::sqrt(squared_dist), way};
      raw_wc.left_ =
          find_raw_next_node(raw_wc, direction::kBackward,
                             approx_distance_lng_degrees, best, segment_idx);
      raw_wc.right_ =
          find_raw_next_node(raw_wc, direction::kForward,
                             approx_distance_lng_degrees, best, segment_idx);
      if (raw_wc.left_.valid() || raw_wc.right_.valid()) {
        way_candidates.emplace_back(std::move(raw_wc));
      }
    }
  });
  utl::sort(way_candidates);
  return way_candidates;
}

std::vector<raw_way_candidate> lookup::get_raw_match(
    location const& query, double max_match_distance) const {
  auto way_candidates = get_raw_way_candidates(query, max_match_distance);
  auto i = 0U;
  while (way_candidates.empty() && i++ < 4U) {
    max_match_distance *= 2U;
    way_candidates = get_raw_way_candidates(query, max_match_distance);
  }
  return way_candidates;
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
                 c.dist_to_node_ += segment_dist;
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

match_t lookup::match(profile_parameters const& params,
                      location const& query,
                      bool const reverse,
                      direction const search_dir,
                      double const max_match_distance,
                      bitvec<node_idx_t> const* blocked,
                      search_profile const p,
                      std::optional<std::span<raw_way_candidate const>>
                          raw_way_candidates) const {
  return with_profile(p, [&]<IsProfile Profile>(Profile&&) {
    return match<Profile>(std::get<typename Profile::parameters>(params), query, reverse, search_dir, max_match_distance,
                          blocked, raw_way_candidates);
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
