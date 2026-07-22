#include "osr/lookup.h"

#include "osr/routing/parameters.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
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

uint32_t interleave(uint32_t x) {
  x = (x | (x << 8)) & 0x00FF00FF;
  x = (x | (x << 4)) & 0x0F0F0F0F;
  x = (x | (x << 2)) & 0x33333333;
  x = (x | (x << 1)) & 0x55555555;
  return x;
}

uint32_t hilbertXYToIndex_logarithmic(uint32_t x, uint32_t y) {
  uint32_t A, B, C, D;

  // Initial prefix scan round, prime with x and y
  {
    uint32_t a = x ^ y;
    uint32_t b = 0xFFFF ^ a;
    uint32_t c = 0xFFFF ^ (x | y);
    uint32_t d = x & (y ^ 0xFFFF);

    A = a | (b >> 1);
    B = (a >> 1) ^ a;

    C = ((c >> 1) ^ (b & (d >> 1))) ^ c;
    D = ((a & (c >> 1)) ^ (d >> 1)) ^ d;
  }

  {
    uint32_t a = A;
    uint32_t b = B;
    uint32_t c = C;
    uint32_t d = D;

    A = ((a & (a >> 2)) ^ (b & (b >> 2)));
    B = ((a & (b >> 2)) ^ (b & ((a ^ b) >> 2)));

    C ^= ((a & (c >> 2)) ^ (b & (d >> 2)));
    D ^= ((b & (c >> 2)) ^ ((a ^ b) & (d >> 2)));
  }

  {
    uint32_t a = A;
    uint32_t b = B;
    uint32_t c = C;
    uint32_t d = D;

    A = ((a & (a >> 4)) ^ (b & (b >> 4)));
    B = ((a & (b >> 4)) ^ (b & ((a ^ b) >> 4)));

    C ^= ((a & (c >> 4)) ^ (b & (d >> 4)));
    D ^= ((b & (c >> 4)) ^ ((a ^ b) & (d >> 4)));
  }

  // Final round and projection
  {
    uint32_t a = A;
    uint32_t b = B;
    uint32_t c = C;
    uint32_t d = D;

    C ^= ((a & (c >> 8)) ^ (b & (d >> 8)));
    D ^= ((b & (c >> 8)) ^ ((a ^ b) & (d >> 8)));
  }

  // Undo transformation prefix scan
  uint32_t a = C ^ (C >> 1);
  uint32_t b = D ^ (D >> 1);

  // Recover index bits
  uint32_t i0 = x ^ y;
  uint32_t i1 = b | (0xFFFF ^ (i0 | a));

  return (interleave(i1) << 1) | interleave(i0);
}

uint32_t hilbert(point const& p) {
  uint32_t x = ((p.lng_ + 180.0) / 360.0) * 0xFFFF;
  uint32_t y = ((p.lat_ + 90.0) / 180.0) * 0xFFFF;
  return hilbertXYToIndex_logarithmic(x, y);
}

void lookup::build_rtree() {
  auto sorted_ways = std::vector<way_idx_t>();
  sorted_ways.resize(ways_.n_ways());
  auto curve = std::vector<std::uint32_t>();
  curve.resize(ways_.n_ways());
  std::iota(sorted_ways.begin(), sorted_ways.end(), way_idx_t{0});

  for (auto way : sorted_ways) {
    curve[way.v_] = hilbert(
        ways_.way_polylines_[way][ways_.way_polylines_[way].size() / 2]);
  }

  std::stable_sort(sorted_ways.begin(), sorted_ways.end(),
                   [&](way_idx_t const a, way_idx_t const b) {
                     return curve[a.v_] < curve[b.v_];
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
      auto raw_wc =
          raw_way_candidate{static_cast<float>(std::sqrt(squared_dist)), way};
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

match_t lookup::match(profile_parameters const& params,
                      location const& query,
                      bool const reverse,
                      direction const search_dir,
                      double const max_match_distance,
                      bitvec<node_idx_t> const* blocked,
                      search_profile const p,
                      std::optional<std::span<raw_way_candidate const>>
                          raw_way_candidates) const {
  return with_profile(p, [&]<Profile P>(P&&) {
    return match<P>(std::get<typename P::parameters>(params), query, reverse,
                    search_dir, max_match_distance, blocked, std::nullopt,
                    raw_way_candidates);
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
