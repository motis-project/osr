#include "osr/lookup.h"

#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"

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

match_t lookup::match(location const& query,
                      bool const reverse,
                      direction const search_dir,
                      double const max_match_distance,
                      bitvec<node_idx_t> const* blocked,
                      search_profile const p) const {
  switch (p) {
    case search_profile::kFoot:
      return match<foot<false>>(query, reverse, search_dir, max_match_distance,
                                blocked);
    case search_profile::kWheelchair:
      return match<foot<true>>(query, reverse, search_dir, max_match_distance,
                               blocked);
    case search_profile::kCar:
      return match<car>(query, reverse, search_dir, max_match_distance,
                        blocked);
    case search_profile::kBike:
      return match<bike>(query, reverse, search_dir, max_match_distance,
                         blocked);
    case search_profile::kCarParking:
      return match<car_parking<false>>(query, reverse, search_dir,
                                       max_match_distance, blocked);
    case search_profile::kCarParkingWheelchair:
      return match<car_parking<true>>(query, reverse, search_dir,
                                      max_match_distance, blocked);
    case search_profile::kBikeSharing:
      return match<bike_sharing>(query, reverse, search_dir, max_match_distance,
                                 blocked);
  }
  throw utl::fail("{} is not a valid profile", static_cast<std::uint8_t>(p));
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
