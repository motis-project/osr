#include "osr/lookup.h"

#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/combi_profile.h"

namespace osr {

lookup::lookup(ways const& ways) : rtree_{rtree_new()}, ways_{ways} {
  utl::verify(rtree_ != nullptr, "rtree creation failed");
  for (auto i = way_idx_t{0U}; i != ways.n_ways(); ++i) {
    insert(i);
  }
}

lookup::~lookup() { rtree_free(rtree_); }

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
    case search_profile::kCombiFootCarFootViaParking:
      return match<combi_foot_car_foot_profile>(query, reverse, search_dir,
                                      max_match_distance, blocked);
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

void lookup::insert(way_idx_t const way) {
  auto b = geo::box{};
  for (auto const& c : ways_.way_polylines_[way]) {
    b.extend(c);
  }

  auto const min = b.min_.lnglat();
  auto const max = b.max_.lnglat();
  rtree_insert(rtree_, min.data(), max.data(),
               // NOLINTNEXTLINE(performance-no-int-to-ptr)
               reinterpret_cast<void*>(static_cast<std::size_t>(to_idx(way))));
}

}  // namespace osr