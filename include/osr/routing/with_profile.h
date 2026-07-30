#pragma once

#include "osr/elevation_storage.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/hgv.h"
#include "osr/routing/profiles/railway.h"
#include "osr/routing/tracking.h"

namespace osr {

template <typename Fn>
auto with_profile(search_profile const p, Fn&& fn) {
  switch (p) {
    case search_profile::kFoot: return fn(foot<false, elevator_tracking>{});
    case search_profile::kWheelchair:
      return fn(foot<true, elevator_tracking>{});
    case search_profile::kBike:
      return fn(bike<bike_costing::kSafe, kElevationNoCost>{});
    case search_profile::kBikeFast:
      return fn(bike<bike_costing::kFast, kElevationNoCost>{});
    case search_profile::kBikeElevationLow:
      return fn(bike<bike_costing::kSafe, kElevationLowCost>{});
    case search_profile::kBikeElevationHigh:
      return fn(bike<bike_costing::kSafe, kElevationHighCost>{});
    case search_profile::kCar: return fn(car{});
    case search_profile::kHgv: return fn(hgv{});
    case search_profile::kCarDropOff: return fn(car_parking<false, false>{});
    case search_profile::kCarDropOffWheelchair:
      return fn(car_parking<true, false>{});
    case search_profile::kCarParking: return fn(car_parking<false, true>{});
    case search_profile::kCarParkingWheelchair:
      return fn(car_parking<true, true>{});
    case search_profile::kBikeSharing: return fn(bike_sharing{});
    case search_profile::kCarSharing:
      return fn(car_sharing<track_node_tracking>{});
    case search_profile::kBus: return fn(bus{});
    case search_profile::kRailway: return fn(railway{});
    case search_profile::kFerry: return fn(ferry{});
  }
  throw utl::fail("with_profile not implemented for {}", to_str(p));
}

template <search_profile P>
struct profile_selector {
  using type = void;
};

template <>
struct profile_selector<search_profile::kFoot> {
  using type = foot<false, elevator_tracking>;
};
template <>
struct profile_selector<search_profile::kWheelchair> {
  using type = foot<true, elevator_tracking>;
};
template <>
struct profile_selector<search_profile::kBike> {
  using type = bike<bike_costing::kSafe, kElevationNoCost>;
};
template <>
struct profile_selector<search_profile::kBikeFast> {
  using type = bike<bike_costing::kFast, kElevationNoCost>;
};
template <>
struct profile_selector<search_profile::kBikeElevationLow> {
  using type = bike<bike_costing::kSafe, kElevationLowCost>;
};
template <>
struct profile_selector<search_profile::kBikeElevationHigh> {
  using type = bike<bike_costing::kSafe, kElevationHighCost>;
};
template <>
struct profile_selector<search_profile::kCar> {
  using type = car;
};
template <>
struct profile_selector<search_profile::kCarDropOff> {
  using type = car_parking<false, false>;
};
template <>
struct profile_selector<search_profile::kCarDropOffWheelchair> {
  using type = car_parking<true, false>;
};
template <>
struct profile_selector<search_profile::kCarParking> {
  using type = car_parking<false, true>;
};
template <>
struct profile_selector<search_profile::kCarParkingWheelchair> {
  using type = car_parking<true, true>;
};
template <>
struct profile_selector<search_profile::kBikeSharing> {
  using type = bike_sharing;
};
template <>
struct profile_selector<search_profile::kCarSharing> {
  using type = car_sharing<track_node_tracking>;
};
template <>
struct profile_selector<search_profile::kBus> {
  using type = bus;
};
template <>
struct profile_selector<search_profile::kRailway> {
  using type = railway;
};
template <>
struct profile_selector<search_profile::kFerry> {
  using type = ferry;
};

}  // namespace osr
