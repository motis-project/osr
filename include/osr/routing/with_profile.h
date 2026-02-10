#pragma once

#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"

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

}  // namespace osr
