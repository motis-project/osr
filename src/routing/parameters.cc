#include "osr/routing/parameters.h"

namespace osr {

profile_parameters get_parameters(search_profile const p) {
  switch (p) {
    case search_profile::kFoot:
      return foot<false, elevator_tracking>::parameters{};
    case search_profile::kWheelchair:
      return foot<true, elevator_tracking>::parameters{};
    case search_profile::kBike:
      return bike<bike_costing::kSafe, kElevationNoCost>::parameters{};
    case search_profile::kBikeFast:
      return bike<bike_costing::kFast, kElevationNoCost>::parameters{};
    case search_profile::kBikeElevationLow:
      return bike<bike_costing::kSafe, kElevationLowCost>::parameters{};
    case search_profile::kBikeElevationHigh:
      return bike<bike_costing::kSafe, kElevationHighCost>::parameters{};
    case search_profile::kCar: return car::parameters{};
    case search_profile::kCarDropOff:
      return car_parking<false, false>::parameters{};
    case search_profile::kCarDropOffWheelchair:
      return car_parking<true, false>::parameters{};
    case search_profile::kCarParking:
      return car_parking<false, true>::parameters{};
    case search_profile::kCarParkingWheelchair:
      return car_parking<true, true>::parameters{};
    case search_profile::kBikeSharing: return bike_sharing::parameters{};
    case search_profile::kCarSharing:
      return car_sharing<track_node_tracking>::parameters{};
  }
  throw utl::fail("with_profile not implemented for {}", to_str(p));
}

}  // namespace osr
