#include "osr/routing/profile.h"

#include "utl/verify.h"

#include "cista/hash.h"

namespace osr {

search_profile to_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("foot"): return search_profile::kFoot;
    case cista::hash("wheelchair"): return search_profile::kWheelchair;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("bike_fast"): return search_profile::kBikeFast;
    case cista::hash("bike_elevation_low"):
      return search_profile::kBikeElevationLow;
    case cista::hash("bike_elevation_high"):
      return search_profile::kBikeElevationHigh;
    case cista::hash("car"): return search_profile::kCar;
    case cista::hash("car_parking"): return search_profile::kCarParking;
    case cista::hash("car_parking_wheelchair"):
      return search_profile::kCarParkingWheelchair;
    case cista::hash("car_dropoff"): return search_profile::kCarDropOff;
    case cista::hash("car_dropoff_wheelchair"):
      return search_profile::kCarDropOffWheelchair;
    case cista::hash("bike_sharing"): return search_profile::kBikeSharing;
    case cista::hash("car_sharing"): return search_profile::kCarSharing;
  }
  throw utl::fail("{} is not a valid profile", s);
}

std::string_view to_str(search_profile const p) {
  switch (p) {
    case search_profile::kFoot: return "foot";
    case search_profile::kWheelchair: return "wheelchair";
    case search_profile::kCar: return "car";
    case search_profile::kBike: return "bike";
    case search_profile::kBikeFast: return "bike_fast";
    case search_profile::kBikeElevationLow: return "bike_elevation_low";
    case search_profile::kBikeElevationHigh: return "bike_elevation_high";
    case search_profile::kCarParking: return "car_parking";
    case search_profile::kCarParkingWheelchair: return "car_parking_wheelchair";
    case search_profile::kCarDropOff: return "car_dropoff";
    case search_profile::kCarDropOffWheelchair: return "car_dropoff_wheelchair";
    case search_profile::kBikeSharing: return "bike_sharing";
    case search_profile::kCarSharing: return "car_sharing";
  }
  throw utl::fail("{} is not a valid profile", static_cast<std::uint8_t>(p));
}

bool is_rental_profile(search_profile const p) {
  return p == search_profile::kBikeSharing || p == search_profile::kCarSharing;
}

}  // namespace osr
