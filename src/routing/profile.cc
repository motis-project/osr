#include "osr/routing/profile.h"

#include "utl/verify.h"

#include "cista/hash.h"

namespace osr {

search_profile to_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("foot"): return search_profile::kFoot;
    case cista::hash("wheelchair"): return search_profile::kWheelchair;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("car"): return search_profile::kCar;
    case cista::hash("car_parking"): return search_profile::kCarParking;
    case cista::hash("car_parking_wheelchair"):
      return search_profile::kCarParkingWheelchair;
    case cista::hash("combi_foot_car_foot_via_parking"):
      return search_profile::kCombiFootCarFootViaParking;
  }
  throw utl::fail("{} is not a valid profile", s);
}

std::string_view to_str(search_profile const p) {
  switch (p) {
    case search_profile::kFoot: return "foot";
    case search_profile::kWheelchair: return "wheelchair";
    case search_profile::kCar: return "car";
    case search_profile::kBike: return "bike";
    case search_profile::kCarParking: return "car_parking";
    case search_profile::kCarParkingWheelchair: return "car_parking_wheelchair";
    case search_profile::kCombiFootCarFootViaParking:
      return "combi_foot_car_foot_via_parking";
  }
  throw utl::fail("{} is not a valid profile", static_cast<std::uint8_t>(p));
}

}  // namespace osr