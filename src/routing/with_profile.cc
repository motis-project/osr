#include "osr/routing/with_profile.h"

namespace osr {

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