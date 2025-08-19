#pragma once

#include <cinttypes>
#include <string_view>

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kBikeFast,
  kBikeElevationLow,
  kBikeElevationHigh,
  kCar,
  kCarDropOff,
  kCarDropOffWheelchair,
  kCarParking,
  kCarParkingWheelchair,
  kBikeSharing,
  kCarSharing,
};

constexpr auto const kNumProfiles =
    static_cast<std::underlying_type_t<search_profile>>(10U);

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

bool is_rental_profile(search_profile);

}  // namespace osr
