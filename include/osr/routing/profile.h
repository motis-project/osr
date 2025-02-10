#pragma once

#include <cinttypes>
#include <string_view>

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kBikeElevationLow,
  kBikeElevationHigh,
  kCar,
  kCarParking,
  kCarParkingWheelchair,
  kBikeSharing,
};

constexpr auto const kNumProfiles =
    static_cast<std::underlying_type_t<search_profile>>(7U);

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

}  // namespace osr
