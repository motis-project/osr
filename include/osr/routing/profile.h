#pragma once

#include <cinttypes>
#include <string_view>

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kCar,
  kCarParking,
  kCarParkingWheelchair,
  kCombiFootCarFootViaParking,
};

search_profile to_profile(std::string_view);

std::string_view to_str(search_profile);

}  // namespace osr