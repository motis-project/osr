#include "osr/routing/mode.h"

#include "utl/verify.h"

#include "cista/hash.h"

namespace osr {

std::string_view to_str(mode const m) {
  switch (m) {
    case mode::kFoot: return "foot";
    case mode::kWheelchair: return "wheelchair";
    case mode::kCar: return "car";
    case mode::kBike: return "bike";
    case mode::kRailway: return "railway";
    case mode::kFerry: return "ferry";
  }
  throw utl::fail("{} is not a valid mode", static_cast<std::uint8_t>(m));
}

}  // namespace osr
