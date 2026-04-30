#pragma once

#include <cstdint>
#include <string>

namespace osr {

enum class instruction_action: std::uint8_t {
  kNone,
  kDestination,
  kDestinationLeft,
  kDestinationRight,
  kBecomes,
  kContinue,
  kTurnSlightRight,
  kTurnSlightLeft,
  kTurnRight,
  kTurnLeft,
  kTurnSharpRight,
  kTurnSharpLeft,
  kTurnAround,
  kRampStraight,
  kRampLeft,
  kRampRight,
  kExitRight,
  kExitLeft,
  kStayStraight,
  kStayLeft,
  kStayRight,
  kEnterRoundabout,
  kExitRoundabout,
  kElevator,
  kStairs
};

std::string instruction_action_to_string(instruction_action action);

}