#include "osr/routing/instructions/routing_instruction.h"

#include <string>

namespace osr {

std::string instruction_action_to_string(instruction_action const action) {
  switch (action) {
    case instruction_action::kNone: return "kNone";
    case instruction_action::kDestination: return "kDestination";
    case instruction_action::kDestinationLeft: return "kDestinationLeft";
    case instruction_action::kDestinationRight: return "kDestinationRight";
    case instruction_action::kBecomes: return "kBecomes";
    case instruction_action::kContinue: return "kContinue";
    case instruction_action::kTurnSlightRight: return "kTurnSlightRight";
    case instruction_action::kTurnSlightLeft: return "kTurnSlightLeft";
    case instruction_action::kTurnRight: return "kTurnRight";
    case instruction_action::kTurnLeft: return "kTurnLeft";
    case instruction_action::kTurnSharpRight: return "kTurnSharpRight";
    case instruction_action::kTurnSharpLeft: return "kTurnSharpLeft";
    case instruction_action::kTurnAround: return "kTurnAround";
    case instruction_action::kRampStraight: return "kRampStraight";
    case instruction_action::kRampLeft: return "kRampLeft";
    case instruction_action::kRampRight: return "kRampRight";
    case instruction_action::kExitRight: return "kExitRight";
    case instruction_action::kExitLeft: return "kExitLeft";
    case instruction_action::kStayStraight: return "kStayStraight";
    case instruction_action::kStayLeft: return "kStayLeft";
    case instruction_action::kStayRight: return "kStayRight";
    case instruction_action::kEnterRoundabout: return "kEnterRoundabout";
    case instruction_action::kExitRoundabout: return "kExitRoundabout";
    case instruction_action::kElevator: return "kElevator";
    case instruction_action::kStairs: return "kStairs";
  }
}

}
