#include "osr/routing/instructions/routing_instruction.h"

#include <string>
#include "utl/overloaded.h"
#include "osr/ways.h"

namespace osr {

std::string direction_to_string(relative_direction const dir) {
  switch (dir) {
    case relative_direction::kTurnAround: return "turn_around";
    case relative_direction::kSharpRight: return "sharp_right";
    case relative_direction::kRight: return "right";
    case relative_direction::kSlightRight: return "slight_right";
    case relative_direction::kSharpLeft: return "sharp_left";
    case relative_direction::kLeft: return "left";
    case relative_direction::kSlightLeft: return "slight_left";
    case relative_direction::kStraight: return "straight";
    case relative_direction::kInvalid: return "invalid";
  }
  return "invalid";
}

relative_direction direction_from_string(std::string_view const s) {
  if (s == "turn_around") return relative_direction::kTurnAround;
  if (s == "sharp_right") return relative_direction::kSharpRight;
  if (s == "right") return relative_direction::kRight;
  if (s == "slight_right") return relative_direction::kSlightRight;
  if (s == "sharp_left") return relative_direction::kSharpLeft;
  if (s == "left") return relative_direction::kLeft;
  if (s == "slight_left") return relative_direction::kSlightLeft;
  if (s == "straight") return relative_direction::kStraight;
  return relative_direction::kInvalid;
}

std::string vertical_direction_to_string(vertical_direction const dir) {
  switch (dir) {
    case vertical_direction::kUp: return "up";
    case vertical_direction::kDown: return "down";
  }
  return "up";
}

vertical_direction vertical_direction_from_string(std::string_view const s) {
  if (s == "down") return vertical_direction::kDown;
  return vertical_direction::kUp;
}

boost::json::value to_json(routing_instruction const& instruction, ways const& w) {
  return std::visit(
      utl::overloaded{
          [](none_instruction const&) -> boost::json::value {
            return {{"type", "none"}};
          },
          [](destination_instruction const& inst) -> boost::json::value {
            return {{"type", "destination"}, {"direction", direction_to_string(inst.direction_)}};
          },
          [](becomes_instruction const&) -> boost::json::value {
            return {{"type", "becomes"}};
          },
          [](continue_instruction const&) -> boost::json::value {
            return {{"type", "continue"}};
          },
          [](turn_instruction const& inst) -> boost::json::value {
            return {{"type", "turn"}, {"direction", direction_to_string(inst.direction_)}};
          },
          [](ramp_on_instruction const& inst) -> boost::json::value {
            return {{"type", "ramp_on"}, {"direction", direction_to_string(inst.direction_)}};
          },
          [](ramp_off_instruction const& inst) -> boost::json::value {
            return {{"type", "ramp_off"}, {"direction", direction_to_string(inst.direction_)}};
          },
          [](stay_instruction const& inst) -> boost::json::value {
            return {{"type", "stay"}, {"direction", direction_to_string(inst.direction_)}};
          },
          [](enter_roundabout_instruction const&) -> boost::json::value {
            return {{"type", "enter_roundabout"}};
          },
          [](exit_roundabout_instruction const& inst) -> boost::json::value {
            return {{"type", "exit_roundabout"}, {"exit", inst.exit_number_}};
          },
          [](elevator_instruction const& inst) -> boost::json::value {
            return {{"type", "elevator"},
                    {"direction", vertical_direction_to_string(inst.direction_)},
                    {"relative_direction", direction_to_string(inst.relative_direction_)}};
          },
          [](stairs_instruction const& inst) -> boost::json::value {
            return {{"type", "stairs"},
                    {"direction", vertical_direction_to_string(inst.direction_)},
                    {"relative_direction", direction_to_string(inst.relative_direction_)}};
          },
          [&w](crossing_instruction const& inst) -> boost::json::value {
            auto const street_name = inst.street_ == string_idx_t::invalid()
                                         ? ""
                                         : w.strings_[inst.street_].view();
            return {{"type", "crossing"},
                    {"street", street_name}};
          }},
      instruction);
}

routing_instruction from_json(boost::json::value const& json, ways const& w) {
  if (!json.is_object()) {
    return none_instruction{};
  }
  auto const& obj = json.as_object();
  if (!obj.contains("type")) {
    return none_instruction{};
  }
  auto const type_val = obj.at("type").as_string();
  if (type_val == "none") {
    return none_instruction{};
  } else if (type_val == "destination") {
    relative_direction direction = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = direction_from_string(obj.at("direction").as_string());
    }
    return destination_instruction{direction};
  } else if (type_val == "becomes") {
    return becomes_instruction{};
  } else if (type_val == "continue") {
    return continue_instruction{};
  } else if (type_val == "turn") {
    relative_direction direction = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = direction_from_string(obj.at("direction").as_string());
    }
    return turn_instruction{direction};
  } else if (type_val == "ramp_on") {
    relative_direction direction = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = direction_from_string(obj.at("direction").as_string());
    }
    return ramp_on_instruction{direction};
  } else if (type_val == "ramp_off") {
    relative_direction direction = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = direction_from_string(obj.at("direction").as_string());
    }
    return ramp_off_instruction{direction};
  } else if (type_val == "stay") {
    relative_direction direction = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = direction_from_string(obj.at("direction").as_string());
    }
    return stay_instruction{direction};
  } else if (type_val == "enter_roundabout") {
    return enter_roundabout_instruction{};
  } else if (type_val == "exit_roundabout") {
    std::uint16_t exit = 0;
    if (obj.contains("exit")) {
      exit = static_cast<std::uint16_t>(obj.at("exit").as_int64());
    }
    return exit_roundabout_instruction{exit};
  } else if (type_val == "elevator") {
    vertical_direction direction = vertical_direction::kUp;
    relative_direction rel_dir = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = vertical_direction_from_string(obj.at("direction").as_string());
    }
    if (obj.contains("relative_direction")) {
      rel_dir = direction_from_string(obj.at("relative_direction").as_string());
    }
    return elevator_instruction{direction, rel_dir};
  } else if (type_val == "stairs") {
    vertical_direction direction = vertical_direction::kUp;
    relative_direction rel_dir = relative_direction::kInvalid;
    if (obj.contains("direction")) {
      direction = vertical_direction_from_string(obj.at("direction").as_string());
    }
    if (obj.contains("relative_direction")) {
      rel_dir = direction_from_string(obj.at("relative_direction").as_string());
    }
    return stairs_instruction{direction, rel_dir};
  } else if (type_val == "crossing") {
    string_idx_t street = string_idx_t::invalid();
    if (obj.contains("street")) {
      auto const name = obj.at("street").as_string();
      const auto opt_idx = w.find_string(name.c_str());
      if (opt_idx.has_value()) {
        street = opt_idx.value();
      }
    }
    return crossing_instruction{street};
  }
  return none_instruction{};
}

std::string to_string(routing_instruction const& instruction) {
  return std::visit(
      utl::overloaded{
          [](none_instruction const&) -> std::string { return "kNone"; },
          [](destination_instruction const& inst) -> std::string {
            if (inst.direction_ == relative_direction::kLeft) return "kDestinationLeft";
            if (inst.direction_ == relative_direction::kRight) return "kDestinationRight";
            return "kDestination";
          },
          [](becomes_instruction const&) -> std::string { return "kBecomes"; },
          [](continue_instruction const&) -> std::string { return "kContinue"; },
          [](turn_instruction const& inst) -> std::string {
            switch (inst.direction_) {
              case relative_direction::kSlightRight: return "kTurnSlightRight";
              case relative_direction::kRight: return "kTurnRight";
              case relative_direction::kSharpRight: return "kTurnSharpRight";
              case relative_direction::kTurnAround: return "kTurnAround";
              case relative_direction::kSharpLeft: return "kTurnSharpLeft";
              case relative_direction::kLeft: return "kTurnLeft";
              case relative_direction::kSlightLeft: return "kTurnSlightLeft";
              default: return "kContinue";
            }
          },
          [](ramp_on_instruction const& inst) -> std::string {
            switch (inst.direction_) {
              case relative_direction::kLeft: return "kRampLeft";
              case relative_direction::kRight: return "kRampRight";
              default: return "kRampStraight";
            }
          },
          [](ramp_off_instruction const& inst) -> std::string {
            switch (inst.direction_) {
              case relative_direction::kLeft: return "kExitLeft";
              case relative_direction::kRight: return "kExitRight";
              default: return "kExitRight";
            }
          },
          [](stay_instruction const& inst) -> std::string {
            switch (inst.direction_) {
              case relative_direction::kStraight: return "kStayStraight";
              case relative_direction::kSlightLeft: return "kStaySlightLeft";
              case relative_direction::kLeft: return "kStayLeft";
              case relative_direction::kSlightRight: return "kStaySlightRight";
              case relative_direction::kRight: return "kStayRight";
              default: return "kStayStraight";
            }
          },
          [](enter_roundabout_instruction const&) -> std::string { return "kEnterRoundabout"; },
          [](exit_roundabout_instruction const&) -> std::string { return "kExitRoundabout"; },
          [](elevator_instruction const&) -> std::string { return "kElevator"; },
          [](stairs_instruction const&) -> std::string { return "kStairs"; },
          [](crossing_instruction const&) -> std::string { return "kCrossing"; }},
      instruction);
}

}

