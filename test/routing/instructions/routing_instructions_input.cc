#include "osr/routing/instructions/routing_instructions_input.h"

#include "boost/json/parse.hpp"

namespace osr::test {

search_profile profile_from_string(std::string_view const s) {
  if (s == "foot") return search_profile::kFoot;
  if (s == "car") return search_profile::kCar;
  if (s == "bike") return search_profile::kBike;
  if (s == "wheelchair") return search_profile::kWheelchair;
  if (s == "bike_elevation_low") return search_profile::kBikeElevationLow;
  if (s == "bike_elevation_high") return search_profile::kBikeElevationHigh;
  return search_profile::kFoot;
}

instruction_action action_from_string(std::string_view const s) {
  if (s == "kNone") return instruction_action::kNone;
  if (s == "kDestination") return instruction_action::kDestination;
  if (s == "kDestinationLeft") return instruction_action::kDestinationLeft;
  if (s == "kDestinationRight") return instruction_action::kDestinationRight;
  if (s == "kBecomes") return instruction_action::kBecomes;
  if (s == "kContinue") return instruction_action::kContinue;
  if (s == "kTurnSlightRight") return instruction_action::kTurnSharpRight;
  if (s == "kTurnSlightLeft") return instruction_action::kTurnSlightLeft;
  if (s == "kTurnRight") return instruction_action::kTurnRight;
  if (s == "kTurnLeft") return instruction_action::kTurnLeft;
  if (s == "kTurnSharpRight") return instruction_action::kTurnSharpRight;
  if (s == "kTurnSharpLeft") return instruction_action::kTurnSharpLeft;
  if (s == "kTurnAround") return instruction_action::kTurnAround;
  if (s == "kRampStraight") return instruction_action::kRampStraight;
  if (s == "kRampLeft") return instruction_action::kRampLeft;
  if (s == "kRampRight") return instruction_action::kRampRight;
  if (s == "kExitRight") return instruction_action::kExitRight;
  if (s == "kExitLeft") return instruction_action::kExitLeft;
  if (s == "kStayStraight") return instruction_action::kStayStraight;
  if (s == "kStayLeft") return instruction_action::kStayLeft;
  if (s == "kStayRight") return instruction_action::kStayRight;
  if (s == "kEnterRoundabout") return instruction_action::kEnterRoundabout;
  if (s == "kExitRoundabout") return instruction_action::kExitRoundabout;
  if (s == "kElevator") return instruction_action::kElevator;
  if (s == "kStairs") return instruction_action::kStairs;
  return instruction_action::kNone;
}

direction direction_from_string(std::string_view const s) {
  if (s == "backward") return direction::kBackward;
  return direction::kForward;
}

routing_algorithm routing_algo_from_string(std::string_view const s) {
  if (s == "bidirectional") return routing_algorithm::kAStarBi;
  return routing_algorithm::kDijkstra;
}

std::vector<routing_instructions_input> load_from_json(
    const std::string& path) {
  std::ifstream ifs(path);
  std::string content((std::istreambuf_iterator(ifs)),
                      std::istreambuf_iterator<char>());

  auto value = boost::json::parse(content);
  auto& arr = value.as_array();
  std::vector<routing_instructions_input> cases;

  for (auto& item : arr) {
    auto& test_case_obj = item.as_object();

    // Parse Start/End
    auto& s = test_case_obj.at("start").as_object();
    location const start = {
        static_cast<float>(s.at("lat").as_double()),
        static_cast<float>(s.at("lng").as_double()),
        level_t{static_cast<float>(s.at("lvl").as_double())}};

    auto& e = test_case_obj.at("end").as_object();
    location const end = {static_cast<float>(e.at("lat").as_double()),
                          static_cast<float>(e.at("lng").as_double()),
                          level_t{static_cast<float>(e.at("lvl").as_double())}};

    // Parse Profile
    const auto profile =
        profile_from_string(test_case_obj.at("profile").as_string());

    // Parse Direction
    const auto direction =
        direction_from_string(test_case_obj.at("direction").as_string());

    // Parse routing algorithm
    const auto algo =
        routing_algo_from_string(test_case_obj.at("routing_algo").as_string());

    // Parse Actions Array
    std::vector<instruction_action> expected_actions;
    for (auto& act : test_case_obj.at("expected_actions").as_array()) {
      expected_actions.push_back(action_from_string(act.as_string()));
    }

    cases.emplace_back(test_case_obj.at("name").as_string().c_str(),
                       test_case_obj.at("osm_file").as_string().c_str(), start,
                       end, profile, direction, algo,
                       std::move(expected_actions));
  }
  return cases;
}

}