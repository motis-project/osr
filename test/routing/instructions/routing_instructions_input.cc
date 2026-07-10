#include "osr/routing/instructions/routing_instructions_input.h"
#include <algorithm>
#include <filesystem>
#include <fstream>

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

direction direction_from_string(std::string_view const s) {
  if (s == "backward") return direction::kBackward;
  return direction::kForward;
}

routing_algorithm routing_algo_from_string(std::string_view const s) {
  if (s == "bidirectional") return routing_algorithm::kAStarBi;
  return routing_algorithm::kDijkstra;
}

std::vector<routing_instructions_input> load_from_json(
    std::filesystem::path const& test_cases_path,
    std::filesystem::path const& osm_pbf_dir_path) {
  std::ifstream ifs(test_cases_path);
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
        level_t{static_cast<float>(s.at("lvl").as_int64())}};

    auto& e = test_case_obj.at("end").as_object();
    location const end = {static_cast<float>(e.at("lat").as_double()),
                          static_cast<float>(e.at("lng").as_double()),
                          level_t{static_cast<float>(e.at("lvl").as_int64())}};

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
    std::vector<boost::json::value> expected_actions;
    for (auto& act : test_case_obj.at("expected_actions").as_array()) {
      expected_actions.push_back(act);
    }

    cases.emplace_back(
        test_case_obj.at("name").as_string().c_str(),
        osm_pbf_dir_path / test_case_obj.at("osm_file").as_string().c_str(),
        start, end, profile, direction, algo, std::move(expected_actions));
  }
  std::stable_sort(cases.begin(), cases.end(),
                   [](routing_instructions_input const& a,
                      routing_instructions_input const& b) {
                     return a.osm_file < b.osm_file;
                   });
  return cases;
}

}