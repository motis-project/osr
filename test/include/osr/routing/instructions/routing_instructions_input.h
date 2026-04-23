#pragma once

#include <string>
#include <vector>

#include "osr/location.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/instructions/routing_instruction.h"
#include "osr/routing/profile.h"

namespace osr::test {

struct routing_instructions_input {
  std::string test_name;
  std::string osm_file;
  location start;
  location end;
  search_profile profile;
  direction direction;
  routing_algorithm routing_algo;
  std::vector<instruction_action> expected_actions;
};

std::vector<routing_instructions_input> load_from_json(const std::string& path);

}
