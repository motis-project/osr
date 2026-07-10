#pragma once

#include <string>
#include <variant>
#include "boost/json.hpp"
#include "osr/routing/instructions/directions.h"
#include "osr/ways.h"

namespace osr {

struct none_instruction {
  auto operator<=>(none_instruction const&) const = default;
};

struct destination_instruction {
  relative_direction direction_{relative_direction::kInvalid};
  auto operator<=>(destination_instruction const&) const = default;
};

struct becomes_instruction {
  auto operator<=>(becomes_instruction const&) const = default;
};

struct continue_instruction {
  auto operator<=>(continue_instruction const&) const = default;
};

struct turn_instruction {
  relative_direction direction_{relative_direction::kInvalid};
  auto operator<=>(turn_instruction const&) const = default;
};

struct ramp_on_instruction {
  relative_direction direction_{relative_direction::kInvalid};
  auto operator<=>(ramp_on_instruction const&) const = default;
};

struct ramp_off_instruction {
  relative_direction direction_{relative_direction::kInvalid};
  auto operator<=>(ramp_off_instruction const&) const = default;
};

struct stay_instruction {
  relative_direction direction_{relative_direction::kInvalid};
  auto operator<=>(stay_instruction const&) const = default;
};

struct enter_roundabout_instruction {
  auto operator<=>(enter_roundabout_instruction const&) const = default;
};

struct exit_roundabout_instruction {
  std::uint16_t exit_number_{0};
  auto operator<=>(exit_roundabout_instruction const&) const = default;
};

struct elevator_instruction {
  vertical_direction direction_{vertical_direction::kUp};
  relative_direction relative_direction_{relative_direction::kInvalid};
  auto operator<=>(elevator_instruction const&) const = default;
};

struct stairs_instruction {
  vertical_direction direction_{vertical_direction::kUp};
  relative_direction relative_direction_{relative_direction::kInvalid};
  auto operator<=>(stairs_instruction const&) const = default;
};

struct crossing_instruction {
  string_idx_t street_{string_idx_t::invalid()};

  bool operator==(crossing_instruction const&) const = default;
  auto operator<=>(crossing_instruction const& o) const {
    return cista::to_idx(street_) <=> cista::to_idx(o.street_);
  }
};

using routing_instruction = std::variant<
    none_instruction,
    destination_instruction,
    becomes_instruction,
    continue_instruction,
    turn_instruction,
    ramp_on_instruction,
    ramp_off_instruction,
    stay_instruction,
    enter_roundabout_instruction,
    exit_roundabout_instruction,
    elevator_instruction,
    stairs_instruction,
    crossing_instruction
>;

boost::json::value to_json(routing_instruction const& instruction, ways const& w);
routing_instruction from_json(boost::json::value const& json, ways const& w);
std::string to_string(routing_instruction const& instruction);

}