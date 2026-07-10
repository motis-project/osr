#pragma once

#include <cstdint>

#include "geo/latlng.h"

namespace osr {

enum class vertical_direction : std::uint8_t {
  kUp,
  kDown
};

enum class relative_direction: std::uint8_t {
  kTurnAround,
  kSharpRight,
  kRight,
  kSlightRight,
  kSharpLeft,
  kLeft,
  kSlightLeft,
  kStraight,
  kInvalid
};

double normalize(double angle);

relative_direction get_relative_direction(double angle);

bool is_rightish(relative_direction direction);

bool is_leftish(relative_direction direction);

bool is_slightish(relative_direction dir);

double get_angle(geo::latlng const& p1,
                 geo::latlng const& shared,
                 geo::latlng const& p2);



} // namespace osr
