#pragma once

#include <cstdint>

#include "geo/latlng.h"

namespace osr {

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

relative_direction get_relative_direction(const double angle);

double get_angle(geo::latlng const& p1,
                 geo::latlng const& shared,
                 geo::latlng const& p2);



} // namespace osr
