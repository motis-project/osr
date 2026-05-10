#pragma once

#include <cstdint>
#include <algorithm>

#include "boost/math/ccmath/ccmath.hpp"

#include "osr/types.h"

namespace osr {

using quantized_angle_t = std::uint8_t;

constexpr auto const kAngleBins = 256U;
constexpr auto const kAngleHalfCircle = kAngleBins / 2U;

struct turn_bearing {
  bool operator==(turn_bearing const&) const = default;

  constexpr quantized_angle_t get(direction const dir) const noexcept {
    return dir == direction::kForward ? to_next_ : to_prev_;
  }

  quantized_angle_t to_prev_{};
  quantized_angle_t to_next_{};
};

constexpr quantized_angle_t quantize_angle(double const degrees) {
  auto normalized = boost::math::ccmath::fmod(degrees, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }

  return static_cast<quantized_angle_t>(
      static_cast<unsigned>(boost::math::ccmath::lround(
          normalized * static_cast<double>(kAngleBins) / 360.0)) %
      kAngleBins);
}

constexpr quantized_angle_t quantize_turn_angle(double const degrees) {
  return quantize_angle(std::clamp(degrees, 0.0, 180.0));
}

constexpr quantized_angle_t reverse_bearing(quantized_angle_t const bearing) {
  return static_cast<quantized_angle_t>(
      (static_cast<unsigned>(bearing) + kAngleHalfCircle) % kAngleBins);
}

constexpr quantized_angle_t get_turn_angle(turn_bearing const from,
                                           direction const from_dir,
                                           turn_bearing const to,
                                           direction const to_dir) {
  auto const approach = reverse_bearing(from.get(opposite(from_dir)));
  auto const departure = to.get(to_dir);

  auto const diff = static_cast<quantized_angle_t>(
      std::max(approach, departure) - std::min(approach, departure));
  return std::min(diff, static_cast<quantized_angle_t>(kAngleBins - diff));
}

}  // namespace osr
