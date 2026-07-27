#pragma once

#include <cmath>
#include <cstdint>

#include <algorithm>
#include <limits>
#include <optional>
#include <string_view>

namespace osr {

std::string_view trim(std::string_view);

std::optional<double> parse_double(std::string_view);

std::optional<double> parse_length_m(std::string_view);

std::optional<double> parse_weight_t(std::string_view);

std::optional<double> parse_speed_km_h(std::string_view);

std::optional<double> parse_unitless(std::string_view);

template <typename T>
std::optional<T> to_integer(std::optional<double> const value,
                            double const factor = 1.0) {
  if (!value.has_value() || *value < 0.0) {
    return std::nullopt;
  }
  return static_cast<T>(std::min<double>(std::round(*value * factor),
                                         std::numeric_limits<T>::max()));
}

}  // namespace osr
