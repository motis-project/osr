#pragma once

#include <cmath>
#include <cstdlib>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>

namespace osr {

// "8%", "-8 %", "8" (percent), "4.5°" (degrees) -> percent, signed.
// "up" / "down" / "yes" carry no value.
inline std::optional<float> parse_incline_percent(std::string_view s) {
  while (!s.empty() && s.back() == ' ') {
    s.remove_suffix(1U);
  }
  auto is_degrees = false;
  if (s.ends_with("%")) {
    s.remove_suffix(1U);
  } else if (s.ends_with("\xC2\xB0")) {  // degree sign
    s.remove_suffix(2U);
    is_degrees = true;
  }
  auto const str = std::string{s};
  char* end = nullptr;
  auto const v = std::strtof(str.c_str(), &end);
  if (str.empty() || end == str.c_str() ||
      std::string_view{end}.find_first_not_of(' ') != std::string_view::npos ||
      !std::isfinite(v)) {
    return std::nullopt;
  }
  return is_degrees ? static_cast<float>(
                          std::tan(v * std::numbers::pi / 180.0) * 100.0)
                    : v;
}

// "1.2", "1.2 m", "120 cm", "1200 mm" -> meters. Imperial units: no value.
inline std::optional<float> parse_width_meters(std::string_view const s) {
  auto const str = std::string{s};
  char* end = nullptr;
  auto const v = std::strtof(str.c_str(), &end);
  if (str.empty() || end == str.c_str() || !std::isfinite(v) || v <= 0.0F) {
    return std::nullopt;
  }
  auto unit = std::string_view{end};
  while (!unit.empty() && unit.front() == ' ') {
    unit.remove_prefix(1U);
  }
  if (unit.empty() || unit == "m") {
    return v;
  } else if (unit == "cm") {
    return v / 100.0F;
  } else if (unit == "mm") {
    return v / 1000.0F;
  }
  return std::nullopt;
}

}  // namespace osr
