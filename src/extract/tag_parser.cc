#include "osr/extract/tag_parser.h"

#include <cctype>
#include <cmath>
#include <string>

#include "cista/hash.h"

#include "utl/parser/arg_parser.h"

namespace osr {

namespace {

using namespace std::string_view_literals;

struct measure {
  double value_{};
  std::string_view unit_{};
};

struct parsed_double {
  double value_{};
  std::size_t consumed_{};
};

bool is_space(char const c) {
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

std::string to_lower(std::string_view s) {
  auto out = std::string{};
  out.reserve(s.size());
  for (auto const c : s) {
    out.push_back(
        static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
  }
  return out;
}

std::optional<parsed_double> parse_double_prefix(std::string_view s) {
  if (s.empty()) {
    return std::nullopt;
  }
  auto cs = utl::cstr{s};
  auto const* const start = cs.str;
  auto value = 0.0;
  utl::parse_fp(cs, value);
  if (cs.str == start || !std::isfinite(value)) {
    return std::nullopt;
  }
  return parsed_double{value, static_cast<std::size_t>(cs.str - s.data())};
}

std::optional<measure> parse_measure(std::string_view value) {
  value = trim(value);
  auto const result = parse_double_prefix(value);
  if (!result.has_value()) {
    return std::nullopt;
  }
  return measure{result->value_, trim(value.substr(result->consumed_))};
}

}  // namespace

std::string_view trim(std::string_view value) {
  while (!value.empty() && is_space(value.front())) {
    value.remove_prefix(1U);
  }
  while (!value.empty() && is_space(value.back())) {
    value.remove_suffix(1U);
  }
  return value;
}

std::optional<double> parse_double(std::string_view s) {
  s = trim(s);
  auto const result = parse_double_prefix(s);
  return result.has_value() ? std::optional{result->value_} : std::nullopt;
}

std::optional<double> parse_length_m(std::string_view value) {
  auto const m = parse_measure(value);
  if (!m.has_value()) {
    return std::nullopt;
  }

  auto unit = trim(m->unit_);
  if (unit.starts_with('\'')) {
    unit.remove_prefix(1U);
    if (auto const inches = parse_measure(unit);
        inches.has_value() && trim(inches->unit_) == "\""sv) {
      return (m->value_ * 12.0 + inches->value_) * 0.0254;
    }
    if (unit.empty()) {
      return m->value_ * 0.3048;
    }
  }

  auto const lower = to_lower(unit);
  switch (cista::hash(lower)) {
    case cista::hash(""):
    case cista::hash("m"):
    case cista::hash("metre"):
    case cista::hash("metres"):
    case cista::hash("meter"):
    case cista::hash("meters"): return m->value_;
    case cista::hash("km"):
    case cista::hash("kilometre"):
    case cista::hash("kilometres"):
    case cista::hash("kilometer"):
    case cista::hash("kilometers"): return m->value_ * 1000.0;
    case cista::hash("mi"):
    case cista::hash("mile"):
    case cista::hash("miles"): return m->value_ * 1609.344;
    case cista::hash("nmi"): return m->value_ * 1852.0;
    case cista::hash("yd"):
    case cista::hash("yds"): return m->value_ * 0.9144;
    case cista::hash("ft"): return m->value_ * 0.3048;
    case cista::hash("in"):
    case cista::hash("\""): return m->value_ * 0.0254;
    default: return std::nullopt;
  }
}

std::optional<double> parse_weight_t(std::string_view value) {
  auto const m = parse_measure(value);
  if (!m.has_value()) {
    return std::nullopt;
  }
  auto const lower = to_lower(trim(m->unit_));
  switch (cista::hash(lower)) {
    case cista::hash(""):
    case cista::hash("t"): return m->value_;
    case cista::hash("kg"): return m->value_ / 1000.0;
    case cista::hash("st"):
    case cista::hash("ton"):
    case cista::hash("tons"): return m->value_ * 0.9071847;
    case cista::hash("lt"): return m->value_ * 1.016047;
    case cista::hash("lb"):
    case cista::hash("lbs"): return m->value_ * 0.00045359237;
    case cista::hash("cwt"): return m->value_ * 0.0508;
    default: return std::nullopt;
  }
}

std::optional<double> parse_speed_km_h(std::string_view value) {
  auto const m = parse_measure(value);
  if (!m.has_value()) {
    return std::nullopt;
  }
  auto const lower = to_lower(trim(m->unit_));
  switch (cista::hash(lower)) {
    case cista::hash(""):
    case cista::hash("km/h"):
    case cista::hash("kph"):
    case cista::hash("kmh"):
    case cista::hash("kmph"): return m->value_;
    case cista::hash("mph"): return m->value_ * 1.609344;
    case cista::hash("knots"): return m->value_ * 1.852;
    default: return std::nullopt;
  }
}

std::optional<double> parse_unitless(std::string_view value) {
  auto const m = parse_measure(value);
  if (!m.has_value() || !trim(m->unit_).empty()) {
    return std::nullopt;
  }
  return m->value_;
}

}  // namespace osr
