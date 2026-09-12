#pragma once

#include <cstdint>

#include <string_view>

#include "cista/hash.h"

#include "osr/types.h"

namespace osr {

using conditional_condition_idx_t =
    cista::strong<std::uint32_t, struct conditional_condition_idx_>;
using conditional_condition_set_idx_t =
    cista::strong<std::uint32_t, struct conditional_condition_set_idx_>;
using conditional_opening_hours_idx_t =
    cista::strong<std::uint32_t, struct conditional_opening_hours_idx_>;
using conditional_opening_hours_rule_idx_t =
    cista::strong<std::uint32_t, struct conditional_opening_hours_rule_idx_>;
using conditional_timezone_idx_t =
    cista::strong<std::uint16_t, struct conditional_timezone_idx_>;

struct conditional_range {
  constexpr bool empty() const { return begin_ == end_; }
  constexpr std::uint32_t size() const { return end_ - begin_; }

  std::uint32_t begin_{0U};
  std::uint32_t end_{0U};
};

enum class conditional_restriction_field : std::uint8_t {
  kAccess,
  kOneway,
  kMaxSpeed,
  kMaxLength,
  kMaxWeightRating,
  kMaxHeight,
  kMaxWidth,
  kMaxWeight,
  kMaxAxleLoad,
  kMaxAxles,
  kHazmat,
  kHazmatWater,
  kTrailer,
};

enum class conditional_transport_mode : std::uint8_t {
  kUnspecified,
  kAccess,
  kVehicle,
  kMotorVehicle,
  kMotorcar,
  kHgv,
  kBus,
  kPsv,
  kFoot,
  kBicycle,
};

enum class conditional_osm_direction : std::uint8_t {
  kNone,
  kForward,
  kBackward,
};

constexpr bool is_supported_conditional_restriction_key(std::string_view key) {
  using namespace std::string_view_literals;

  constexpr auto const kSuffix = ":conditional"sv;
  if (!key.ends_with(kSuffix)) {
    return false;
  }
  key.remove_suffix(kSuffix.size());

  if (key.ends_with(":forward"sv)) {
    key.remove_suffix(":forward"sv.size());
  } else if (key.ends_with(":backward"sv)) {
    key.remove_suffix(":backward"sv.size());
  }

  switch (cista::hash(key)) {
    case cista::hash("access"):
    case cista::hash("oneway"):
    case cista::hash("hgv"):
    case cista::hash("motor_vehicle"):
    case cista::hash("maxspeed"):
    case cista::hash("maxspeed:hgv"):
    case cista::hash("maxlength"):
    case cista::hash("maxlength:hgv"):
    case cista::hash("maxweightrating"):
    case cista::hash("maxweightrating:hgv"):
    case cista::hash("maxheight"):
    case cista::hash("maxheight:hgv"):
    case cista::hash("maxwidth"):
    case cista::hash("maxwidth:hgv"):
    case cista::hash("maxweight"):
    case cista::hash("maxweight:hgv"):
    case cista::hash("maxaxleload"):
    case cista::hash("maxaxleload:hgv"):
    case cista::hash("maxaxles"):
    case cista::hash("maxaxles:hgv"):
    case cista::hash("hazmat"):
    case cista::hash("hazmat:water"):
    case cista::hash("hgv:trailer"): return true;
    default: return false;
  }
}

enum class conditional_oneway_value : std::uint8_t {
  kNo,
  kForward,
  kBackward,
};

enum class conditional_numeric_unit : std::uint8_t {
  kUnitless,
  kCentimeter,
  kKilometerPerHour,
  kWeight100Kg,
};

enum class conditional_numeric_state : std::uint8_t {
  kValue,
  kNone,
};

struct conditional_numeric_value {
  std::uint32_t value_{0U};
  conditional_numeric_unit unit_{conditional_numeric_unit::kUnitless};
  conditional_numeric_state state_{conditional_numeric_state::kValue};
};

template <typename Value>
struct conditional_restriction {
  Value value_{};
  conditional_condition_set_idx_t condition_set_{
      conditional_condition_set_idx_t::invalid()};
  conditional_restriction_field field_{};
  conditional_transport_mode mode_{conditional_transport_mode::kUnspecified};
  conditional_osm_direction direction_{conditional_osm_direction::kNone};
};

using conditional_access_restriction = conditional_restriction<access_value>;
using conditional_oneway_restriction =
    conditional_restriction<conditional_oneway_value>;
using conditional_numeric_restriction =
    conditional_restriction<conditional_numeric_value>;

struct conditional_condition_set {
  conditional_range conditions_{};
  conditional_timezone_idx_t timezone_{conditional_timezone_idx_t::invalid()};
};

enum class conditional_condition_type : std::uint8_t {
  kOpeningHours,
  kVehicleProperty,
  kAccessPurpose,
  kVehicleUsage,
};

enum class conditional_comparison : std::uint8_t {
  kNone,
  kEqual,
  kLess,
  kLessEqual,
  kGreater,
  kGreaterEqual,
};

enum class conditional_vehicle_property : std::uint8_t {
  kWeight,
  kWeightRating,
  kLength,
  kWidth,
  kHeight,
  kAxleLoad,
  kAxles,
};

enum class conditional_symbolic_condition : std::uint8_t {
  kDestination,
  kDelivery,
  kPrivate,
  kHazmat,
  kHazmatWater,
  kTrailer,
};

struct conditional_condition {
  conditional_numeric_value value_{};
  std::uint32_t selector_{0U};
  conditional_condition_type type_{};
  conditional_comparison comparison_{conditional_comparison::kNone};
};

enum class opening_hours_rule_modifier : std::uint8_t {
  kOpen,
  kClosed,
  kUnknown,
};

struct opening_hours_rule {
  conditional_range years_{};
  conditional_range weeks_{};
  conditional_range monthdays_{};
  conditional_range weekdays_{};
  conditional_range times_{};
  opening_hours_rule_modifier modifier_{opening_hours_rule_modifier::kOpen};
};

struct opening_hours {
  conditional_range rules_{};
};

struct opening_hours_year_range {
  std::uint16_t from_{0U};
  std::uint16_t to_{0U};
  std::uint16_t step_{1U};
};

struct opening_hours_week_range {
  std::uint8_t from_{1U};
  std::uint8_t to_{53U};
  std::uint8_t step_{1U};
};

struct opening_hours_date {
  std::uint16_t year_{0U};
  std::uint8_t month_{0U};
  std::uint8_t day_{0U};
  std::int8_t nth_weekday_{0};
};

struct opening_hours_monthday_range {
  opening_hours_date from_{};
  opening_hours_date to_{};
};

struct opening_hours_weekday_range {
  std::uint8_t from_{1U};
  std::uint8_t to_{7U};
  std::int8_t nth_from_{0};
  std::int8_t nth_to_{0};
};

struct opening_hours_time_span {
  std::uint16_t from_minutes_{0U};
  std::uint16_t to_minutes_{0U};
};

struct way_conditional_restrictions {
  constexpr bool empty() const {
    return access_.empty() && oneway_.empty() && numeric_.empty();
  }

  conditional_range access_{};
  conditional_range oneway_{};
  conditional_range numeric_{};
};

}  // namespace osr
