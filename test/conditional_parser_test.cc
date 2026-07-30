#include "osr/extract/conditional_parser.h"

#include <cstdint>

#include <optional>
#include <string_view>

#include "gtest/gtest.h"

#include "osr/geojson.h"

using namespace std::string_view_literals;

namespace {

struct parser_fixture {
  parser_fixture() : builder_{.routing_ = routing_} {}

  bool parse(std::string_view const key, std::string_view const value) {
    return osr::parse_conditional_restriction_tag(key, value, builder_);
  }

  std::optional<osr::conditional_condition_set_idx_t> parse_condition_set(
      std::string_view const condition) {
    return osr::parse_conditional_condition_set(condition, builder_);
  }

  osr::conditional_condition const& condition(std::uint32_t const idx) const {
    return routing_.conditional_conditions_[idx];
  }

  osr::opening_hours_rule const& opening_hours_rule(
      std::uint32_t const idx) const {
    return routing_.opening_hours_rules_[idx];
  }

  osr::conditional_condition_set const& condition_set(
      std::uint32_t const idx) const {
    return routing_.conditional_condition_sets_[idx];
  }

  osr::conditional_numeric_value numeric_value(std::uint32_t const idx) const {
    return routing_.conditional_numeric_[idx].value_;
  }

  osr::ways::routing routing_{};
  osr::conditional_storage_builder builder_;
};

}  // namespace

TEST(conditional_parser, parses_hgv_access_with_weekday_time_range) {
  auto f = parser_fixture{};

  ASSERT_TRUE(
      f.parse("hgv:conditional"sv, "destination @ (Mo-Fr 06:00-10:00)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_access_.size());
  auto const& restriction = f.routing_.conditional_access_.front();
  EXPECT_EQ(osr::access_value::kDestination, restriction.value_);
  EXPECT_EQ(osr::conditional_transport_mode::kHgv, restriction.mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess, restriction.field_);

  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());

  ASSERT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);
  auto const& rule = f.opening_hours_rule(0U);
  ASSERT_EQ(1U, rule.weekdays_.size());
  ASSERT_EQ(1U, rule.times_.size());
  EXPECT_EQ(1U, f.routing_.opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(5U, f.routing_.opening_hours_weekday_ranges_[0].to_);
  EXPECT_EQ(6U * 60U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(10U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
}

TEST(conditional_parser, parses_oneway_conditional_values) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("oneway:conditional"sv,
                      "yes @ (Mo-Fr 07:00-16:00); no @ (delivery); "
                      "-1 @ (2026 May 26-2026 Dec 31)"sv));

  ASSERT_EQ(3U, f.routing_.conditional_oneway_.size());
  EXPECT_EQ(osr::conditional_oneway_value::kForward,
            f.routing_.conditional_oneway_[0].value_);
  EXPECT_EQ(osr::conditional_oneway_value::kNo,
            f.routing_.conditional_oneway_[1].value_);
  EXPECT_EQ(osr::conditional_oneway_value::kBackward,
            f.routing_.conditional_oneway_[2].value_);
  EXPECT_EQ(osr::conditional_restriction_field::kOneway,
            f.routing_.conditional_oneway_[2].field_);
}

TEST(conditional_parser, semicolons_in_opening_hours) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("access:conditional"sv,
                      "no @ (Mo-Fr 07:00-09:00; Sa 10:00-12:00); "
                      "delivery @ (trailer)"sv));

  ASSERT_EQ(2U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::access_value::kNo, f.routing_.conditional_access_[0].value_);
  EXPECT_EQ(osr::access_value::kDelivery,
            f.routing_.conditional_access_[1].value_);
  ASSERT_EQ(2U, f.routing_.opening_hours_rules_.size());

  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);

  ASSERT_EQ(1U, f.routing_.opening_hours_.size());
  EXPECT_EQ(0U, f.routing_.opening_hours_[0].rules_.begin_);
  EXPECT_EQ(2U, f.routing_.opening_hours_[0].rules_.end_);

  auto const& rule0 = f.opening_hours_rule(0U);
  ASSERT_EQ(1U, rule0.weekdays_.size());
  ASSERT_EQ(1U, rule0.times_.size());

  auto const& rule1 = f.opening_hours_rule(1U);
  ASSERT_EQ(1U, rule1.weekdays_.size());
  ASSERT_EQ(1U, rule1.times_.size());

  ASSERT_EQ(2U, f.routing_.opening_hours_weekday_ranges_.size());
  EXPECT_EQ(1U, f.routing_.opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(5U, f.routing_.opening_hours_weekday_ranges_[0].to_);
  EXPECT_EQ(6U, f.routing_.opening_hours_weekday_ranges_[1].from_);
  EXPECT_EQ(6U, f.routing_.opening_hours_weekday_ranges_[1].to_);

  ASSERT_EQ(2U, f.routing_.opening_hours_time_spans_.size());
  EXPECT_EQ(7U * 60U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(9U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
  EXPECT_EQ(10U * 60U, f.routing_.opening_hours_time_spans_[1].from_minutes_);
  EXPECT_EQ(12U * 60U, f.routing_.opening_hours_time_spans_[1].to_minutes_);

  auto const& trailer = f.condition(1U);
  EXPECT_EQ(osr::conditional_condition_type::kVehicleUsage, trailer.type_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_symbolic_condition::kTrailer),
      trailer.selector_);
}

TEST(conditional_parser, parses_numeric_hgv_and_vehicle_condition) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("maxspeed:hgv:forward:conditional"sv,
                      "50 mph @ (weight > 7.5 AND Dec-Mar)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
  auto const& restriction = f.routing_.conditional_numeric_.front();
  EXPECT_EQ(osr::conditional_restriction_field::kMaxSpeed, restriction.field_);
  EXPECT_EQ(osr::conditional_transport_mode::kHgv, restriction.mode_);
  EXPECT_EQ(osr::conditional_osm_direction::kForward, restriction.direction_);
  EXPECT_EQ(osr::conditional_numeric_unit::kKilometerPerHour,
            restriction.value_.unit_);
  EXPECT_EQ(80U, restriction.value_.value_);

  ASSERT_EQ(2U, f.routing_.conditional_conditions_.size());
  auto const& weight = f.condition(0U);
  EXPECT_EQ(osr::conditional_condition_type::kVehicleProperty, weight.type_);
  EXPECT_EQ(osr::conditional_comparison::kGreater, weight.comparison_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_vehicle_property::kWeight),
      weight.selector_);
  EXPECT_EQ(75U, weight.value_.value_);
  EXPECT_EQ(osr::conditional_numeric_unit::kWeight100Kg, weight.value_.unit_);

  auto const& month_range = f.routing_.opening_hours_monthday_ranges_[0];
  EXPECT_EQ(12U, month_range.from_.month_);
  EXPECT_EQ(3U, month_range.to_.month_);
}

TEST(conditional_parser, opening_hours_calendar_selectors) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("access:conditional"sv,
                      "no @ (2024-2026 May 01-Oct 31 week 01-53/2 "
                      "Su[1] 08:00-12:00,13:00-17:00)"sv));

  ASSERT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);

  ASSERT_EQ(1U, f.routing_.opening_hours_.size());
  ASSERT_EQ(1U, f.routing_.opening_hours_[0].rules_.size());
  auto const& rule = f.opening_hours_rule(0U);

  ASSERT_EQ(1U, f.routing_.opening_hours_year_ranges_.size());
  EXPECT_EQ(2024U, f.routing_.opening_hours_year_ranges_[0].from_);
  EXPECT_EQ(2026U, f.routing_.opening_hours_year_ranges_[0].to_);
  ASSERT_EQ(1U, rule.years_.size());
  ASSERT_EQ(0U, rule.years_.begin_);

  ASSERT_EQ(1U, f.routing_.opening_hours_week_ranges_.size());
  EXPECT_EQ(1U, f.routing_.opening_hours_week_ranges_[0].from_);
  EXPECT_EQ(53U, f.routing_.opening_hours_week_ranges_[0].to_);
  EXPECT_EQ(2U, f.routing_.opening_hours_week_ranges_[0].step_);
  ASSERT_EQ(1U, rule.weeks_.size());
  ASSERT_EQ(0U, rule.weeks_.begin_);

  ASSERT_EQ(1U, f.routing_.opening_hours_monthday_ranges_.size());
  EXPECT_EQ(5U, f.routing_.opening_hours_monthday_ranges_[0].from_.month_);
  EXPECT_EQ(1U, f.routing_.opening_hours_monthday_ranges_[0].from_.day_);
  EXPECT_EQ(10U, f.routing_.opening_hours_monthday_ranges_[0].to_.month_);
  EXPECT_EQ(31U, f.routing_.opening_hours_monthday_ranges_[0].to_.day_);
  ASSERT_EQ(1U, rule.monthdays_.size());
  ASSERT_EQ(0U, rule.monthdays_.begin_);

  ASSERT_EQ(1U, f.routing_.opening_hours_weekday_ranges_.size());
  EXPECT_EQ(7U, f.routing_.opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(1, f.routing_.opening_hours_weekday_ranges_[0].nth_from_);
  ASSERT_EQ(1U, rule.weekdays_.size());
  ASSERT_EQ(0U, rule.weekdays_.begin_);

  ASSERT_EQ(2U, f.routing_.opening_hours_time_spans_.size());
  EXPECT_EQ(8U * 60U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(12U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
  EXPECT_EQ(13U * 60U, f.routing_.opening_hours_time_spans_[1].from_minutes_);
  EXPECT_EQ(17U * 60U, f.routing_.opening_hours_time_spans_[1].to_minutes_);
  ASSERT_EQ(2U, rule.times_.size());
  ASSERT_EQ(0U, rule.times_.begin_);

  EXPECT_EQ(
      "2024-2026 week 1-53/2 May 01-Oct 31 Su[1] "
      "08:00-12:00,13:00-17:00",
      osr::opening_hours_to_string(f.routing_, f.condition(0U).selector_));
}

TEST(conditional_parser, supports_24_7_and_open_ended_time) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (24/7)"sv));
  ASSERT_TRUE(f.parse("access:conditional"sv, "no @ (Mo 18:00+)"sv));

  ASSERT_EQ(2U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::access_value::kNo, f.routing_.conditional_access_[0].value_);
  EXPECT_EQ(osr::conditional_transport_mode::kHgv,
            f.routing_.conditional_access_[0].mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess,
            f.routing_.conditional_access_[0].field_);
  EXPECT_EQ(osr::access_value::kNo, f.routing_.conditional_access_[1].value_);
  EXPECT_EQ(osr::conditional_transport_mode::kAccess,
            f.routing_.conditional_access_[1].mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess,
            f.routing_.conditional_access_[1].field_);

  ASSERT_EQ(2U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);
  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(1U).type_);

  ASSERT_EQ(2U, f.routing_.opening_hours_.size());
  ASSERT_EQ(2U, f.routing_.opening_hours_rules_.size());

  auto const& rule0 = f.opening_hours_rule(0U);
  ASSERT_EQ(0U, rule0.weekdays_.size());
  ASSERT_EQ(1U, rule0.times_.size());

  auto const& rule1 = f.opening_hours_rule(1U);
  ASSERT_EQ(1U, rule1.weekdays_.size());
  ASSERT_EQ(1U, rule1.times_.size());

  ASSERT_EQ(1U, f.routing_.opening_hours_weekday_ranges_.size());
  EXPECT_EQ(1U, f.routing_.opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(1U, f.routing_.opening_hours_weekday_ranges_[0].to_);

  ASSERT_EQ(2U, f.routing_.opening_hours_time_spans_.size());
  EXPECT_EQ(0U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(24U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
  EXPECT_EQ(18U * 60U, f.routing_.opening_hours_time_spans_[1].from_minutes_);
  EXPECT_EQ(24U * 60U, f.routing_.opening_hours_time_spans_[1].to_minutes_);

  EXPECT_EQ("Mo 18:00-24:00",
            osr::opening_hours_to_string(
                f.routing_, f.routing_.conditional_conditions_[1].selector_));
}

TEST(conditional_parser, supports_unparenthesized_conditions) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("motor_vehicle:conditional"sv, "no @ 18:00-06:00"sv));
  ASSERT_TRUE(
      f.parse("maxweightrating:hgv:conditional"sv, "none @ destination"sv));

  ASSERT_EQ(1U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::access_value::kNo, f.routing_.conditional_access_[0].value_);
  EXPECT_EQ(osr::conditional_transport_mode::kMotorVehicle,
            f.routing_.conditional_access_[0].mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess,
            f.routing_.conditional_access_[0].field_);

  ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
  EXPECT_EQ(osr::conditional_numeric_state::kNone,
            f.routing_.conditional_numeric_[0].value_.state_);
  EXPECT_EQ(osr::conditional_transport_mode::kHgv,
            f.routing_.conditional_numeric_[0].mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kMaxWeightRating,
            f.routing_.conditional_numeric_[0].field_);

  ASSERT_EQ(2U, f.routing_.conditional_condition_sets_.size());

  ASSERT_EQ(2U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);
  EXPECT_EQ(osr::conditional_condition_type::kAccessPurpose,
            f.condition(1U).type_);
  EXPECT_EQ(static_cast<std::uint32_t>(
                osr::conditional_symbolic_condition::kDestination),
            f.condition(1U).selector_);

  ASSERT_EQ(1U, f.routing_.opening_hours_.size());
  ASSERT_EQ(1U, f.routing_.opening_hours_rules_.size());

  auto const& rule = f.opening_hours_rule(0U);
  ASSERT_EQ(0U, rule.weekdays_.size());
  ASSERT_EQ(1U, rule.times_.size());

  ASSERT_EQ(1U, f.routing_.opening_hours_time_spans_.size());
  EXPECT_EQ(18U * 60U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(6U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
}

TEST(conditional_parser, parses_access_purpose_conditions) {
  auto const expect_access_purpose =
      [](std::string_view const text,
         osr::conditional_symbolic_condition const selector) {
        auto f = parser_fixture{};
        auto const idx = f.parse_condition_set(text);
        ASSERT_TRUE(idx.has_value());
        ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
        ASSERT_EQ(1U, f.routing_.conditional_conditions_.size());
        EXPECT_EQ(osr::conditional_condition_type::kAccessPurpose,
                  f.condition(0U).type_);
        EXPECT_EQ(static_cast<std::uint32_t>(selector),
                  f.condition(0U).selector_);
      };

  expect_access_purpose("destination"sv,
                        osr::conditional_symbolic_condition::kDestination);
  expect_access_purpose("delivery"sv,
                        osr::conditional_symbolic_condition::kDelivery);
  expect_access_purpose("private"sv,
                        osr::conditional_symbolic_condition::kPrivate);
}

TEST(conditional_parser, parses_supported_access_values) {
  struct test_case {
    std::string_view value_;
    osr::access_value expected_;
  };

  for (auto const& tc :
       {test_case{"yes"sv, osr::access_value::kYes},
        test_case{"designated"sv, osr::access_value::kDesignated},
        test_case{"permissive"sv, osr::access_value::kPermissive},
        test_case{"private"sv, osr::access_value::kPrivate},
        test_case{"no"sv, osr::access_value::kNo},
        test_case{"destination"sv, osr::access_value::kDestination},
        test_case{"delivery"sv, osr::access_value::kDelivery},
        test_case{"discouraged"sv, osr::access_value::kDiscouraged}}) {
    auto f = parser_fixture{};

    ASSERT_TRUE(f.parse("access:conditional"sv,
                        std::string{tc.value_} + " @ (Mo 08:00-09:00)"));

    ASSERT_EQ(1U, f.routing_.conditional_access_.size());
    EXPECT_EQ(tc.expected_, f.routing_.conditional_access_.front().value_);
    EXPECT_EQ(osr::conditional_transport_mode::kAccess,
              f.routing_.conditional_access_.front().mode_);
    EXPECT_EQ(osr::conditional_restriction_field::kAccess,
              f.routing_.conditional_access_.front().field_);
  }
}

TEST(conditional_parser, parses_supported_numeric_restrictions_and_units) {
  struct test_case {
    std::string_view key_;
    std::string_view value_;
    osr::conditional_restriction_field field_;
    std::uint32_t parsed_value_;
    osr::conditional_numeric_unit unit_;
  };

  for (auto const& tc : {
           test_case{"maxspeed:conditional"sv, "10 knots"sv,
                     osr::conditional_restriction_field::kMaxSpeed, 19U,
                     osr::conditional_numeric_unit::kKilometerPerHour},
           test_case{"maxlength:hgv:conditional"sv, "2 mi"sv,
                     osr::conditional_restriction_field::kMaxLength, 321869U,
                     osr::conditional_numeric_unit::kCentimeter},
           test_case{"maxlength:hgv:conditional"sv, "2 nmi"sv,
                     osr::conditional_restriction_field::kMaxLength, 370400U,
                     osr::conditional_numeric_unit::kCentimeter},
           test_case{"maxheight:hgv:backward:conditional"sv, "12'6\""sv,
                     osr::conditional_restriction_field::kMaxHeight, 381U,
                     osr::conditional_numeric_unit::kCentimeter},
           test_case{"maxwidth:conditional"sv, "8 ft"sv,
                     osr::conditional_restriction_field::kMaxWidth, 244U,
                     osr::conditional_numeric_unit::kCentimeter},
           test_case{"maxweight:hgv:conditional"sv, "10000 kg"sv,
                     osr::conditional_restriction_field::kMaxWeight, 100U,
                     osr::conditional_numeric_unit::kWeight100Kg},
           test_case{"maxweight:hgv:conditional"sv, "10 cwt"sv,
                     osr::conditional_restriction_field::kMaxWeight, 5U,
                     osr::conditional_numeric_unit::kWeight100Kg},
           test_case{"maxaxleload:conditional"sv, "20000 lb"sv,
                     osr::conditional_restriction_field::kMaxAxleLoad, 91U,
                     osr::conditional_numeric_unit::kWeight100Kg},
           test_case{"maxaxles:hgv:conditional"sv, "3"sv,
                     osr::conditional_restriction_field::kMaxAxles, 3U,
                     osr::conditional_numeric_unit::kUnitless},
       }) {
    auto f = parser_fixture{};

    ASSERT_TRUE(f.parse(tc.key_, std::string{tc.value_} + " @ (Mo)"));

    ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
    auto const& restriction = f.routing_.conditional_numeric_.front();
    EXPECT_EQ(tc.field_, restriction.field_);
    EXPECT_EQ(tc.parsed_value_, restriction.value_.value_);
    EXPECT_EQ(tc.unit_, restriction.value_.unit_);
  }
}

TEST(conditional_parser, parses_numeric_none_aliases) {
  for (std::string_view const value : {"none"sv, "no"sv, "unknown"sv}) {
    auto f = parser_fixture{};

    ASSERT_TRUE(f.parse("maxweightrating:hgv:conditional"sv,
                        std::string{value} + " @ (destination)"));

    ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
    EXPECT_EQ(osr::conditional_numeric_state::kNone,
              f.numeric_value(0U).state_);
    EXPECT_EQ(osr::conditional_numeric_unit::kUnitless,
              f.numeric_value(0U).unit_);
  }
}

TEST(conditional_parser, parses_hazmat_and_trailer_keys) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("hazmat:conditional"sv, "no @ (Mo 08:00-09:00)"sv));
  ASSERT_TRUE(f.parse("hazmat:water:conditional"sv, "no @ (Tu 08:00-09:00)"sv));
  ASSERT_TRUE(f.parse("hgv:trailer:conditional"sv, "no @ (We 08:00-09:00)"sv));

  ASSERT_EQ(3U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::conditional_restriction_field::kHazmat,
            f.routing_.conditional_access_[0].field_);
  EXPECT_EQ(osr::conditional_restriction_field::kHazmatWater,
            f.routing_.conditional_access_[1].field_);
  EXPECT_EQ(osr::conditional_restriction_field::kTrailer,
            f.routing_.conditional_access_[2].field_);
  for (auto const& restriction : f.routing_.conditional_access_) {
    EXPECT_EQ(osr::conditional_transport_mode::kHgv, restriction.mode_);
    EXPECT_EQ(osr::access_value::kNo, restriction.value_);
  }
}

TEST(conditional_parser, parses_vehicle_usage_conditions) {
  auto f = parser_fixture{};

  auto const idx = f.parse_condition_set("hazmat AND trailer"sv);

  ASSERT_TRUE(idx.has_value());
  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(2U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(0U, f.condition_set(0U).conditions_.begin_);
  EXPECT_EQ(2U, f.condition_set(0U).conditions_.end_);

  EXPECT_EQ(osr::conditional_condition_type::kVehicleUsage,
            f.condition(0U).type_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_symbolic_condition::kHazmat),
      f.condition(0U).selector_);
  EXPECT_EQ(osr::conditional_condition_type::kVehicleUsage,
            f.condition(1U).type_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_symbolic_condition::kTrailer),
      f.condition(1U).selector_);
}

TEST(conditional_parser, parses_vehicle_property_comparisons_and_units) {
  struct expected_condition {
    std::string_view text_;
    osr::conditional_vehicle_property property_;
    osr::conditional_comparison comparison_;
    std::uint32_t value_;
    osr::conditional_numeric_unit unit_;
  };

  auto const expected = {
      expected_condition{"weight_rating <= 900 kg"sv,
                         osr::conditional_vehicle_property::kWeightRating,
                         osr::conditional_comparison::kLessEqual, 9U,
                         osr::conditional_numeric_unit::kWeight100Kg},
      expected_condition{"axle_load >= 8 st"sv,
                         osr::conditional_vehicle_property::kAxleLoad,
                         osr::conditional_comparison::kGreaterEqual, 73U,
                         osr::conditional_numeric_unit::kWeight100Kg},
      expected_condition{"height = 3.8 m"sv,
                         osr::conditional_vehicle_property::kHeight,
                         osr::conditional_comparison::kEqual, 380U,
                         osr::conditional_numeric_unit::kCentimeter},
      expected_condition{"width < 100 in"sv,
                         osr::conditional_vehicle_property::kWidth,
                         osr::conditional_comparison::kLess, 254U,
                         osr::conditional_numeric_unit::kCentimeter},
      expected_condition{"axles == 5"sv,
                         osr::conditional_vehicle_property::kAxles,
                         osr::conditional_comparison::kEqual, 5U,
                         osr::conditional_numeric_unit::kUnitless}};

  for (auto const& e : expected) {
    auto f = parser_fixture{};
    auto const idx = f.parse_condition_set(e.text_);

    ASSERT_TRUE(idx.has_value()) << e.text_;
    ASSERT_EQ(1U, f.routing_.conditional_conditions_.size()) << e.text_;
    auto const& c = f.condition(0U);
    EXPECT_EQ(osr::conditional_condition_type::kVehicleProperty, c.type_);
    EXPECT_EQ(static_cast<std::uint32_t>(e.property_), c.selector_);
    EXPECT_EQ(e.comparison_, c.comparison_);
    EXPECT_EQ(e.value_, c.value_.value_);
    EXPECT_EQ(e.unit_, c.value_.unit_);
  }
}

TEST(conditional_parser, parses_condition_sets_with_and_separators) {
  auto f = parser_fixture{};

  auto const idx =
      f.parse_condition_set("weight > 7.5 and length < 10 && axles == 5"sv);

  ASSERT_TRUE(idx.has_value());
  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(3U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(0U, f.condition_set(0U).conditions_.begin_);
  EXPECT_EQ(3U, f.condition_set(0U).conditions_.end_);
  EXPECT_EQ(osr::conditional_vehicle_property::kWeight,
            static_cast<osr::conditional_vehicle_property>(
                f.condition(0U).selector_));
  EXPECT_EQ(osr::conditional_vehicle_property::kLength,
            static_cast<osr::conditional_vehicle_property>(
                f.condition(1U).selector_));
  EXPECT_EQ(osr::conditional_vehicle_property::kAxles,
            static_cast<osr::conditional_vehicle_property>(
                f.condition(2U).selector_));
}

TEST(conditional_parser, parses_opening_hours_modifiers_and_multiple_rules) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("access:conditional"sv,
                      "no @ (Mo-Fr 09:00-17:00 open; "
                      "Sa 10:00-14:00 closed; Su unknown)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::access_value::kNo, f.routing_.conditional_access_[0].value_);
  EXPECT_EQ(osr::conditional_transport_mode::kAccess,
            f.routing_.conditional_access_[0].mode_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess,
            f.routing_.conditional_access_[0].field_);

  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(1U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.condition(0U).type_);

  ASSERT_EQ(1U, f.routing_.opening_hours_.size());
  EXPECT_EQ(0U, f.routing_.opening_hours_[0].rules_.begin_);
  EXPECT_EQ(3U, f.routing_.opening_hours_[0].rules_.end_);
  ASSERT_EQ(3U, f.routing_.opening_hours_rules_.size());

  auto const& rule0 = f.opening_hours_rule(0U);
  EXPECT_EQ(osr::opening_hours_rule_modifier::kOpen, rule0.modifier_);
  ASSERT_EQ(1U, rule0.weekdays_.size());
  ASSERT_EQ(0U, rule0.weekdays_.begin_);
  ASSERT_EQ(1U, rule0.times_.size());
  ASSERT_EQ(0U, rule0.times_.begin_);

  auto const& rule1 = f.opening_hours_rule(1U);
  EXPECT_EQ(osr::opening_hours_rule_modifier::kClosed, rule1.modifier_);
  ASSERT_EQ(1U, rule1.weekdays_.size());
  ASSERT_EQ(1U, rule1.weekdays_.begin_);
  ASSERT_EQ(1U, rule1.times_.size());
  ASSERT_EQ(1U, rule1.times_.begin_);

  auto const& rule2 = f.opening_hours_rule(2U);
  EXPECT_EQ(osr::opening_hours_rule_modifier::kUnknown, rule2.modifier_);
  ASSERT_EQ(1U, rule2.weekdays_.size());
  ASSERT_EQ(2U, rule2.weekdays_.begin_);
  ASSERT_EQ(0U, rule2.times_.size());

  ASSERT_EQ(3U, f.routing_.opening_hours_weekday_ranges_.size());
  EXPECT_EQ(1U, f.routing_.opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(5U, f.routing_.opening_hours_weekday_ranges_[0].to_);
  EXPECT_EQ(6U, f.routing_.opening_hours_weekday_ranges_[1].from_);
  EXPECT_EQ(6U, f.routing_.opening_hours_weekday_ranges_[1].to_);
  EXPECT_EQ(7U, f.routing_.opening_hours_weekday_ranges_[2].from_);
  EXPECT_EQ(7U, f.routing_.opening_hours_weekday_ranges_[2].to_);

  ASSERT_EQ(2U, f.routing_.opening_hours_time_spans_.size());
  EXPECT_EQ(9U * 60U, f.routing_.opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(17U * 60U, f.routing_.opening_hours_time_spans_[0].to_minutes_);
  EXPECT_EQ(10U * 60U, f.routing_.opening_hours_time_spans_[1].from_minutes_);
  EXPECT_EQ(14U * 60U, f.routing_.opening_hours_time_spans_[1].to_minutes_);
}

TEST(conditional_parser, rejects_unsupported_symbolic_conditions) {
  for (std::string_view const value :
       {"no @ (customers)"sv, "no @ (agricultural)"sv, "no @ (forestry)"sv,
        "no @ (permit)"sv, "no @ (wet)"sv, "no @ (snow)"sv, "no @ (disabled)"sv,
        "no @ (doctor)"sv, "no @ (emergency)"sv, "no @ (female)"sv,
        "no @ (foo)"sv}) {
    auto f = parser_fixture{};
    EXPECT_FALSE(f.parse("access:conditional"sv, value));
    EXPECT_TRUE(f.builder_.way_.empty());
    EXPECT_TRUE(f.routing_.conditional_access_.empty());
    EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());
  }

  auto f = parser_fixture{};
  EXPECT_FALSE(f.parse_condition_set("wet"sv).has_value());
  EXPECT_FALSE(f.parse_condition_set("foo"sv).has_value());
  EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());
  EXPECT_TRUE(f.routing_.conditional_conditions_.empty());
}

TEST(conditional_parser, rejects_unsupported_vehicle_properties) {
  for (std::string_view const value :
       {"no @ (wheels > 6)"sv, "no @ (occupants > 2)"sv}) {
    auto f = parser_fixture{};
    EXPECT_FALSE(f.parse("access:conditional"sv, value));
    EXPECT_TRUE(f.builder_.way_.empty());
    EXPECT_TRUE(f.routing_.conditional_access_.empty());
    EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());
    EXPECT_TRUE(f.routing_.conditional_conditions_.empty());
  }

  auto f = parser_fixture{};
  EXPECT_FALSE(f.parse_condition_set("wheels > 6"sv).has_value());
  EXPECT_FALSE(f.parse_condition_set("occupants > 2"sv).has_value());
  EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());
  EXPECT_TRUE(f.routing_.conditional_conditions_.empty());
}

TEST(conditional_parser, ignores_unsupported_condition_pieces) {
  auto f = parser_fixture{};

  ASSERT_TRUE(f.parse("maxheight:hgv:conditional"sv,
                      "3.8 @ (weight > 7.5 AND wheels > 6); "
                      "4.0 @ (weight > 7.5)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
  EXPECT_EQ(400U, f.routing_.conditional_numeric_[0].value_.value_);
  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(1U, f.routing_.conditional_conditions_.size());

  auto const& weight = f.condition(0U);
  EXPECT_EQ(osr::conditional_condition_type::kVehicleProperty, weight.type_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_vehicle_property::kWeight),
      weight.selector_);
  EXPECT_EQ(osr::conditional_comparison::kGreater, weight.comparison_);
  EXPECT_EQ(75U, weight.value_.value_);

  auto g = parser_fixture{};
  EXPECT_FALSE(
      g.parse_condition_set("weight > 7.5 AND wheels > 6"sv).has_value());
  EXPECT_TRUE(g.routing_.conditional_condition_sets_.empty());
  EXPECT_TRUE(g.routing_.conditional_conditions_.empty());
}

TEST(conditional_parser, ignores_unsupported_clauses) {
  auto f = parser_fixture{};

  ASSERT_TRUE(
      f.parse("access:conditional"sv, "no @ (wheels > 6); no @ (trailer)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_access_.size());
  EXPECT_EQ(osr::access_value::kNo,
            f.routing_.conditional_access_.front().value_);
  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(1U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(osr::conditional_condition_type::kVehicleUsage,
            f.condition(0U).type_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_symbolic_condition::kTrailer),
      f.condition(0U).selector_);
}

TEST(conditional_parser, rejects_unsupported_conditional_keys) {
  for (std::string_view const key :
       {"vehicle:conditional"sv, "maxstay:conditional"sv, "fee:conditional"sv,
        "opening_hours:conditional"sv}) {
    auto f = parser_fixture{};
    EXPECT_FALSE(f.parse(key, "no @ (Mo-Fr 06:00-10:00)"sv));
    EXPECT_TRUE(f.builder_.way_.empty());
    EXPECT_TRUE(f.routing_.conditional_access_.empty());
    EXPECT_TRUE(f.routing_.conditional_numeric_.empty());
    EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());
  }
}

TEST(conditional_parser, failed_parse_rolls_back_storage) {
  auto f = parser_fixture{};

  EXPECT_FALSE(f.parse("access:conditional"sv,
                       "no @ (Mo-Fr 06:00-10:00); "
                       "no @ (weight > foo)"sv));
  EXPECT_TRUE(f.builder_.way_.empty());
  EXPECT_TRUE(f.routing_.conditional_access_.empty());
  EXPECT_TRUE(f.routing_.conditional_condition_sets_.empty());

  ASSERT_TRUE(f.parse("access:conditional"sv, "no @ (Mo-Fr 06:00-10:00)"sv));
  EXPECT_EQ(1U, f.routing_.conditional_condition_sets_.size());
}

TEST(conditional_parser, rejects_unsupported_opening_hours_features) {
  for (std::string_view const value :
       {"no @ (PH)"sv, "no @ (sunrise-sunset)"sv,
        "no @ (Mo-Fr \"except deliveries\")"sv, "no @ (easter)"sv}) {
    auto f = parser_fixture{};
    EXPECT_FALSE(f.parse("access:conditional"sv, value));
    EXPECT_TRUE(f.builder_.way_.empty());
    EXPECT_TRUE(f.routing_.conditional_access_.empty());
    EXPECT_TRUE(f.routing_.opening_hours_.empty());
  }
}

TEST(conditional_parser, parses_turn_restriction_condition_sets) {
  auto f = parser_fixture{};

  auto const time_condition =
      f.parse_condition_set("Mo-Fr 07:00-09:00,16:00-18:00"sv);
  ASSERT_TRUE(time_condition.has_value());
  ASSERT_EQ(1U, f.routing_.conditional_condition_sets_.size());
  ASSERT_EQ(1U, f.routing_.conditional_conditions_.size());
  EXPECT_EQ(osr::conditional_condition_type::kOpeningHours,
            f.routing_.conditional_conditions_.front().type_);

  auto const vehicle_condition = f.parse_condition_set("length > 6"sv);
  ASSERT_TRUE(vehicle_condition.has_value());
  ASSERT_EQ(2U, f.routing_.conditional_conditions_.size());
  auto const& length = f.routing_.conditional_conditions_.back();
  EXPECT_EQ(osr::conditional_condition_type::kVehicleProperty, length.type_);
  EXPECT_EQ(osr::conditional_comparison::kGreater, length.comparison_);
  EXPECT_EQ(
      static_cast<std::uint32_t>(osr::conditional_vehicle_property::kLength),
      length.selector_);
  EXPECT_EQ(600U, length.value_.value_);
}
