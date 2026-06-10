#include <chrono>
#include <optional>
#include <string_view>

#include "gtest/gtest.h"

#include "osr/extract/conditional_parser.h"
#include "osr/routing/conditional.h"
#include "osr/routing/profiles/hgv.h"
#include "osr/ways.h"

using namespace std::string_view_literals;

namespace {

osr::conditional_wall_time wall_time(int const year,
                                     unsigned const month,
                                     unsigned const day,
                                     int const hour,
                                     int const minute) {
  auto const tp = std::chrono::time_point_cast<std::chrono::seconds>(
      std::chrono::sys_days{std::chrono::year{year} /
                            std::chrono::month{month} / std::chrono::day{day}} +
      std::chrono::hours{hour} + std::chrono::minutes{minute});
  return *osr::to_conditional_wall_time(tp);
}

osr::routing_time_t utc_time(int const year,
                             unsigned const month,
                             unsigned const day,
                             int const hour,
                             int const minute) {
  return std::chrono::time_point_cast<std::chrono::seconds>(
      std::chrono::sys_days{std::chrono::year{year} /
                            std::chrono::month{month} / std::chrono::day{day}} +
      std::chrono::hours{hour} + std::chrono::minutes{minute});
}

struct match_fixture {
  match_fixture() : builder_{.routing_ = routing_} {}

  bool parse(std::string_view const key, std::string_view const value) {
    return osr::parse_conditional_restriction_tag(key, value, builder_);
  }

  std::optional<osr::conditional_condition_set_idx_t> parse_condition_set(
      std::string_view const condition) {
    return osr::parse_conditional_condition_set(condition, builder_);
  }

  bool matches_opening_hours(
      osr::conditional_condition_set_idx_t const condition_set,
      osr::conditional_wall_time const& t) const {
    return osr::matches_conditional_condition_set(
        routing_, condition_set, std::optional{t},
        [](auto const&) { return false; });
  }

  bool matches(osr::conditional_condition_set_idx_t const condition_set,
               int const year,
               int const month,
               int const day,
               int const hour,
               int const minute) const {
    return matches_opening_hours(condition_set,
                                 wall_time(year, month, day, hour, minute));
  }

  bool matches_profile(
      osr::hgv::parameters const& params,
      osr::conditional_condition_set_idx_t const condition_set,
      std::optional<osr::conditional_wall_time> const& t) const {
    return osr::hgv::matches_profile_condition_set(params, routing_,
                                                   condition_set, t);
  }

  bool matches_profile(osr::hgv::parameters const& params,
                       osr::conditional_condition_set_idx_t const condition_set,
                       int const year,
                       int const month,
                       int const day,
                       int const hour,
                       int const minute) const {
    return matches_profile(
        params, condition_set,
        std::optional{wall_time(year, month, day, hour, minute)});
  }

  bool matches_profile(
      osr::hgv::parameters const& params,
      osr::conditional_condition_set_idx_t const condition_set) const {
    return matches_profile(params, condition_set, std::nullopt);
  }

  bool matches_profile_utc(
      osr::hgv::parameters const& params,
      osr::conditional_condition_set_idx_t const condition_set,
      osr::routing_time_t const t) const {
    return osr::hgv::matches_profile_condition_set_utc(params, routing_,
                                                       condition_set, t);
  }

  osr::ways::routing routing_{};
  osr::conditional_storage_builder builder_;
};

}  // namespace

TEST(conditional_match, weekday_time_range) {
  auto f = match_fixture{};
  ASSERT_TRUE(
      f.parse("hgv:conditional"sv, "destination @ (Mo-Fr 06:00-10:00)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 6, 0));  // Mon 06:00
  EXPECT_TRUE(f.matches(idx, 2024, 6, 5, 8, 0));  // Wed 08:00
  EXPECT_TRUE(f.matches(idx, 2024, 6, 7, 9, 59));  // Fri 09:59

  EXPECT_FALSE(f.matches(idx, 2024, 6, 3, 5, 0));  // Mon 05:00 (before time)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 3, 10, 0));  // Mon 10:00 (end)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 8, 8, 0));  // Sat 08:00 (wrong day)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 9, 8, 0));  // Sun 08:00 (wrong day)
}

TEST(conditional_match, wall_date_uses_iso_week_number) {
  auto const wall_date = [](int const year, unsigned const month,
                            unsigned const day) {
    return osr::to_conditional_wall_date(std::chrono::sys_days{
        std::chrono::year{year} / std::chrono::month{month} /
        std::chrono::day{day}});
  };

  EXPECT_EQ(53U, wall_date(2021, 1, 1).iso_week_);
  EXPECT_EQ(1U, wall_date(2024, 1, 1).iso_week_);
  EXPECT_EQ(7U, wall_date(2024, 1, 7).weekday_);
  EXPECT_EQ(1U, wall_date(2024, 12, 30).iso_week_);
}

TEST(conditional_match, timezone_converts_utc_to_local_wall_time) {
  auto f = match_fixture{};
  f.routing_.timezones_.emplace_back("Europe/Berlin"sv);
  f.builder_.timezone_ = osr::conditional_timezone_idx_t{0U};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (08:00-09:00)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;
  auto const params = osr::hgv::parameters{};

  EXPECT_TRUE(f.matches_profile_utc(params, idx, utc_time(2024, 6, 3, 6, 30)));
  EXPECT_FALSE(f.matches_profile_utc(params, idx, utc_time(2024, 6, 3, 8, 30)));
  EXPECT_TRUE(f.matches_profile_utc(params, idx, utc_time(2024, 12, 3, 7, 30)));
}

TEST(conditional_match, timezone_handles_dst_spring_forward) {
  auto f = match_fixture{};
  f.routing_.timezones_.emplace_back("Europe/Berlin"sv);
  f.builder_.timezone_ = osr::conditional_timezone_idx_t{0U};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (Su 02:00-03:30)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;
  auto const params = osr::hgv::parameters{};

  EXPECT_FALSE(
      f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 0, 30)));
  EXPECT_TRUE(f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 1, 0)));
  EXPECT_TRUE(f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 1, 29)));
  EXPECT_FALSE(
      f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 1, 30)));
}

TEST(conditional_match, timezone_handles_dst_fall_back) {
  auto f = match_fixture{};
  f.routing_.timezones_.emplace_back("Europe/Berlin"sv);
  f.builder_.timezone_ = osr::conditional_timezone_idx_t{0U};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (Su 02:00-03:00)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;
  auto const params = osr::hgv::parameters{};

  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 0, 30)));
  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 1, 30)));
  EXPECT_FALSE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 2, 0)));
}

TEST(conditional_match,
     timezone_handles_dst_spring_forward_in_overnight_range) {
  auto f = match_fixture{};
  f.routing_.timezones_.emplace_back("Europe/Berlin"sv);
  f.builder_.timezone_ = osr::conditional_timezone_idx_t{0U};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (Sa 22:00-03:30)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;
  auto const params = osr::hgv::parameters{};

  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 3, 30, 22, 30)));
  EXPECT_TRUE(f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 1, 15)));
  EXPECT_FALSE(
      f.matches_profile_utc(params, idx, utc_time(2024, 3, 31, 1, 30)));
}

TEST(conditional_match, timezone_handles_dst_fall_back_in_overnight_range) {
  auto f = match_fixture{};
  f.routing_.timezones_.emplace_back("Europe/Berlin"sv);
  f.builder_.timezone_ = osr::conditional_timezone_idx_t{0U};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (Sa 22:00-03:00)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;
  auto const params = osr::hgv::parameters{};

  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 26, 21, 30)));
  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 0, 30)));
  EXPECT_TRUE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 1, 30)));
  EXPECT_FALSE(
      f.matches_profile_utc(params, idx, utc_time(2024, 10, 27, 2, 0)));
}

TEST(conditional_match, always_24_7) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("hgv:conditional"sv, "no @ (24/7)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 0, 0));
  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 12, 0));
  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 23, 59));
  EXPECT_TRUE(f.matches(idx, 2024, 12, 25, 8, 0));
}

TEST(conditional_match, open_ended_time) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("access:conditional"sv, "no @ (Mo 18:00+)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 18, 0));  // Mon 18:00
  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 23, 0));  // Mon 23:00

  EXPECT_FALSE(f.matches(idx, 2024, 6, 3, 17, 59));  // Mon 17:59 (before time)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 4, 18, 0));  // Tue 18:00 (wrong day)
}

TEST(conditional_match, midnight_spanning_time) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("motor_vehicle:conditional"sv, "no @ 18:00-06:00"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 18, 0));  // 18:00 (start)
  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 22, 0));  // 22:00 (same day)
  EXPECT_TRUE(f.matches(idx, 2024, 6, 4, 2, 0));  // 02:00 (next day)

  EXPECT_FALSE(f.matches(idx, 2024, 6, 3, 10, 0));  // 10:00 (outside range)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 4, 6, 0));  // 06:00 (end)
}

TEST(conditional_match, calendar_selectors) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("access:conditional"sv,
                      "no @ (2024-2026 May 01-Oct 31 week 01-53/2 "
                      "Su[1] 08:00-12:00,13:00-17:00)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(
      f.matches(idx, 2024, 7, 7, 10, 0));  // Jul 7 (1st Su, odd week), 10:00

  EXPECT_FALSE(f.matches(idx, 2024, 4, 30, 10, 0));  // Apr 30 (before May)
  EXPECT_FALSE(f.matches(idx, 2024, 5, 1, 10, 0));  // May 1 (Wed, not Su)
  EXPECT_FALSE(f.matches(idx, 2024, 7, 7, 7, 0));  // Jul 7 07:00 (before time)
  EXPECT_FALSE(
      f.matches(idx, 2024, 7, 7, 12, 30));  // Jul 7 12:30 (between times)
}

TEST(conditional_match, multiple_rules_with_modifiers) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("access:conditional"sv,
                      "no @ (Mo-Fr 09:00-17:00 open; "
                      "Sa 10:00-14:00 closed; Su unknown)"sv));

  auto const idx = f.routing_.conditional_access_[0].condition_set_;

  EXPECT_TRUE(f.matches(idx, 2024, 6, 3, 10, 0));  // Mon 10:00 (open)
  EXPECT_TRUE(
      f.matches(idx, 2024, 6, 9, 12, 0));  // Sun 12:00 (unknown → allowed)

  EXPECT_FALSE(f.matches(idx, 2024, 6, 3, 8, 0));  // Mon 08:00 (before time)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 8, 12, 0));  // Sat 12:00 (closed)
  EXPECT_FALSE(f.matches(idx, 2024, 6, 8, 9, 0));  // Sat 09:00 (before Sa time)
}

TEST(conditional_match, vehicle_property_weight) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("maxheight:hgv:conditional"sv, "4.0 @ (weight > 7.5)"sv));

  ASSERT_EQ(1U, f.routing_.conditional_numeric_.size());
  auto const idx = f.routing_.conditional_numeric_[0].condition_set_;

  auto const heavy = osr::hgv::parameters{.weight_100kg_ = 80U};  // 8.0 t
  auto const light = osr::hgv::parameters{.weight_100kg_ = 70U};  // 7.0 t

  EXPECT_TRUE(f.matches_profile(heavy, idx));
  EXPECT_FALSE(f.matches_profile(light, idx));
}

TEST(conditional_match, vehicle_property_height_equal) {
  auto f = match_fixture{};

  auto const idx = f.parse_condition_set("height = 3.8 m"sv);
  ASSERT_TRUE(idx.has_value());

  auto const tall = osr::hgv::parameters{.height_cm_ = 380U};  // 3.8 m
  auto const too_tall = osr::hgv::parameters{.height_cm_ = 400U};  // 4.0 m

  EXPECT_TRUE(f.matches_profile(tall, *idx));
  EXPECT_FALSE(f.matches_profile(too_tall, *idx));
}

TEST(conditional_match, vehicle_property_axles_equal) {
  auto f = match_fixture{};

  auto const idx = f.parse_condition_set("axles == 5"sv);
  ASSERT_TRUE(idx.has_value());

  auto const five_axles = osr::hgv::parameters{.axle_count_ = 5U};
  auto const three_axles = osr::hgv::parameters{.axle_count_ = 3U};

  EXPECT_TRUE(f.matches_profile(five_axles, *idx));
  EXPECT_FALSE(f.matches_profile(three_axles, *idx));
}

TEST(conditional_match, vehicle_property_length_less) {
  auto f = match_fixture{};

  auto const idx = f.parse_condition_set("length < 10"sv);
  ASSERT_TRUE(idx.has_value());

  auto const short_vehicle = osr::hgv::parameters{.length_cm_ = 900U};  // 9 m
  auto const long_vehicle = osr::hgv::parameters{.length_cm_ = 1200U};  // 12 m

  EXPECT_TRUE(f.matches_profile(short_vehicle, *idx));
  EXPECT_FALSE(f.matches_profile(long_vehicle, *idx));
}

TEST(conditional_match, vehicle_property_weight_rating_less_equal) {
  auto f = match_fixture{};

  auto const idx = f.parse_condition_set("weight_rating <= 900 kg"sv);
  ASSERT_TRUE(idx.has_value());

  auto const light = osr::hgv::parameters{.weight_100kg_ = 8U};  // 800 kg
  auto const heavy = osr::hgv::parameters{.weight_100kg_ = 10U};  // 1000 kg

  EXPECT_TRUE(f.matches_profile(light, *idx));
  EXPECT_FALSE(f.matches_profile(heavy, *idx));
}

TEST(conditional_match, vehicle_usage_hazmat_and_trailer) {
  auto f = match_fixture{};

  auto const idx = f.parse_condition_set("hazmat AND trailer"sv);
  ASSERT_TRUE(idx.has_value());

  auto const with_hazmat_trailer =
      osr::hgv::parameters{.hazmat_ = true, .trailer_ = true};

  auto const no_hazmat = osr::hgv::parameters{.trailer_ = true};

  auto const no_trailer =
      osr::hgv::parameters{.hazmat_ = true, .trailer_ = false};

  EXPECT_TRUE(f.matches_profile(with_hazmat_trailer, *idx));
  EXPECT_FALSE(f.matches_profile(no_hazmat, *idx));
  EXPECT_FALSE(f.matches_profile(no_trailer, *idx));
}

TEST(conditional_match, vehicle_property_with_time) {
  auto f = match_fixture{};
  ASSERT_TRUE(f.parse("maxspeed:hgv:forward:conditional"sv,
                      "50 mph @ (weight > 7.5 AND Dec-Mar)"sv));

  auto const idx = f.routing_.conditional_numeric_[0].condition_set_;

  auto const heavy = osr::hgv::parameters{.weight_100kg_ = 80U};  // 8.0 t
  auto const light = osr::hgv::parameters{.weight_100kg_ = 70U};  // 7.0 t

  EXPECT_TRUE(
      f.matches_profile(heavy, idx, 2024, 12, 15, 10, 0));  // Dec, heavy
  EXPECT_FALSE(
      f.matches_profile(heavy, idx, 2024, 6, 15, 10, 0));  // Jun, heavy
  EXPECT_FALSE(
      f.matches_profile(light, idx, 2024, 12, 15, 10, 0));  // Dec, light
}
