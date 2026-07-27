#pragma once

#include <cassert>
#include <cstdint>

#include <chrono>
#include <optional>
#include <string_view>
#include <utility>

#include "date/iso_week.h"
#include "date/tz.h"

#include "osr/ways.h"

namespace osr {

struct conditional_wall_date {
  std::int32_t year_{};
  std::uint8_t month_{};
  std::uint8_t day_{};
  std::uint8_t weekday_{};
  std::uint8_t iso_week_{};
};

struct conditional_wall_time {
  conditional_wall_date date_{};
  conditional_wall_date previous_date_{};
  std::uint16_t minutes_{};
};

inline conditional_wall_date to_conditional_wall_date(
    std::chrono::sys_days const day) {
  auto const ymd = std::chrono::year_month_day{day};
  auto const iso = iso_week::year_weeknum_weekday{
      iso_week::sys_days{day.time_since_epoch()}};
  return conditional_wall_date{
      .year_ = static_cast<std::int32_t>(int(ymd.year())),
      .month_ = static_cast<std::uint8_t>(unsigned(ymd.month())),
      .day_ = static_cast<std::uint8_t>(unsigned(ymd.day())),
      .weekday_ = static_cast<std::uint8_t>(unsigned{iso.weekday()}),
      .iso_week_ = static_cast<std::uint8_t>(unsigned{iso.weeknum()})};
}

inline constexpr bool compare_conditional_value(
    std::uint32_t const lhs,
    conditional_comparison const comparison,
    std::uint32_t const rhs) {
  switch (comparison) {
    case conditional_comparison::kNone: return lhs != 0U;
    case conditional_comparison::kEqual: return lhs == rhs;
    case conditional_comparison::kLess: return lhs < rhs;
    case conditional_comparison::kLessEqual: return lhs <= rhs;
    case conditional_comparison::kGreater: return lhs > rhs;
    case conditional_comparison::kGreaterEqual: return lhs >= rhs;
  }
  return false;
}

inline std::optional<conditional_wall_time> to_conditional_wall_time(
    std::optional<routing_time_t> const t,
    conditional_timezone_idx_t const timezone_idx,
    timezone_cache_t const& timezones) {
  if (!t.has_value()) {
    return std::nullopt;
  }
  auto local_time = *t;
  if (timezone_idx != conditional_timezone_idx_t::invalid()) {
    assert(to_idx(timezone_idx) < timezones.size());
    auto const* timezone = timezones[to_idx(timezone_idx)];
    auto const zoned = date::zoned_time<routing_time_t::duration>{timezone, *t};
    local_time = routing_time_t{zoned.get_local_time().time_since_epoch()};
  }
  auto const day = std::chrono::floor<std::chrono::days>(local_time);
  auto const minutes =
      std::chrono::duration_cast<std::chrono::minutes>(local_time - day)
          .count();
  return conditional_wall_time{
      .date_ = to_conditional_wall_date(day),
      .previous_date_ = to_conditional_wall_date(day - std::chrono::days{1}),
      .minutes_ = static_cast<std::uint16_t>(minutes)};
}

template <typename Range, typename Fn>
bool conditional_any_matches(conditional_range const range,
                             Range const& values,
                             Fn&& fn) {
  if (range.empty()) {
    return true;
  }
  for (auto i = range.begin_; i != range.end_; ++i) {
    if (fn(values[i])) {
      return true;
    }
  }
  return false;
}

inline constexpr bool matches_conditional_year(
    conditional_wall_date const& t, opening_hours_year_range const& r) {
  return t.year_ >= r.from_ && t.year_ <= r.to_ &&
         ((t.year_ - r.from_) % r.step_) == 0;
}

inline constexpr bool matches_conditional_week(
    conditional_wall_date const& t, opening_hours_week_range const& r) {
  return t.iso_week_ >= r.from_ && t.iso_week_ <= r.to_ &&
         ((t.iso_week_ - r.from_) % r.step_) == 0;
}

inline constexpr bool matches_conditional_weekday(
    conditional_wall_date const& t, opening_hours_weekday_range const& r) {
  return r.from_ <= r.to_ ? t.weekday_ >= r.from_ && t.weekday_ <= r.to_
                          : t.weekday_ >= r.from_ || t.weekday_ <= r.to_;
}

inline constexpr bool matches_conditional_monthday(
    conditional_wall_date const& t, opening_hours_monthday_range const& r) {
  auto const value = static_cast<std::uint16_t>(t.month_ * 32U + t.day_);
  auto const date_value = [](opening_hours_date const& d, bool const end) {
    auto const month = d.month_ == 0U ? (end ? 12U : 1U) : d.month_;
    auto const day = d.day_ == 0U ? (end ? 31U : 1U) : d.day_;
    return static_cast<std::uint16_t>(month * 32U + day);
  };
  if (r.from_.year_ != 0U && t.year_ < r.from_.year_) {
    return false;
  }
  if (r.to_.year_ != 0U && t.year_ > r.to_.year_) {
    return false;
  }
  auto const from = date_value(r.from_, false);
  auto const to = date_value(r.to_, true);
  return from <= to ? value >= from && value <= to
                    : value >= from || value <= to;
}

inline constexpr bool matches_conditional_time(
    conditional_wall_time const& t, opening_hours_time_span const& r) {
  auto const to = r.to_minutes_ <= r.from_minutes_
                      ? static_cast<std::uint16_t>(r.to_minutes_ + 24U * 60U)
                      : r.to_minutes_;
  return (t.minutes_ >= r.from_minutes_ && t.minutes_ < to) ||
         (static_cast<std::uint16_t>(t.minutes_ + 24U * 60U) >=
              r.from_minutes_ &&
          static_cast<std::uint16_t>(t.minutes_ + 24U * 60U) < to);
}

inline bool matches_conditional_date(conditional_wall_date const& t,
                                     ways::routing const& w,
                                     opening_hours_rule const& rule) {
  return conditional_any_matches(
             rule.years_, w.opening_hours_year_ranges_,
             [&](auto const& r) { return matches_conditional_year(t, r); }) &&
         conditional_any_matches(
             rule.weeks_, w.opening_hours_week_ranges_,
             [&](auto const& r) { return matches_conditional_week(t, r); }) &&
         conditional_any_matches(rule.monthdays_,
                                 w.opening_hours_monthday_ranges_,
                                 [&](auto const& r) {
                                   return matches_conditional_monthday(t, r);
                                 }) &&
         conditional_any_matches(
             rule.weekdays_, w.opening_hours_weekday_ranges_,
             [&](auto const& r) { return matches_conditional_weekday(t, r); });
}

inline bool matches_conditional_time_with_date(
    conditional_wall_time const& t,
    ways::routing const& w,
    opening_hours_rule const& rule,
    opening_hours_time_span const& r) {
  auto const to = r.to_minutes_ <= r.from_minutes_
                      ? static_cast<std::uint16_t>(r.to_minutes_ + 24U * 60U)
                      : r.to_minutes_;
  if (t.minutes_ >= r.from_minutes_ && t.minutes_ < to) {
    return matches_conditional_date(t.date_, w, rule);
  }
  auto const previous_day_minutes =
      static_cast<std::uint16_t>(t.minutes_ + 24U * 60U);
  return previous_day_minutes >= r.from_minutes_ && previous_day_minutes < to &&
         matches_conditional_date(t.previous_date_, w, rule);
}

inline bool matches_conditional_opening_hours(ways::routing const& w,
                                              std::uint32_t const oh_idx,
                                              conditional_wall_time const& t) {
  auto const& oh = w.opening_hours_[oh_idx];
  for (auto i = oh.rules_.begin_; i != oh.rules_.end_; ++i) {
    auto const& rule = w.opening_hours_rules_[i];
    auto const matches =
        rule.times_.empty()
            ? matches_conditional_date(t.date_, w, rule)
            : conditional_any_matches(
                  rule.times_, w.opening_hours_time_spans_, [&](auto const& r) {
                    return matches_conditional_time_with_date(t, w, rule, r);
                  });
    if (matches) {
      return rule.modifier_ != opening_hours_rule_modifier::kClosed;
    }
  }
  return false;
}

template <typename Fn>
bool matches_conditional_condition(
    ways::routing const& w,
    conditional_condition const& c,
    std::optional<conditional_wall_time> const& t,
    Fn&& fn) {
  switch (c.type_) {
    case conditional_condition_type::kOpeningHours:
      return t.has_value() &&
             matches_conditional_opening_hours(w, c.selector_, *t);
    case conditional_condition_type::kVehicleProperty: [[fallthrough]];
    case conditional_condition_type::kAccessPurpose: [[fallthrough]];
    case conditional_condition_type::kVehicleUsage: return fn(c);
  }
  return false;
}

template <typename Fn>
bool matches_conditional_condition_set(
    ways::routing const& w,
    conditional_condition_set_idx_t const idx,
    std::optional<conditional_wall_time> const& t,
    Fn&& fn) {
  if (idx == conditional_condition_set_idx_t::invalid()) {
    return false;
  }
  auto const& set = w.conditional_condition_sets_[to_idx(idx)];
  for (auto i = set.conditions_.begin_; i != set.conditions_.end_; ++i) {
    if (!matches_conditional_condition(w, w.conditional_conditions_[i], t,
                                       fn)) {
      return false;
    }
  }
  return true;
}

template <typename Fn>
bool matches_conditional_condition_set_utc(
    ways::routing const& w,
    timezone_cache_t const& timezones,
    conditional_condition_set_idx_t const idx,
    std::optional<routing_time_t> const utc_time,
    Fn&& fn) {
  if (idx == conditional_condition_set_idx_t::invalid()) {
    return false;
  }
  auto const& set = w.conditional_condition_sets_[to_idx(idx)];
  return matches_conditional_condition_set(
      w, idx, to_conditional_wall_time(utc_time, set.timezone_, timezones),
      std::forward<Fn>(fn));
}

inline constexpr bool conditional_direction_applies(
    conditional_osm_direction const condition_dir, direction const way_dir) {
  return condition_dir == conditional_osm_direction::kNone ||
         (condition_dir == conditional_osm_direction::kForward &&
          way_dir == direction::kForward) ||
         (condition_dir == conditional_osm_direction::kBackward &&
          way_dir == direction::kBackward);
}

}  // namespace osr
