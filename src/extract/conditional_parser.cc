#include "osr/extract/conditional_parser.h"

#include <cctype>
#include <cstdint>
#include <cstring>

#include <charconv>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cista/hash.h"

#include "osr/extract/tag_parser.h"

namespace osr {

namespace {

using namespace std::string_view_literals;

enum class restriction_kind : std::uint8_t { kAccess, kOneway, kNumeric };
enum class parse_result : std::uint8_t { kAdded, kIgnored, kError };

struct conditional_key {
  restriction_kind kind_{restriction_kind::kAccess};
  conditional_restriction_field field_{};
  conditional_transport_mode mode_{conditional_transport_mode::kUnspecified};
  conditional_osm_direction direction_{conditional_osm_direction::kNone};
};

bool is_space(char const c) {
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

bool is_digit(char const c) {
  return std::isdigit(static_cast<unsigned char>(c)) != 0;
}

std::string to_lower(std::string_view s) {
  auto out = std::string{};
  out.reserve(s.size());
  std::transform(
      begin(s), end(s), std::back_inserter(out),
      [](unsigned char const c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

std::string_view strip_suffix_char(std::string_view s, char const c) {
  while (!s.empty() && s.back() == c) {
    s.remove_suffix(1U);
  }
  return s;
}

std::vector<std::string_view> split(std::string_view s, char const sep) {
  auto parts = std::vector<std::string_view>{};
  while (true) {
    auto const pos = s.find(sep);
    parts.push_back(trim(s.substr(0U, pos)));
    if (pos == std::string_view::npos) {
      return parts;
    }
    s.remove_prefix(pos + 1U);
  }
}

std::vector<std::string_view> split_ws(std::string_view s) {
  auto parts = std::vector<std::string_view>{};
  while (!s.empty()) {
    s = trim(s);
    if (s.empty()) {
      break;
    }
    auto pos = std::size_t{0U};
    while (pos != s.size() && !is_space(s[pos])) {
      ++pos;
    }
    parts.push_back(strip_suffix_char(s.substr(0U, pos), ','));
    s.remove_prefix(pos);
  }
  return parts;
}

std::vector<std::string_view> split_clauses(std::string_view s) {
  auto parts = std::vector<std::string_view>{};
  auto depth = 0;
  auto start = std::size_t{0U};
  for (auto i = std::size_t{0U}; i != s.size(); ++i) {
    if (s[i] == '(') {
      ++depth;
    } else if (s[i] == ')' && depth != 0) {
      --depth;
    } else if (s[i] == ';' && depth == 0) {
      parts.push_back(trim(s.substr(start, i - start)));
      start = i + 1U;
    }
  }
  parts.push_back(trim(s.substr(start)));
  return parts;
}

std::vector<std::string_view> split_condition_and(std::string_view s) {
  auto parts = std::vector<std::string_view>{};
  auto const lower = to_lower(s);
  auto start = std::size_t{0U};
  auto i = std::size_t{0U};
  while (i < s.size()) {
    if (i + 2U <= s.size() && s.substr(i, 2U) == "&&"sv) {
      parts.push_back(trim(s.substr(start, i - start)));
      i += 2U;
      start = i;
      continue;
    }
    if (i + 5U <= s.size() && lower.substr(i, 5U) == " and "sv) {
      parts.push_back(trim(s.substr(start, i - start)));
      i += 5U;
      start = i;
      continue;
    }
    ++i;
  }
  parts.push_back(trim(s.substr(start)));
  return parts;
}

std::optional<std::uint32_t> parse_u32(std::string_view s) {
  s = trim(s);
  if (s.empty()) {
    return std::nullopt;
  }
  auto value = std::uint32_t{0U};
  auto const [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), value);
  if (ec != std::errc{} || ptr != s.data() + s.size()) {
    return std::nullopt;
  }
  return value;
}

std::optional<access_value> parse_access_value(std::string_view value) {
  auto const lower = to_lower(trim(value));
  switch (cista::hash(lower)) {
    case cista::hash("yes"): return access_value::kYes;
    case cista::hash("designated"): return access_value::kDesignated;
    case cista::hash("permissive"): return access_value::kPermissive;
    case cista::hash("private"): return access_value::kPrivate;
    case cista::hash("no"): return access_value::kNo;
    case cista::hash("destination"): return access_value::kDestination;
    case cista::hash("delivery"): return access_value::kDelivery;
    case cista::hash("discouraged"): return access_value::kDiscouraged;
    default: return std::nullopt;
  }
}

std::optional<conditional_oneway_value> parse_oneway_value(
    std::string_view value) {
  auto const lower = to_lower(trim(value));
  switch (cista::hash(lower)) {
    case cista::hash("yes"):
    case cista::hash("true"):
    case cista::hash("1"): return conditional_oneway_value::kForward;
    case cista::hash("-1"):
    case cista::hash("reverse"):
    case cista::hash("backward"): return conditional_oneway_value::kBackward;
    case cista::hash("no"):
    case cista::hash("false"):
    case cista::hash("0"): return conditional_oneway_value::kNo;
    default: return std::nullopt;
  }
}

std::optional<conditional_numeric_value> parse_numeric_value(
    conditional_restriction_field const field, std::string_view value) {
  auto const lower = to_lower(trim(value));
  if (lower == "none" || lower == "no" || lower == "unknown") {
    return conditional_numeric_value{.state_ =
                                         conditional_numeric_state::kNone};
  }

  auto parsed = std::optional<std::uint32_t>{};
  auto unit = conditional_numeric_unit::kUnitless;
  switch (field) {
    case conditional_restriction_field::kMaxSpeed:
      parsed = to_integer<std::uint32_t>(parse_speed_km_h(value));
      unit = conditional_numeric_unit::kKilometerPerHour;
      break;
    case conditional_restriction_field::kMaxLength:
      parsed = to_integer<std::uint32_t>(parse_length_m(value), 100.0);
      unit = conditional_numeric_unit::kCentimeter;
      break;
    case conditional_restriction_field::kMaxHeight: [[fallthrough]];
    case conditional_restriction_field::kMaxWidth:
      parsed = to_integer<std::uint32_t>(parse_length_m(value), 100.0);
      unit = conditional_numeric_unit::kCentimeter;
      break;
    case conditional_restriction_field::kMaxWeight: [[fallthrough]];
    case conditional_restriction_field::kMaxWeightRating: [[fallthrough]];
    case conditional_restriction_field::kMaxAxleLoad:
      parsed = to_integer<std::uint32_t>(parse_weight_t(value), 10.0);
      unit = conditional_numeric_unit::kWeight100Kg;
      break;
    case conditional_restriction_field::kMaxAxles:
      parsed = to_integer<std::uint32_t>(parse_double(value));
      unit = conditional_numeric_unit::kUnitless;
      break;
    default: return std::nullopt;
  }

  if (!parsed.has_value()) {
    return std::nullopt;
  }
  return conditional_numeric_value{.value_ = *parsed, .unit_ = unit};
}

std::optional<conditional_transport_mode> parse_mode(std::string_view part) {
  auto const lower = to_lower(part);
  switch (cista::hash(lower)) {
    case cista::hash("access"): return conditional_transport_mode::kAccess;
    case cista::hash("vehicle"): return conditional_transport_mode::kVehicle;
    case cista::hash("motor_vehicle"):
      return conditional_transport_mode::kMotorVehicle;
    case cista::hash("motorcar"): return conditional_transport_mode::kMotorcar;
    case cista::hash("hgv"): return conditional_transport_mode::kHgv;
    case cista::hash("bus"): return conditional_transport_mode::kBus;
    case cista::hash("psv"): return conditional_transport_mode::kPsv;
    case cista::hash("foot"): return conditional_transport_mode::kFoot;
    case cista::hash("bicycle"): return conditional_transport_mode::kBicycle;
    default: return std::nullopt;
  }
}

std::optional<conditional_key> parse_conditional_key(std::string_view key) {
  if (!is_supported_conditional_restriction_key(key)) {
    return std::nullopt;
  }

  constexpr auto const kSuffix = ":conditional"sv;
  key.remove_suffix(kSuffix.size());
  auto const parts = split(key, ':');
  if (parts.empty() || parts.front().empty()) {
    return std::nullopt;
  }

  auto out = conditional_key{};
  auto first = parts.front();
  auto second = parts.size() > 1U ? parts[1] : std::string_view{};

  if (first == "hazmat"sv && second == "water"sv) {
    out.kind_ = restriction_kind::kAccess;
    out.field_ = conditional_restriction_field::kHazmatWater;
    out.mode_ = conditional_transport_mode::kHgv;
  } else if (first == "hgv"sv && second == "trailer"sv) {
    out.kind_ = restriction_kind::kAccess;
    out.field_ = conditional_restriction_field::kTrailer;
    out.mode_ = conditional_transport_mode::kHgv;
  } else if (first == "hazmat"sv) {
    out.kind_ = restriction_kind::kAccess;
    out.field_ = conditional_restriction_field::kHazmat;
    out.mode_ = conditional_transport_mode::kHgv;
  } else if (first == "oneway"sv) {
    out.kind_ = restriction_kind::kOneway;
    out.field_ = conditional_restriction_field::kOneway;
  } else if (first == "maxspeed"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxSpeed;
  } else if (first == "maxlength"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxLength;
  } else if (first == "maxweightrating"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxWeightRating;
  } else if (first == "maxheight"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxHeight;
  } else if (first == "maxwidth"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxWidth;
  } else if (first == "maxweight"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxWeight;
  } else if (first == "maxaxleload"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxAxleLoad;
  } else if (first == "maxaxles"sv) {
    out.kind_ = restriction_kind::kNumeric;
    out.field_ = conditional_restriction_field::kMaxAxles;
  } else if (auto const mode = parse_mode(first); mode.has_value()) {
    out.kind_ = restriction_kind::kAccess;
    out.field_ = conditional_restriction_field::kAccess;
    out.mode_ = *mode;
  } else {
    return std::nullopt;
  }

  for (auto i = std::size_t{1U}; i != parts.size(); ++i) {
    if ((parts[i] == "forward"sv || parts[i] == "backward"sv) &&
        !(parts[i - 1U] == "hazmat"sv && parts[i] == "water"sv)) {
      out.direction_ = parts[i] == "forward"sv
                           ? conditional_osm_direction::kForward
                           : conditional_osm_direction::kBackward;
    } else if (auto const mode = parse_mode(parts[i]); mode.has_value()) {
      out.mode_ = *mode;
    }
  }

  return out;
}

template <typename Vec, typename Value>
void push_ranged(conditional_range& range, Vec& vec, Value&& value) {
  if (range.empty()) {
    range.begin_ = static_cast<std::uint32_t>(vec.size());
  }
  vec.push_back(std::forward<Value>(value));
  range.end_ = static_cast<std::uint32_t>(vec.size());
}

std::uint8_t month(std::string_view token) {
  auto const lower =
      to_lower(token.substr(0U, std::min<std::size_t>(3U, token.size())));
  switch (cista::hash(lower)) {
    case cista::hash("jan"): return 1U;
    case cista::hash("feb"): return 2U;
    case cista::hash("mar"): return 3U;
    case cista::hash("apr"): return 4U;
    case cista::hash("may"): return 5U;
    case cista::hash("jun"): return 6U;
    case cista::hash("jul"): return 7U;
    case cista::hash("aug"): return 8U;
    case cista::hash("sep"): return 9U;
    case cista::hash("oct"): return 10U;
    case cista::hash("nov"): return 11U;
    case cista::hash("dec"): return 12U;
    default: return 0U;
  }
}

std::uint8_t weekday(std::string_view token) {
  auto const lower =
      to_lower(token.substr(0U, std::min<std::size_t>(2U, token.size())));
  switch (cista::hash(lower)) {
    case cista::hash("mo"): return 1U;
    case cista::hash("tu"): return 2U;
    case cista::hash("we"): return 3U;
    case cista::hash("th"): return 4U;
    case cista::hash("fr"): return 5U;
    case cista::hash("sa"): return 6U;
    case cista::hash("su"): return 7U;
    default: return 0U;
  }
}

bool is_weekday_selector(std::string_view token) {
  return weekday(token) != 0U && (token.size() == 2U || token[2] == '-' ||
                                  token[2] == '[' || token[2] == ',');
}

std::pair<std::string_view, std::int8_t> strip_nth_weekday(
    std::string_view token) {
  auto const open = token.find('[');
  if (open == std::string_view::npos || !token.ends_with(']')) {
    return {token, 0};
  }
  auto const nth = token.substr(open + 1U, token.size() - open - 2U);
  auto value = 0;
  auto const [ptr, ec] =
      std::from_chars(nth.data(), nth.data() + nth.size(), value);
  if (ec != std::errc{} || ptr != nth.data() + nth.size() || value < -5 ||
      value > 5) {
    return {token.substr(0U, open), 0};
  }
  return {token.substr(0U, open), static_cast<std::int8_t>(value)};
}

std::optional<std::uint16_t> parse_time(std::string_view token) {
  auto const colon = token.find(':');
  if (colon == std::string_view::npos) {
    return std::nullopt;
  }
  auto const h = parse_u32(token.substr(0U, colon));
  auto const m = parse_u32(token.substr(colon + 1U));
  if (!h.has_value() || !m.has_value() || *h > 48U || *m > 59U) {
    return std::nullopt;
  }
  return static_cast<std::uint16_t>((*h * 60U) + *m);
}

bool parse_time_token(std::string_view token,
                      opening_hours_rule& rule,
                      conditional_storage_builder& builder) {
  auto parsed_any = false;
  for (auto part : split(token, ',')) {
    auto span = opening_hours_time_span{};
    if (part.ends_with('+')) {
      part.remove_suffix(1U);
      auto const from = parse_time(part);
      if (!from.has_value()) {
        return false;
      }
      span.from_minutes_ = *from;
      span.to_minutes_ = 24U * 60U;
    } else {
      auto const dash = part.find('-');
      if (dash == std::string_view::npos) {
        return false;
      }
      auto const from = parse_time(part.substr(0U, dash));
      auto const to = parse_time(part.substr(dash + 1U));
      if (!from.has_value() || !to.has_value()) {
        return false;
      }
      span.from_minutes_ = *from;
      span.to_minutes_ = *to;
    }
    push_ranged(rule.times_, builder.routing_.opening_hours_time_spans_, span);
    parsed_any = true;
  }
  return parsed_any;
}

bool parse_weekday_token(std::string_view token,
                         opening_hours_rule& rule,
                         conditional_storage_builder& builder) {
  auto parsed_any = false;
  for (auto part : split(token, ',')) {
    auto const dash = part.find('-');
    auto const [from_token, nth_from] = strip_nth_weekday(
        dash == std::string_view::npos ? part : part.substr(0U, dash));
    auto const [to_token, nth_to] = strip_nth_weekday(
        dash == std::string_view::npos ? part : part.substr(dash + 1U));
    auto const from = weekday(from_token);
    auto const to = weekday(to_token);
    if (from == 0U || to == 0U) {
      return false;
    }
    push_ranged(rule.weekdays_, builder.routing_.opening_hours_weekday_ranges_,
                opening_hours_weekday_range{.from_ = from,
                                            .to_ = to,
                                            .nth_from_ = nth_from,
                                            .nth_to_ = nth_to});
    parsed_any = true;
  }
  return parsed_any;
}

bool parse_week_token(std::string_view token,
                      opening_hours_rule& rule,
                      conditional_storage_builder& builder) {
  auto const slash = token.find('/');
  auto range =
      slash == std::string_view::npos ? token : token.substr(0U, slash);
  auto const step = slash == std::string_view::npos
                        ? std::optional<std::uint32_t>{1U}
                        : parse_u32(token.substr(slash + 1U));
  auto const dash = range.find('-');
  auto const from = parse_u32(range.substr(0U, dash));
  auto const to = dash == std::string_view::npos
                      ? from
                      : parse_u32(range.substr(dash + 1U));
  if (!from.has_value() || !to.has_value() || !step.has_value() || *from < 1U ||
      *to > 53U || *step == 0U) {
    return false;
  }
  push_ranged(
      rule.weeks_, builder.routing_.opening_hours_week_ranges_,
      opening_hours_week_range{.from_ = static_cast<std::uint8_t>(*from),
                               .to_ = static_cast<std::uint8_t>(*to),
                               .step_ = static_cast<std::uint8_t>(*step)});
  return true;
}

bool parse_year_token(std::string_view token,
                      opening_hours_rule& rule,
                      conditional_storage_builder& builder) {
  if (token.size() < 4U || !is_digit(token[0]) || !is_digit(token[1]) ||
      !is_digit(token[2]) || !is_digit(token[3])) {
    return false;
  }
  auto const slash = token.find('/');
  auto range =
      slash == std::string_view::npos ? token : token.substr(0U, slash);
  auto const step = slash == std::string_view::npos
                        ? std::optional<std::uint32_t>{1U}
                        : parse_u32(token.substr(slash + 1U));
  auto const dash = range.find('-');
  auto const from = parse_u32(range.substr(0U, dash));
  auto const to = dash == std::string_view::npos
                      ? from
                      : parse_u32(range.substr(dash + 1U));
  if (!from.has_value() || !to.has_value() || !step.has_value() ||
      *from > std::numeric_limits<std::uint16_t>::max() ||
      *to > std::numeric_limits<std::uint16_t>::max()) {
    return false;
  }
  push_ranged(
      rule.years_, builder.routing_.opening_hours_year_ranges_,
      opening_hours_year_range{.from_ = static_cast<std::uint16_t>(*from),
                               .to_ = static_cast<std::uint16_t>(*to),
                               .step_ = static_cast<std::uint16_t>(*step)});
  return true;
}

bool parse_month_token(std::vector<std::string_view> const& tokens,
                       std::size_t& i,
                       opening_hours_rule& rule,
                       conditional_storage_builder& builder) {
  auto const first = tokens[i];
  auto const dash = first.find('-');
  auto const from_month = month(first.substr(0U, dash));
  if (from_month == 0U) {
    return false;
  }

  auto range = opening_hours_monthday_range{};
  range.from_.month_ = from_month;
  range.to_ = range.from_;

  if (dash != std::string_view::npos) {
    auto const to_month = month(first.substr(dash + 1U));
    if (to_month == 0U) {
      return false;
    }
    range.to_.month_ = to_month;
    push_ranged(rule.monthdays_,
                builder.routing_.opening_hours_monthday_ranges_, range);
    return true;
  }

  if (i + 1U >= tokens.size()) {
    push_ranged(rule.monthdays_,
                builder.routing_.opening_hours_monthday_ranges_, range);
    return true;
  }

  auto const next = tokens[i + 1U];
  if (next.empty() || !is_digit(next.front())) {
    push_ranged(rule.monthdays_,
                builder.routing_.opening_hours_monthday_ranges_, range);
    return true;
  }

  auto const next_dash = next.find('-');
  auto const from_day = parse_u32(next.substr(0U, next_dash));
  if (!from_day.has_value() || *from_day == 0U || *from_day > 31U) {
    return false;
  }
  range.from_.day_ = static_cast<std::uint8_t>(*from_day);
  range.to_ = range.from_;
  ++i;

  if (next_dash != std::string_view::npos) {
    auto const after_dash = next.substr(next_dash + 1U);
    if (!after_dash.empty() && is_digit(after_dash.front())) {
      if (after_dash.size() == 4U && i + 2U < tokens.size()) {
        auto const to_year = parse_u32(after_dash);
        auto const to_month = month(tokens[i + 1U]);
        auto const to_day = parse_u32(tokens[i + 2U]);
        if (!to_year.has_value() ||
            *to_year > std::numeric_limits<std::uint16_t>::max() ||
            to_month == 0U || !to_day.has_value() || *to_day == 0U ||
            *to_day > 31U) {
          return false;
        }
        range.to_.year_ = static_cast<std::uint16_t>(*to_year);
        range.to_.month_ = to_month;
        range.to_.day_ = static_cast<std::uint8_t>(*to_day);
        i += 2U;
      } else {
        auto const to_day = parse_u32(after_dash);
        if (!to_day.has_value() || *to_day == 0U || *to_day > 31U) {
          return false;
        }
        range.to_.day_ = static_cast<std::uint8_t>(*to_day);
      }
    } else {
      auto const to_month = month(after_dash);
      if (to_month == 0U || i + 1U >= tokens.size()) {
        return false;
      }
      auto const to_day = parse_u32(tokens[i + 1U]);
      if (!to_day.has_value() || *to_day == 0U || *to_day > 31U) {
        return false;
      }
      range.to_.month_ = to_month;
      range.to_.day_ = static_cast<std::uint8_t>(*to_day);
      ++i;
    }
  }

  push_ranged(rule.monthdays_, builder.routing_.opening_hours_monthday_ranges_,
              range);
  return true;
}

bool has_unsupported_opening_hours_token(std::string_view s) {
  auto const lower = to_lower(s);
  return lower.find("\"") != std::string::npos ||
         lower.find(" ph") != std::string::npos || lower == "ph" ||
         lower.find(" sh") != std::string::npos || lower == "sh" ||
         lower.find("easter") != std::string::npos ||
         lower.find("sunrise") != std::string::npos ||
         lower.find("sunset") != std::string::npos ||
         lower.find("dawn") != std::string::npos ||
         lower.find("dusk") != std::string::npos;
}

bool looks_like_opening_hours(std::string_view s) {
  auto const lower = to_lower(s);
  if (lower.find(':') != std::string::npos ||
      lower.find("24/7") != std::string::npos) {
    return true;
  }
  for (auto const token : split_ws(s)) {
    auto const part = strip_suffix_char(token, ',');
    auto const part_lower = to_lower(part);
    auto const looks_like_year = part.size() >= 4U && is_digit(part[0]) &&
                                 is_digit(part[1]) && is_digit(part[2]) &&
                                 is_digit(part[3]);
    if (part_lower == "week" || is_weekday_selector(part) ||
        month(part) != 0U || looks_like_year) {
      return true;
    }
  }
  return false;
}

bool parse_opening_hours_rule(std::string_view text,
                              conditional_storage_builder& builder) {
  text = trim(text);
  if (text.empty() || has_unsupported_opening_hours_token(text)) {
    return false;
  }

  auto rule = opening_hours_rule{};
  // Zero the padding too (all members default to 0 / kOpen == 0) so the
  // serialized output is byte-reproducible.
  std::memset(&rule, 0, sizeof(rule));
  auto parsed_any = false;
  auto tokens = split_ws(text);
  if (tokens.size() == 1U && tokens.front() == "24/7"sv) {
    push_ranged(
        rule.times_, builder.routing_.opening_hours_time_spans_,
        opening_hours_time_span{.from_minutes_ = 0U, .to_minutes_ = 24U * 60U});
    parsed_any = true;
  }

  for (auto i = std::size_t{0U}; i != tokens.size(); ++i) {
    auto const token = tokens[i];
    auto const lower = to_lower(token);
    if (lower == "open" || lower == "on") {
      rule.modifier_ = opening_hours_rule_modifier::kOpen;
      continue;
    }
    if (lower == "off" || lower == "closed") {
      rule.modifier_ = opening_hours_rule_modifier::kClosed;
      continue;
    }
    if (lower == "unknown") {
      rule.modifier_ = opening_hours_rule_modifier::kUnknown;
      continue;
    }
    if (token == "24/7"sv) {
      continue;
    }
    if (lower == "week") {
      if (i + 1U >= tokens.size() ||
          !parse_week_token(tokens[++i], rule, builder)) {
        return false;
      }
      parsed_any = true;
      continue;
    }
    if (token.find(':') != std::string_view::npos) {
      if (!parse_time_token(token, rule, builder)) {
        return false;
      }
      parsed_any = true;
      continue;
    }
    if (is_weekday_selector(token)) {
      if (!parse_weekday_token(token, rule, builder)) {
        return false;
      }
      parsed_any = true;
      continue;
    }
    if (month(token) != 0U) {
      if (!parse_month_token(tokens, i, rule, builder)) {
        return false;
      }
      parsed_any = true;
      continue;
    }
    if (parse_year_token(token, rule, builder)) {
      parsed_any = true;
      continue;
    }
    return false;
  }

  if (!parsed_any) {
    return false;
  }
  builder.routing_.opening_hours_rules_.push_back(rule);
  return true;
}

std::optional<std::uint32_t> parse_opening_hours(
    std::string_view text, conditional_storage_builder& builder) {
  auto const oh_idx =
      static_cast<std::uint32_t>(builder.routing_.opening_hours_.size());
  builder.routing_.opening_hours_.push_back(opening_hours{});
  auto& oh = builder.routing_.opening_hours_[oh_idx];
  oh.rules_.begin_ =
      static_cast<std::uint32_t>(builder.routing_.opening_hours_rules_.size());
  for (auto const rule : split(text, ';')) {
    if (!parse_opening_hours_rule(rule, builder)) {
      return std::nullopt;
    }
  }
  oh.rules_.end_ =
      static_cast<std::uint32_t>(builder.routing_.opening_hours_rules_.size());
  return oh_idx;
}

std::optional<conditional_vehicle_property> parse_vehicle_property(
    std::string_view token) {
  auto const lower = to_lower(trim(token));
  switch (cista::hash(lower)) {
    case cista::hash("weight"): return conditional_vehicle_property::kWeight;
    case cista::hash("weight_rating"):
    case cista::hash("weightrating"):
      return conditional_vehicle_property::kWeightRating;
    case cista::hash("length"): return conditional_vehicle_property::kLength;
    case cista::hash("width"): return conditional_vehicle_property::kWidth;
    case cista::hash("height"): return conditional_vehicle_property::kHeight;
    case cista::hash("axleload"):
    case cista::hash("axle_load"):
      return conditional_vehicle_property::kAxleLoad;
    case cista::hash("axles"): return conditional_vehicle_property::kAxles;
    default: return std::nullopt;
  }
}

std::optional<conditional_symbolic_condition> parse_symbolic(
    std::string_view token) {
  auto const lower = to_lower(trim(token));
  switch (cista::hash(lower)) {
    case cista::hash("destination"):
      return conditional_symbolic_condition::kDestination;
    case cista::hash("delivery"):
      return conditional_symbolic_condition::kDelivery;
    case cista::hash("private"):
      return conditional_symbolic_condition::kPrivate;
    case cista::hash("hazmat"): return conditional_symbolic_condition::kHazmat;
    case cista::hash("hazmat:water"):
      return conditional_symbolic_condition::kHazmatWater;
    case cista::hash("trailer"):
      return conditional_symbolic_condition::kTrailer;
    default: return std::nullopt;
  }
}

bool is_access_purpose_condition(conditional_symbolic_condition const value) {
  switch (value) {
    case conditional_symbolic_condition::kDestination: [[fallthrough]];
    case conditional_symbolic_condition::kDelivery: [[fallthrough]];
    case conditional_symbolic_condition::kPrivate: return true;
    case conditional_symbolic_condition::kHazmat: [[fallthrough]];
    case conditional_symbolic_condition::kHazmatWater: [[fallthrough]];
    case conditional_symbolic_condition::kTrailer: return false;
  }
  return false;
}

std::optional<conditional_comparison> parse_comparison(std::string_view op) {
  if (op == "="sv || op == "=="sv) {
    return conditional_comparison::kEqual;
  }
  if (op == "<"sv) {
    return conditional_comparison::kLess;
  }
  if (op == "<="sv) {
    return conditional_comparison::kLessEqual;
  }
  if (op == ">"sv) {
    return conditional_comparison::kGreater;
  }
  if (op == ">="sv) {
    return conditional_comparison::kGreaterEqual;
  }
  return std::nullopt;
}

parse_result parse_vehicle_condition(std::string_view text,
                                     conditional_condition& condition) {
  for (auto const op : {">="sv, "<="sv, "=="sv, ">"sv, "<"sv, "="sv}) {
    auto const pos = text.find(op);
    if (pos == std::string_view::npos) {
      continue;
    }
    auto const property = parse_vehicle_property(text.substr(0U, pos));
    auto const comparison = parse_comparison(op);
    if (!comparison.has_value()) {
      return parse_result::kError;
    }
    if (!property.has_value()) {
      return parse_result::kIgnored;
    }

    auto const value_text = text.substr(pos + op.size());
    auto value = std::optional<conditional_numeric_value>{};
    switch (*property) {
      case conditional_vehicle_property::kWeight: [[fallthrough]];
      case conditional_vehicle_property::kWeightRating: [[fallthrough]];
      case conditional_vehicle_property::kAxleLoad:
        value = conditional_numeric_value{
            .value_ =
                to_integer<std::uint32_t>(parse_weight_t(value_text), 10.0)
                    .value_or(0U),
            .unit_ = conditional_numeric_unit::kWeight100Kg};
        break;
      case conditional_vehicle_property::kLength: [[fallthrough]];
      case conditional_vehicle_property::kWidth: [[fallthrough]];
      case conditional_vehicle_property::kHeight:
        value = conditional_numeric_value{
            .value_ =
                to_integer<std::uint32_t>(parse_length_m(value_text), 100.0)
                    .value_or(0U),
            .unit_ = conditional_numeric_unit::kCentimeter};
        break;
      default:
        value = conditional_numeric_value{
            .value_ = to_integer<std::uint32_t>(parse_double(value_text))
                          .value_or(0U),
            .unit_ = conditional_numeric_unit::kUnitless};
        break;
    }
    if (value->value_ == 0U && trim(value_text) != "0"sv) {
      return parse_result::kError;
    }
    condition = conditional_condition{
        .value_ = *value,
        .selector_ = static_cast<std::uint32_t>(*property),
        .type_ = conditional_condition_type::kVehicleProperty,
        .comparison_ = *comparison};
    return parse_result::kAdded;
  }
  return parse_result::kIgnored;
}

parse_result append_condition_piece(std::string_view text,
                                    conditional_storage_builder& builder) {
  text = trim(text);
  if (text.empty()) {
    return parse_result::kError;
  }
  if (has_unsupported_opening_hours_token(text)) {
    return parse_result::kIgnored;
  }

  if (looks_like_opening_hours(text)) {
    if (auto const oh = parse_opening_hours(text, builder); oh.has_value()) {
      builder.routing_.conditional_conditions_.push_back(conditional_condition{
          .selector_ = *oh,
          .type_ = conditional_condition_type::kOpeningHours});
      return parse_result::kAdded;
    }
    return parse_result::kError;
  }

  auto condition = conditional_condition{};
  switch (parse_vehicle_condition(text, condition)) {
    case parse_result::kAdded:
      builder.routing_.conditional_conditions_.push_back(condition);
      return parse_result::kAdded;
    case parse_result::kIgnored: break;
    case parse_result::kError: return parse_result::kError;
  }

  if (auto const symbolic = parse_symbolic(text); symbolic.has_value()) {
    auto const access_purpose = is_access_purpose_condition(*symbolic);
    builder.routing_.conditional_conditions_.push_back(conditional_condition{
        .selector_ = static_cast<std::uint32_t>(*symbolic),
        .type_ = access_purpose ? conditional_condition_type::kAccessPurpose
                                : conditional_condition_type::kVehicleUsage});
    return parse_result::kAdded;
  }

  return parse_result::kIgnored;
}

struct condition_set_result {
  parse_result result_{parse_result::kIgnored};
  conditional_condition_set_idx_t idx_{
      conditional_condition_set_idx_t::invalid()};
};

condition_set_result append_condition_set(
    std::string_view text, conditional_storage_builder& builder) {
  text = trim(text);
  auto set = conditional_condition_set{};
  set.timezone_ = builder.timezone_;
  auto added = false;
  set.conditions_.begin_ = static_cast<std::uint32_t>(
      builder.routing_.conditional_conditions_.size());
  for (auto const piece : split_condition_and(text)) {
    switch (append_condition_piece(piece, builder)) {
      case parse_result::kAdded: added = true; break;
      case parse_result::kIgnored:
        return condition_set_result{.result_ = parse_result::kIgnored};
      case parse_result::kError:
        return condition_set_result{.result_ = parse_result::kError};
    }
  }
  set.conditions_.end_ = static_cast<std::uint32_t>(
      builder.routing_.conditional_conditions_.size());
  if (!added) {
    return condition_set_result{.result_ = parse_result::kIgnored};
  }
  auto const idx = conditional_condition_set_idx_t{static_cast<std::uint32_t>(
      builder.routing_.conditional_condition_sets_.size())};
  builder.routing_.conditional_condition_sets_.push_back(set);
  return condition_set_result{.result_ = parse_result::kAdded, .idx_ = idx};
}

struct snapshot {
  way_conditional_restrictions way_{};
  std::size_t access_{};
  std::size_t oneway_{};
  std::size_t numeric_{};
  std::size_t condition_sets_{};
  std::size_t conditions_{};
  std::size_t opening_hours_{};
  std::size_t opening_hours_rules_{};
  std::size_t year_ranges_{};
  std::size_t week_ranges_{};
  std::size_t monthday_ranges_{};
  std::size_t weekday_ranges_{};
  std::size_t time_spans_{};
};

snapshot make_snapshot(conditional_storage_builder const& builder) {
  auto const& r = builder.routing_;
  return snapshot{.way_ = builder.way_,
                  .access_ = r.conditional_access_.size(),
                  .oneway_ = r.conditional_oneway_.size(),
                  .numeric_ = r.conditional_numeric_.size(),
                  .condition_sets_ = r.conditional_condition_sets_.size(),
                  .conditions_ = r.conditional_conditions_.size(),
                  .opening_hours_ = r.opening_hours_.size(),
                  .opening_hours_rules_ = r.opening_hours_rules_.size(),
                  .year_ranges_ = r.opening_hours_year_ranges_.size(),
                  .week_ranges_ = r.opening_hours_week_ranges_.size(),
                  .monthday_ranges_ = r.opening_hours_monthday_ranges_.size(),
                  .weekday_ranges_ = r.opening_hours_weekday_ranges_.size(),
                  .time_spans_ = r.opening_hours_time_spans_.size()};
}

void restore(snapshot const& s, conditional_storage_builder& builder) {
  auto& r = builder.routing_;
  builder.way_ = s.way_;
  r.conditional_access_.resize(static_cast<std::uint32_t>(s.access_));
  r.conditional_oneway_.resize(static_cast<std::uint32_t>(s.oneway_));
  r.conditional_numeric_.resize(static_cast<std::uint32_t>(s.numeric_));
  r.conditional_condition_sets_.resize(
      static_cast<std::uint32_t>(s.condition_sets_));
  r.conditional_conditions_.resize(static_cast<std::uint32_t>(s.conditions_));
  r.opening_hours_.resize(static_cast<std::uint32_t>(s.opening_hours_));
  r.opening_hours_rules_.resize(
      static_cast<std::uint32_t>(s.opening_hours_rules_));
  r.opening_hours_year_ranges_.resize(
      static_cast<std::uint32_t>(s.year_ranges_));
  r.opening_hours_week_ranges_.resize(
      static_cast<std::uint32_t>(s.week_ranges_));
  r.opening_hours_monthday_ranges_.resize(
      static_cast<std::uint32_t>(s.monthday_ranges_));
  r.opening_hours_weekday_ranges_.resize(
      static_cast<std::uint32_t>(s.weekday_ranges_));
  r.opening_hours_time_spans_.resize(static_cast<std::uint32_t>(s.time_spans_));
}

parse_result append_clause(conditional_key const& key,
                           std::string_view value,
                           std::string_view condition,
                           conditional_storage_builder& builder) {
  auto const condition_set = append_condition_set(condition, builder);
  if (condition_set.result_ != parse_result::kAdded) {
    return condition_set.result_;
  }
  switch (key.kind_) {
    case restriction_kind::kAccess: {
      auto const access = parse_access_value(value);
      if (!access.has_value()) {
        return parse_result::kError;
      }
      push_ranged(
          builder.way_.access_, builder.routing_.conditional_access_,
          conditional_access_restriction{.value_ = *access,
                                         .condition_set_ = condition_set.idx_,
                                         .field_ = key.field_,
                                         .mode_ = key.mode_,
                                         .direction_ = key.direction_});
      return parse_result::kAdded;
    }
    case restriction_kind::kOneway: {
      auto const oneway = parse_oneway_value(value);
      if (!oneway.has_value()) {
        return parse_result::kError;
      }
      push_ranged(
          builder.way_.oneway_, builder.routing_.conditional_oneway_,
          conditional_oneway_restriction{.value_ = *oneway,
                                         .condition_set_ = condition_set.idx_,
                                         .field_ = key.field_,
                                         .mode_ = key.mode_,
                                         .direction_ = key.direction_});
      return parse_result::kAdded;
    }
    case restriction_kind::kNumeric: {
      auto const numeric = parse_numeric_value(key.field_, value);
      if (!numeric.has_value()) {
        return parse_result::kError;
      }
      push_ranged(
          builder.way_.numeric_, builder.routing_.conditional_numeric_,
          conditional_numeric_restriction{.value_ = *numeric,
                                          .condition_set_ = condition_set.idx_,
                                          .field_ = key.field_,
                                          .mode_ = key.mode_,
                                          .direction_ = key.direction_});
      return parse_result::kAdded;
    }
  }
  return parse_result::kError;
}

parse_result parse_clause(conditional_key const& key,
                          std::string_view text,
                          conditional_storage_builder& builder) {
  auto const at = text.find('@');
  if (at == std::string_view::npos) {
    return parse_result::kError;
  }
  auto const value = trim(text.substr(0U, at));
  auto condition = trim(text.substr(at + 1U));
  if (value.empty() || condition.empty()) {
    return parse_result::kError;
  }
  if (condition.starts_with('(') && condition.ends_with(')')) {
    condition.remove_prefix(1U);
    condition.remove_suffix(1U);
  }
  return append_clause(key, value, condition, builder);
}

}  // namespace

bool parse_conditional_restriction_tag(std::string_view key,
                                       std::string_view value,
                                       conditional_storage_builder& builder) {
  auto const parsed_key = parse_conditional_key(trim(key));
  if (!parsed_key.has_value()) {
    return false;
  }

  auto const before = make_snapshot(builder);
  auto added = false;
  for (auto const clause : split_clauses(value)) {
    auto const clause_before = make_snapshot(builder);
    if (clause.empty()) {
      restore(before, builder);
      return false;
    }
    switch (parse_clause(*parsed_key, clause, builder)) {
      case parse_result::kAdded: added = true; break;
      case parse_result::kIgnored: restore(clause_before, builder); break;
      case parse_result::kError: restore(before, builder); return false;
    }
  }
  if (!added) {
    restore(before, builder);
  }
  return added;
}

std::optional<conditional_condition_set_idx_t> parse_conditional_condition_set(
    std::string_view condition, conditional_storage_builder& builder) {
  condition = trim(condition);
  if (condition.starts_with('(') && condition.ends_with(')')) {
    condition.remove_prefix(1U);
    condition.remove_suffix(1U);
  }

  auto const before = make_snapshot(builder);
  auto const condition_set = append_condition_set(condition, builder);
  if (condition_set.result_ != parse_result::kAdded) {
    restore(before, builder);
    return std::nullopt;
  }
  return condition_set.idx_;
}

}  // namespace osr
