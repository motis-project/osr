#include <algorithm>
#include <array>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "boost/json.hpp"

#include "fmt/ranges.h"
#include "fmt/std.h"

#include "utl/pairwise.h"
#include "utl/pipes.h"

#include "osr/platforms.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

namespace osr {

inline boost::json::array to_array(point const p) { return {p.lng(), p.lat()}; }

inline boost::json::array to_array(geo::latlng const& p) {
  return {p.lng(), p.lat()};
}

template <typename Collection>
boost::json::value to_line_string(Collection const& line) {
  auto x = boost::json::array{};
  for (auto const& p : line) {
    x.emplace_back(boost::json::array{p.lng(), p.lat()});
  }
  return {{"type", "LineString"}, {"coordinates", x}};
}

template <typename T>
boost::json::value to_line_string(std::initializer_list<T>&& line) {
  return to_line_string(line);
}

inline boost::json::object to_featurecollection_value(
    ways const& w,
    std::optional<osr::path> const& p,
    bool const with_properties = true) {
  return boost::json::object{
      {"type", "FeatureCollection"},
      {"metadata", with_properties
                       ? boost::json::value{{"duration", p->duration_.count()},
                                            {"distance", p->dist_}}
                       : boost::json::value{{}}},
      {"features",
       utl::all(p->segments_) | utl::transform([&](const path::segment& s) {
         return boost::json::object{
             {"type", "Feature"},
             {
                 "properties",
                 {{"level", s.from_level_.to_float()},
                  {"osm_way_id", s.way_ == way_idx_t::invalid()
                                     ? 0U
                                     : to_idx(w.way_osm_idx_[s.way_])},
                  {"cost", s.cost_},
                  {"distance", s.dist_}},
             },
             {"geometry", to_line_string(s.polyline_)}};
       }) | utl::emplace_back_to<boost::json::array>()}};
}

inline std::string to_featurecollection(ways const& w,
                                        std::optional<osr::path> const& p,
                                        bool const with_properties = true) {
  return boost::json::serialize(
      to_featurecollection_value(w, p, with_properties));
}

inline boost::json::value to_point(point const p) {
  return {{"type", "Point"}, {"coordinates", to_array(p)}};
}

inline std::string platform_names(platforms const& pl, platform_idx_t const i) {
  auto names = std::stringstream{};
  for (auto j = 0U; j != pl.platform_names_[i].size(); ++j) {
    names << pl.platform_names_.at(i, j).view() << "\n";
  }
  return names.str();
}

inline char const* to_string(access_value const value) {
  switch (value) {
    case access_value::kUnknown: return "unknown";
    case access_value::kYes: return "yes";
    case access_value::kDesignated: return "designated";
    case access_value::kPermissive: return "permissive";
    case access_value::kPrivate: return "private";
    case access_value::kDelivery: return "delivery";
    case access_value::kDestination: return "destination";
    case access_value::kNo: return "no";
    case access_value::kDiscouraged: return "discouraged";
  }
  return "unknown";
}

inline char const* to_string(conditional_restriction_field const value) {
  switch (value) {
    case conditional_restriction_field::kAccess: return "access";
    case conditional_restriction_field::kOneway: return "oneway";
    case conditional_restriction_field::kMaxSpeed: return "maxspeed";
    case conditional_restriction_field::kMaxLength: return "maxlength";
    case conditional_restriction_field::kMaxWeightRating:
      return "maxweightrating";
    case conditional_restriction_field::kMaxHeight: return "maxheight";
    case conditional_restriction_field::kMaxWidth: return "maxwidth";
    case conditional_restriction_field::kMaxWeight: return "maxweight";
    case conditional_restriction_field::kMaxAxleLoad: return "maxaxleload";
    case conditional_restriction_field::kMaxAxles: return "maxaxles";
    case conditional_restriction_field::kHazmat: return "hazmat";
    case conditional_restriction_field::kHazmatWater: return "hazmat:water";
    case conditional_restriction_field::kTrailer: return "trailer";
  }
  return "other";
}

inline char const* to_string(conditional_transport_mode const value) {
  switch (value) {
    case conditional_transport_mode::kUnspecified: return "";
    case conditional_transport_mode::kAccess: return "access";
    case conditional_transport_mode::kVehicle: return "vehicle";
    case conditional_transport_mode::kMotorVehicle: return "motor_vehicle";
    case conditional_transport_mode::kMotorcar: return "motorcar";
    case conditional_transport_mode::kHgv: return "hgv";
    case conditional_transport_mode::kBus: return "bus";
    case conditional_transport_mode::kPsv: return "psv";
    case conditional_transport_mode::kFoot: return "foot";
    case conditional_transport_mode::kBicycle: return "bicycle";
  }
  return "other";
}

inline char const* to_string(conditional_osm_direction const value) {
  switch (value) {
    case conditional_osm_direction::kNone: return "";
    case conditional_osm_direction::kForward: return "forward";
    case conditional_osm_direction::kBackward: return "backward";
  }
  return "";
}

inline char const* to_string(conditional_oneway_value const value) {
  switch (value) {
    case conditional_oneway_value::kNo: return "no";
    case conditional_oneway_value::kForward: return "yes";
    case conditional_oneway_value::kBackward: return "-1";
  }
  return "no";
}

inline char const* to_string(conditional_comparison const value) {
  switch (value) {
    case conditional_comparison::kNone: return "";
    case conditional_comparison::kEqual: return "=";
    case conditional_comparison::kLess: return "<";
    case conditional_comparison::kLessEqual: return "<=";
    case conditional_comparison::kGreater: return ">";
    case conditional_comparison::kGreaterEqual: return ">=";
  }
  return "";
}

inline char const* to_string(conditional_vehicle_property const value) {
  switch (value) {
    case conditional_vehicle_property::kWeight: return "weight";
    case conditional_vehicle_property::kWeightRating: return "weight_rating";
    case conditional_vehicle_property::kLength: return "length";
    case conditional_vehicle_property::kWidth: return "width";
    case conditional_vehicle_property::kHeight: return "height";
    case conditional_vehicle_property::kAxleLoad: return "axleload";
    case conditional_vehicle_property::kAxles: return "axles";
  }
  return "other";
}

inline char const* to_string(conditional_symbolic_condition const value) {
  switch (value) {
    case conditional_symbolic_condition::kDestination: return "destination";
    case conditional_symbolic_condition::kDelivery: return "delivery";
    case conditional_symbolic_condition::kPrivate: return "private";
    case conditional_symbolic_condition::kHazmat: return "hazmat";
    case conditional_symbolic_condition::kHazmatWater: return "hazmat:water";
    case conditional_symbolic_condition::kTrailer: return "trailer";
  }
  return "other";
}

inline std::string to_string(conditional_numeric_value const& value) {
  switch (value.state_) {
    case conditional_numeric_state::kNone: return "none";
    case conditional_numeric_state::kValue: break;
  }

  switch (value.unit_) {
    case conditional_numeric_unit::kUnitless:
      return fmt::format("{}", value.value_);
    case conditional_numeric_unit::kCentimeter:
      return fmt::format("{} m", value.value_ / static_cast<double>(100U));
    case conditional_numeric_unit::kKilometerPerHour:
      return fmt::format("{} km/h", value.value_);
    case conditional_numeric_unit::kWeight100Kg:
      return fmt::format("{} t", value.value_ / static_cast<double>(10U));
  }
  return fmt::format("{}", value.value_);
}

inline std::string string_value(ways const& w, string_idx_t const idx) {
  return idx == string_idx_t::invalid() ? ""
                                        : std::string{w.strings_[idx].view()};
}

inline char const* month_name(std::uint8_t const month) {
  constexpr auto const kMonths =
      std::array{"",    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                 "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
  return month < std::size(kMonths) ? kMonths[month] : "";
}

inline char const* weekday_name(std::uint8_t const weekday) {
  constexpr auto const kWeekdays =
      std::array{"", "Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"};
  return weekday < std::size(kWeekdays) ? kWeekdays[weekday] : "";
}

inline std::string to_string(opening_hours_time_span const& span) {
  auto const time = [](std::uint16_t const minutes) {
    return fmt::format("{:02}:{:02}", minutes / 60U, minutes % 60U);
  };
  return fmt::format("{}-{}", time(span.from_minutes_), time(span.to_minutes_));
}

inline std::string to_string(opening_hours_weekday_range const& range) {
  auto const nth = [](std::int8_t const value) {
    return value == 0 ? std::string{} : fmt::format("[{}]", value);
  };
  auto const from =
      fmt::format("{}{}", weekday_name(range.from_), nth(range.nth_from_));
  auto const to =
      fmt::format("{}{}", weekday_name(range.to_), nth(range.nth_to_));
  return from == to ? from : fmt::format("{}-{}", from, to);
}

inline std::string to_string(opening_hours_monthday_range const& range) {
  auto const date = [](opening_hours_date const& d) {
    if (d.year_ != 0U && d.month_ != 0U && d.day_ != 0U) {
      return fmt::format("{:04}-{:02}-{:02}", d.year_, d.month_, d.day_);
    }
    if (d.month_ != 0U && d.day_ != 0U) {
      return fmt::format("{} {:02}", month_name(d.month_), d.day_);
    }
    if (d.month_ != 0U) {
      return std::string{month_name(d.month_)};
    }
    if (d.year_ != 0U) {
      return fmt::format("{}", d.year_);
    }
    return std::string{"*"};
  };
  auto const from = date(range.from_);
  auto const to = date(range.to_);
  return from == to ? from : fmt::format("{}-{}", from, to);
}

template <typename T, typename Fn>
void append_range(std::stringstream& ss,
                  char const* prefix,
                  conditional_range const range,
                  T const& values,
                  Fn&& format) {
  if (range.empty()) {
    return;
  }
  if (ss.tellp() != std::stringstream::pos_type{0}) {
    ss << " ";
  }
  ss << prefix;
  for (auto i = range.begin_; i != range.end_; ++i) {
    if (i != range.begin_) {
      ss << ",";
    }
    ss << format(values[i]);
  }
}

inline std::string opening_hours_to_string(ways::routing const& r,
                                           std::uint32_t const idx) {
  auto ss = std::stringstream{};
  auto const& oh = r.opening_hours_[idx];
  for (auto i = oh.rules_.begin_; i != oh.rules_.end_; ++i) {
    if (i != oh.rules_.begin_) {
      ss << "; ";
    }
    auto rule_ss = std::stringstream{};
    auto const& rule = r.opening_hours_rules_[i];
    append_range(
        rule_ss, "", rule.years_, r.opening_hours_year_ranges_,
        [](opening_hours_year_range const& range) {
          auto s = range.from_ == range.to_
                       ? fmt::format("{}", range.from_)
                       : fmt::format("{}-{}", range.from_, range.to_);
          return range.step_ == 1U ? s : fmt::format("{}/{}", s, range.step_);
        });
    append_range(
        rule_ss, "week ", rule.weeks_, r.opening_hours_week_ranges_,
        [](opening_hours_week_range const& range) {
          auto s = range.from_ == range.to_
                       ? fmt::format("{}", range.from_)
                       : fmt::format("{}-{}", range.from_, range.to_);
          return range.step_ == 1U ? s : fmt::format("{}/{}", s, range.step_);
        });
    append_range(rule_ss, "", rule.monthdays_, r.opening_hours_monthday_ranges_,
                 [](opening_hours_monthday_range const& range) {
                   return to_string(range);
                 });
    append_range(rule_ss, "", rule.weekdays_, r.opening_hours_weekday_ranges_,
                 [](opening_hours_weekday_range const& range) {
                   return to_string(range);
                 });
    append_range(
        rule_ss, "", rule.times_, r.opening_hours_time_spans_,
        [](opening_hours_time_span const& span) { return to_string(span); });
    if (rule.modifier_ == opening_hours_rule_modifier::kClosed) {
      rule_ss << " closed";
    } else if (rule.modifier_ == opening_hours_rule_modifier::kUnknown) {
      rule_ss << " unknown";
    }
    ss << rule_ss.str();
  }
  return ss.str();
}

inline std::string condition_to_string(ways const& w,
                                       conditional_condition const& c) {
  switch (c.type_) {
    case conditional_condition_type::kOpeningHours:
      return opening_hours_to_string(*w.r_, c.selector_);
    case conditional_condition_type::kVehicleProperty:
      return fmt::format(
          "{} {} {}",
          to_string(static_cast<conditional_vehicle_property>(c.selector_)),
          to_string(c.comparison_), to_string(c.value_));
    case conditional_condition_type::kAccessPurpose: [[fallthrough]];
    case conditional_condition_type::kVehicleUsage:
      return to_string(
          static_cast<conditional_symbolic_condition>(c.selector_));
  }
  return {};
}

inline std::string condition_set_to_string(
    ways const& w, conditional_condition_set_idx_t const idx) {
  auto const& set = w.r_->conditional_condition_sets_[to_idx(idx)];
  auto ss = std::stringstream{};
  for (auto i = set.conditions_.begin_; i != set.conditions_.end_; ++i) {
    if (i != set.conditions_.begin_) {
      ss << " AND ";
    }
    ss << condition_to_string(w, w.r_->conditional_conditions_[i]);
  }
  return ss.str();
}

inline std::string restriction_mode_string(restriction const& r) {
  auto s = std::string{};
  s.reserve(3U);
  s.push_back(r.applies_to_default_ ? 'D' : '-');
  s.push_back(r.applies_to_bus_ ? 'B' : '-');
  s.push_back(r.applies_to_hgv_ ? 'H' : '-');
  return s;
}

inline std::string restriction_to_string(ways const& w,
                                         node_idx_t const node,
                                         restriction const& r) {
  auto const from_way = w.way_osm_idx_[w.r_->node_ways_[node][r.from_]];
  auto const to_way = w.way_osm_idx_[w.r_->node_ways_[node][r.to_]];
  auto out = fmt::format("({}, {}, {})", to_idx(from_way), to_idx(to_way),
                         restriction_mode_string(r));
  if (r.condition_set_ != conditional_condition_set_idx_t::invalid()) {
    out.append(
        fmt::format(" @ ({})", condition_set_to_string(w, r.condition_set_)));
  }
  return out;
}

template <typename Restriction>
std::string conditional_key(Restriction const& r) {
  auto parts = std::vector<std::string>{};
  if (r.field_ == conditional_restriction_field::kAccess) {
    if (r.mode_ == conditional_transport_mode::kUnspecified ||
        r.mode_ == conditional_transport_mode::kAccess) {
      parts.emplace_back("access");
    } else {
      parts.emplace_back(to_string(r.mode_));
    }
  } else if (r.field_ == conditional_restriction_field::kHazmat) {
    parts.emplace_back("hazmat");
  } else if (r.field_ == conditional_restriction_field::kHazmatWater) {
    parts.emplace_back("hazmat");
    parts.emplace_back("water");
  } else if (r.field_ == conditional_restriction_field::kTrailer) {
    parts.emplace_back("hgv");
    parts.emplace_back("trailer");
  } else {
    parts.emplace_back(to_string(r.field_));
    if (r.mode_ == conditional_transport_mode::kHgv) {
      parts.emplace_back("hgv");
    }
  }
  if (r.direction_ != conditional_osm_direction::kNone) {
    parts.emplace_back(to_string(r.direction_));
  }
  parts.emplace_back("conditional");
  return fmt::format("{}", fmt::join(parts, ":"));
}

template <typename Restriction>
std::string conditional_to_string(ways const& w,
                                  Restriction const& r,
                                  std::string const& value) {
  return fmt::format("{} = {} @ ({})", conditional_key(r), value,
                     condition_set_to_string(w, r.condition_set_));
}

inline std::optional<std::string_view> tz_name(
    ways::routing const& r, conditional_timezone_idx_t const idx) {
  return idx == conditional_timezone_idx_t::invalid()
             ? std::optional<std::string_view>{}
             : std::optional{r.timezones_.at(idx).view()};
}

inline void add_conditional_properties(boost::json::object& properties,
                                       ways const& w,
                                       way_idx_t const way,
                                       way_properties const& way_props) {
  properties["has_conditionals"] = way_props.has_conditionals();
  if (!way_props.has_conditionals()) {
    return;
  }

  auto const* conditionals = w.r_->get_conditional_restrictions(way);
  if (conditionals == nullptr) {
    return;
  }

  auto clauses = boost::json::array{};
  auto timezone = std::optional<std::string_view>{};
  auto append = [&](auto const range, auto const& values,
                    auto&& value_to_string) {
    for (auto i = range.begin_; i != range.end_; ++i) {
      auto const& r = values[i];
      if (!timezone.has_value() &&
          r.condition_set_ != conditional_condition_set_idx_t::invalid()) {
        timezone = tz_name(
            *w.r_, w.r_->conditional_condition_sets_[to_idx(r.condition_set_)]
                       .timezone_);
      }
      clauses.emplace_back(conditional_to_string(w, r, value_to_string(r)));
    }
  };
  append(conditionals->access_, w.r_->conditional_access_,
         [](conditional_access_restriction const& r) {
           return std::string{to_string(r.value_)};
         });
  append(conditionals->oneway_, w.r_->conditional_oneway_,
         [](conditional_oneway_restriction const& r) {
           return std::string{to_string(r.value_)};
         });
  append(conditionals->numeric_, w.r_->conditional_numeric_,
         [](conditional_numeric_restriction const& r) {
           return to_string(r.value_);
         });
  if (timezone.has_value()) {
    properties["timezone"] = *timezone;
  }
  properties["conditionals"] = std::move(clauses);
}

inline void add_hgv_way_info_properties(boost::json::object& properties,
                                        ways::routing const& r,
                                        way_idx_t const way,
                                        way_properties const& way_props) {
  properties["has_hgv_info"] = way_props.has_hgv_info();
  if (!way_props.has_hgv_info()) {
    return;
  }

  auto const* hgv_info = r.get_hgv_info(way);
  if (hgv_info == nullptr) {
    return;
  }

  if (hgv_info->has(hgv_info_field::kAccessFwd)) {
    properties["hgv_access_fwd"] = to_string(hgv_info->hgv_access_fwd());
  }
  if (hgv_info->has(hgv_info_field::kAccessBwd)) {
    properties["hgv_access_bwd"] = to_string(hgv_info->hgv_access_bwd());
  }
  if (hgv_info->has(hgv_info_field::kHazmat)) {
    properties["hazmat_access"] = to_string(hgv_info->hazmat_access());
  }
  if (hgv_info->has(hgv_info_field::kHazmatWater)) {
    properties["hazmat_water_access"] =
        to_string(hgv_info->hazmat_water_access());
  }
  if (hgv_info->has(hgv_info_field::kMaxSpeed)) {
    properties["hgv_max_speed_km_h"] = hgv_info->maxspeed_km_h_;
  }
  if (hgv_info->has(hgv_info_field::kMaxLength)) {
    properties["hgv_max_length_cm"] = hgv_info->maxlength_cm_;
  }
  if (hgv_info->has(hgv_info_field::kMaxWeightRating)) {
    properties["hgv_max_weight_rating_100kg"] =
        hgv_info->maxweightrating_100kg_;
  }
  if (hgv_info->has(hgv_info_field::kMaxHeight)) {
    properties["hgv_max_height_cm"] = hgv_info->maxheight_cm_;
  }
  if (hgv_info->has(hgv_info_field::kMaxWidth)) {
    properties["hgv_max_width_cm"] = hgv_info->maxwidth_cm_;
  }
  if (hgv_info->has(hgv_info_field::kMaxWeight)) {
    properties["hgv_max_weight_100kg"] = hgv_info->maxweight_100kg_;
  }
  if (hgv_info->has(hgv_info_field::kMaxAxleLoad)) {
    properties["hgv_max_axle_load_100kg"] = hgv_info->maxaxleload_100kg_;
  }
  if (hgv_info->has(hgv_info_field::kMaxAxles)) {
    properties["hgv_max_axles"] = hgv_info->maxaxles_;
  }
  if (hgv_info->has(hgv_info_field::kTrailer)) {
    properties["hgv_trailer_access"] = to_string(hgv_info->trailer_access());
  }
}

struct geojson_writer {
  void write_platform(platform_idx_t const i) {
    for (auto const r : platforms_->platform_ref_[i]) {
      auto const geometry = std::visit(
          utl::overloaded{[&](node_idx_t x) {
                            return to_point(platforms_->get_node_pos(x));
                          },
                          [&](way_idx_t x) {
                            return to_line_string(w_.way_polylines_[x]);
                          }},
          to_ref(r));
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties",
           {{"type", is_way(r) ? "way" : "node"},
            {"platform_idx", to_idx(i)},
            {"level", platforms_->get_level(w_, i).to_float()},
            {"names", platform_names(*platforms_, i)}}},
          {"geometry", geometry}});
    }
  }

  void write_way(way_idx_t const i) {
    auto const nodes = w_.r_->way_nodes_[i];
    auto const way_nodes = utl::nwise<2>(nodes);
    auto const dists = w_.r_->way_node_dist_[i];
    auto way_nodes_it = std::begin(way_nodes);
    auto dist_it = std::begin(dists);
    auto const p = w_.r_->way_properties_[i];
    auto n = 0U;
    for (; dist_it != end(dists); ++way_nodes_it, ++dist_it) {
      auto const& [from, to] = *way_nodes_it;
      auto const dist =
          w_.r_->get_way_node_distance(i, static_cast<std::uint16_t>(n));
      auto properties = boost::json::object{
          {"type", "edge"},
          {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
          {"internal_id", to_idx(i)},
          {"component", to_idx(w_.r_->way_component_[i])},
          {"distance", dist},
          {"car", p.is_car_accessible()},
          {"bike", p.is_bike_accessible()},
          {"foot", p.is_foot_accessible()},
          {"bus", p.is_bus_accessible()},
          {"bus_with_penalty", p.is_bus_accessible_with_penalty()},
          {"railway", p.is_railway_accessible()},
          {"railway_with_penalty", p.is_railway_accessible_with_penalty()},
          {"ferry", p.is_ferry_accessible()},
          {"low_emission_zone", p.is_in_low_emission_zone()},
          {"is_big_street", p.is_big_street()},
          {"is_destination", p.is_destination()},
          {"oneway_car", p.is_oneway_car()},
          {"oneway_bike", p.is_oneway_bike()},
          {"oneway_bus_psv", p.is_oneway_bus_psv()},
          {"oneway_reverse", p.is_oneway_reverse()},
          {"max_speed", p.max_speed_km_per_h()},
          {"speed_limit", p.speed_limit_},
          {"from_level", p.from_level().to_float()},
          {"to_level", p.to_level().to_float()},
          {"is_elevator", p.is_elevator()},
          {"sidewalk_separate", p.is_sidewalk_separate()},
          {"is_steps", p.is_steps()},
          {"is_parking", p.is_parking()},
          {"is_ramp", p.is_ramp()},
          {"in_route", p.in_route()},
          {"is_detour", p.is_detour()}};
      add_hgv_way_info_properties(properties, *w_.r_, i, p);
      add_conditional_properties(properties, w_, i, p);
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties", std::move(properties)},
          {"geometry", to_line_string(std::initializer_list<geo::latlng>{
                           w_.get_node_pos(from), w_.get_node_pos(to)})}});
    }

    auto properties = boost::json::object{
        {"type", "geometry"},
        {"osm_way_id", to_idx(w_.way_osm_idx_[i])},
        {"internal_id", to_idx(i)},
        {"component", to_idx(w_.r_->way_component_[i])},
        {"car", p.is_car_accessible()},
        {"bike", p.is_bike_accessible()},
        {"foot", p.is_foot_accessible()},
        {"bus", p.is_bus_accessible()},
        {"bus_with_penalty", p.is_bus_accessible_with_penalty()},
        {"railway", p.is_railway_accessible()},
        {"railway_with_penalty", p.is_railway_accessible_with_penalty()},
        {"ferry", p.is_ferry_accessible()},
        {"low_emission_zone", p.is_in_low_emission_zone()},
        {"is_destination", p.is_destination()},
        {"is_big_street", p.is_big_street()},
        {"oneway_car", p.is_oneway_car()},
        {"oneway_bike", p.is_oneway_bike()},
        {"oneway_bus_psv", p.is_oneway_bus_psv()},
        {"oneway_reverse", p.is_oneway_reverse()},
        {"max_speed", p.max_speed_km_per_h()},
        {"speed_limit", p.speed_limit_},
        {"from_level", p.from_level().to_float()},
        {"to_level", p.to_level().to_float()},
        {"is_elevator", p.is_elevator()},
        {"sidewalk_separate", p.is_sidewalk_separate()},
        {"is_steps", p.is_steps()},
        {"is_parking", p.is_parking()},
        {"is_ramp", p.is_ramp()},
        {"in_route", p.in_route()},
        {"is_detour", p.is_detour()}};
    add_hgv_way_info_properties(properties, *w_.r_, i, p);
    add_conditional_properties(properties, w_, i, p);

    features_.emplace_back(
        boost::json::value{{"type", "Feature"},
                           {"properties", std::move(properties)},
                           {"geometry", to_line_string(w_.way_polylines_[i])}});

    nodes_.insert(begin(nodes), end(nodes));
    ++n;
  }

  template <typename Dijkstra>
  void finish(Dijkstra const* s) {
    for (auto const n : nodes_) {
      auto const p = w_.r_->node_properties_[n];

      auto ss = std::stringstream{};
      Dijkstra::profile_t::resolve_all(*w_.r_, n, kNoLevel, [&](auto const x) {
        auto const cost = s->get_cost(x);
        if (cost != kInfeasible) {
          ss << "{";
          x.print(ss, w_);
          ss << ", " << cost << "}\n";
        }
      });

      auto levels = std::vector<float>();
      foot<true>::for_each_elevator_level(
          *w_.r_, n, [&](level_t const l) { levels.push_back(l.to_float()); });

      auto properties = boost::json::object{
          {"osm_node_id", to_idx(w_.node_to_osm_[n])},
          {"internal_id", to_idx(n)},
          {"car", p.is_car_accessible()},
          {"bike", p.is_bike_accessible()},
          {"foot", p.is_walk_accessible()},
          {"bus", p.is_bus_accessible()},
          {"bus_with_penalty", p.is_bus_accessible_with_penalty()},
          {"is_restricted", w_.r_->node_is_restricted_[n]},
          {"is_entrance", p.is_entrance()},
          {"is_elevator", p.is_elevator()},
          {"is_parking", p.is_parking()},
          {"multi_level", p.is_multi_level()},
          {"levels", boost::json::array(levels.begin(), levels.end())},
          {"ways", fmt::format("{}", w_.r_->node_ways_[n] |
                                         std::views::transform([&](auto&& w) {
                                           return w_.way_osm_idx_[w];
                                         }))},
          {"restrictions",
           fmt::format(
               "[{}]",
               fmt::join(w_.r_->node_restrictions_[n] |
                             std::views::transform([&](restriction const r) {
                               return restriction_to_string(w_, n, r);
                             }),
                         ", "))},
          {"label", ss.str().empty() ? "unreachable" : ss.str()}};
      features_.emplace_back(boost::json::value{
          {"type", "Feature"},
          {"properties", properties},
          {"geometry",
           {{"type", "Point"},
            {"coordinates", to_array(w_.get_node_pos(n))}}}});
    }
  }

  std::string string() {
    return boost::json::serialize(boost::json::value{
        {"type", "FeatureCollection"}, {"features", features_}});
  }

  boost::json::value json() {
    return boost::json::value{{"type", "FeatureCollection"},
                              {"features", features_}};
  }

  ways const& w_;
  platforms const* platforms_{nullptr};
  boost::json::array features_{};
  hash_set<node_idx_t> nodes_{};
};

}  // namespace osr
