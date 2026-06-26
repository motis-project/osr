#include "gtest/gtest.h"

#include <cstdint>
#include <string>
#include <string_view>

#include "osr/extract/conditional_parser.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/hgv.h"

osr::cost_t cost(const osr::way_properties p) {
  return osr::foot<false>::way_cost(
             {}, osr::ways::routing{}, osr::way_idx_t::invalid(), p,
             osr::direction::kForward, 1, std::nullopt, osr::duration_t{0},
             osr::direction::kForward)
      .cost_;
}

osr::way_properties hgv_way_properties() {
  return {.is_car_accessible_ = true, .speed_limit_ = osr::speed_limit::kmh_80};
}

osr::hgv_way_info hgv_info(osr::access_value const value) {
  return {.fields_ = osr::to_mask(osr::hgv_info_field::kAccess),
          .hgv_access_ = static_cast<std::uint8_t>(value)};
}

osr::hgv_way_info hgv_info(osr::hgv_info_field const field,
                           osr::access_value const value) {
  auto info = osr::hgv_way_info{.fields_ = osr::to_mask(field)};
  switch (field) {
    case osr::hgv_info_field::kHazmat:
      info.hazmat_access_ = static_cast<std::uint8_t>(value);
      break;
    case osr::hgv_info_field::kHazmatWater:
      info.hazmat_water_access_ = static_cast<std::uint8_t>(value);
      break;
    case osr::hgv_info_field::kTrailer:
      info.trailer_access_ = static_cast<std::uint8_t>(value);
      break;
    default: break;
  }
  return info;
}

osr::cost_t hgv_cost(osr::access_value const access) {
  auto const params = osr::hgv::parameters{};
  auto const props = hgv_way_properties();
  auto const info = hgv_info(access);
  return osr::hgv::distance_cost(params, props, &info, 1000U, false, false,
                                 access);
}

osr::cost_t static_hgv_cost(osr::hgv_way_info const& info,
                            osr::hgv::parameters const& params) {
  auto routing = osr::ways::routing{};
  auto const way = osr::way_idx_t{0U};
  auto props = hgv_way_properties();
  props.has_hgv_info_ = true;
  routing.way_properties_.push_back(props);
  routing.way_hgv_info_.emplace_back(way, info);

  auto const result = osr::hgv::way_cost(
      params, routing, way, props, osr::direction::kForward, 1000U,
      std::nullopt, osr::duration_t{0U}, osr::direction::kForward);
  return result.cost_;
}

osr::cost_t low_emission_zone_hgv_cost(osr::hgv::parameters const& params) {
  auto routing = osr::ways::routing{};
  auto const way = osr::way_idx_t{0U};
  auto const props = hgv_way_properties();
  routing.way_properties_.push_back(props);
  routing.way_properties_[way].is_in_low_emission_zone_ = true;

  auto const result = osr::hgv::way_cost(
      params, routing, way, props, osr::direction::kForward, 1000U,
      std::nullopt, osr::duration_t{0U}, osr::direction::kForward);
  return result.cost_;
}

osr::cost_t conditional_hgv_cost(std::string_view const key,
                                 std::string_view const value,
                                 osr::hgv::parameters const& params = {}) {
  auto routing = osr::ways::routing{};
  auto builder = osr::conditional_storage_builder{.routing_ = routing};
  auto const tag = std::string{value} + " @ (24/7)";
  EXPECT_TRUE(osr::parse_conditional_restriction_tag(key, tag, builder));

  auto const way = osr::way_idx_t{0U};
  auto props = hgv_way_properties();
  props.has_conditionals_ = true;
  routing.way_properties_.push_back(props);
  routing.way_conditionals_.emplace_back(way, builder.way_);

  auto const result = osr::hgv::way_cost(
      params, routing, way, props, osr::direction::kForward, 1000U,
      osr::routing_time_t{}, osr::duration_t{0U}, osr::direction::kForward);
  return result.cost_;
}

TEST(way_cost_foot, combinations) {
  EXPECT_EQ(osr::kInfeasible, cost({
                                  .is_foot_accessible_ = false,
                                  .is_bike_accessible_ = false,
                                  .is_sidewalk_separate_ = false,
                              }));
  EXPECT_EQ(osr::kInfeasible, cost({
                                  .is_foot_accessible_ = false,
                                  .is_bike_accessible_ = false,
                                  .is_sidewalk_separate_ = true,
                              }));
  EXPECT_EQ(91, cost({
                    .is_foot_accessible_ = false,
                    .is_bike_accessible_ = true,
                    .is_sidewalk_separate_ = false,
                }));
  EXPECT_EQ(136, cost({
                     .is_foot_accessible_ = false,
                     .is_bike_accessible_ = true,
                     .is_sidewalk_separate_ = true,
                 }));
  EXPECT_EQ(1, cost({
                   .is_foot_accessible_ = true,
                   .is_bike_accessible_ = false,
                   .is_sidewalk_separate_ = false,
               }));
  EXPECT_EQ(46, cost({
                    .is_foot_accessible_ = true,
                    .is_bike_accessible_ = false,
                    .is_sidewalk_separate_ = true,
                }));
  EXPECT_EQ(1, cost({
                   .is_foot_accessible_ = true,
                   .is_bike_accessible_ = true,
                   .is_sidewalk_separate_ = false,
               }));
  EXPECT_EQ(46, cost({
                    .is_foot_accessible_ = true,
                    .is_bike_accessible_ = true,
                    .is_sidewalk_separate_ = true,
                }));
}

TEST(way_cost_hgv, hgv_access_values_change_cost) {
  EXPECT_EQ(45U, hgv_cost(osr::access_value::kYes));
  EXPECT_EQ(36U, hgv_cost(osr::access_value::kDesignated));
  EXPECT_EQ(345U, hgv_cost(osr::access_value::kDelivery));
  EXPECT_EQ(345U, hgv_cost(osr::access_value::kDestination));
}

TEST(way_cost_hgv, hgv_access_values_change_accessibility) {
  EXPECT_TRUE(osr::hgv::access_allowed(osr::access_value::kYes));
  EXPECT_TRUE(osr::hgv::access_allowed(osr::access_value::kDesignated));
  EXPECT_TRUE(osr::hgv::access_allowed(osr::access_value::kDelivery));
  EXPECT_TRUE(osr::hgv::access_allowed(osr::access_value::kDestination));
  EXPECT_FALSE(osr::hgv::access_allowed(osr::access_value::kNo));
  EXPECT_FALSE(osr::hgv::access_allowed(osr::access_value::kDiscouraged));
}

TEST(way_cost_hgv, conditional_hgv_access_values_change_cost) {
  EXPECT_EQ(36U, conditional_hgv_cost("hgv:conditional", "designated"));
  EXPECT_EQ(345U, conditional_hgv_cost("hgv:conditional", "delivery"));
  EXPECT_EQ(345U, conditional_hgv_cost("hgv:conditional", "destination"));
}

TEST(way_cost_hgv, low_emission_zone_access_blocks_hgv) {
  EXPECT_NE(osr::kInfeasible, low_emission_zone_hgv_cost({}));
  EXPECT_EQ(osr::kInfeasible,
            low_emission_zone_hgv_cost({.low_emission_zone_access_ = false}));
}

TEST(way_cost_hgv, designated_hazmat_trailer_values_change_cost) {
  EXPECT_EQ(36U, static_hgv_cost(hgv_info(osr::hgv_info_field::kHazmat,
                                          osr::access_value::kDesignated),
                                 {.hazmat_ = true}));
  EXPECT_EQ(36U, static_hgv_cost(hgv_info(osr::hgv_info_field::kHazmatWater,
                                          osr::access_value::kDesignated),
                                 {.hazmat_water_ = true}));
  EXPECT_EQ(36U, static_hgv_cost(hgv_info(osr::hgv_info_field::kTrailer,
                                          osr::access_value::kDesignated),
                                 {.trailer_ = true}));
}

TEST(way_cost_hgv, conditional_designated_hazmat_trailer_values_change_cost) {
  EXPECT_EQ(36U, conditional_hgv_cost("hazmat:conditional", "designated",
                                      {.hazmat_ = true}));
  EXPECT_EQ(36U, conditional_hgv_cost("hazmat:water:conditional", "designated",
                                      {.hazmat_water_ = true}));
  EXPECT_EQ(36U, conditional_hgv_cost("hgv:trailer:conditional", "designated",
                                      {.trailer_ = true}));
}
