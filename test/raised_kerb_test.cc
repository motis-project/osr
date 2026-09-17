#include "gtest/gtest.h"

#include <filesystem>
#include <initializer_list>
#include <system_error>
#include <tuple>

#include "cista/mmap.h"

#include "geo/latlng.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profile.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

constexpr auto const kWayWithKerbs =
    R"(<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="osr-test">
  <node id="0" lat="0.0000" lon="0.0000"/>
  <node id="11" lat="0.0001" lon="0.0000"/>
  <node id="12" lat="0.0002" lon="0.0000">
    <tag k="barrier" v="kerb"/>
    <tag k="kerb" v="lowered"/>
  </node>
  <node id="13" lat="0.0003" lon="0.0000"/>
  <node id="14" lat="0.0004" lon="0.0000"/>

  <node id="21" lat="0.0000" lon="0.0001"/>
  <node id="22" lat="0.0000" lon="0.0002">
    <tag k="barrier" v="kerb"/>
    <tag k="kerb" v="raised"/>
  </node>
  <node id="23" lat="0.0000" lon="0.0003"/>
  <node id="24" lat="0.0000" lon="0.0004"/>

  <node id="31" lat="-0.0001" lon="0.0000"/>
  <node id="32" lat="-0.0002" lon="0.0000">
    <tag k="barrier" v="kerb"/>
    <tag k="kerb" v="rolled"/>
  </node>
  <node id="33" lat="-0.0003" lon="0.0000"/>
  <node id="34" lat="-0.0004" lon="0.0000"/>

  <node id="41" lat="0.0000" lon="-0.0001"/>
  <node id="42" lat="0.0000" lon="-0.0002">
    <tag k="barrier" v="kerb"/>
    <tag k="kerb" v="yes"/>
  </node>
  <node id="43" lat="0.0000" lon="-0.0003"/>
  <node id="44" lat="0.0000" lon="-0.0004"/>

  <way id="1">
    <nd ref="31"/>
    <nd ref="0"/>
    <nd ref="11"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="2">
    <nd ref="41"/>
    <nd ref="0"/>
    <nd ref="21"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="11">
    <nd ref="11"/>
    <nd ref="12"/>
    <nd ref="13"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="12">
    <nd ref="13"/>
    <nd ref="14"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="21">
    <nd ref="21"/>
    <nd ref="22"/>
    <nd ref="23"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="22">
    <nd ref="23"/>
    <nd ref="24"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="31">
    <nd ref="31"/>
    <nd ref="32"/>
    <nd ref="33"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="32">
    <nd ref="33"/>
    <nd ref="34"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="41">
    <nd ref="41"/>
    <nd ref="42"/>
    <nd ref="43"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="42">
    <nd ref="43"/>
    <nd ref="44"/>
    <tag k="highway" v="service"/>
  </way>
</osm>)";

TEST(routing, kerbs) {
  auto const dir = fs::temp_directory_path() / "osr_ways_with_kerbs_test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(false,
          osr::test::write_osm_pbf("osr_ways_with_kerbs", kWayWithKerbs)
              .generic_string(),
          dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const route = [&](search_profile const profile, geo::latlng const& from,
                         geo::latlng const& to) {
    // Use small matching distance to ensure only complete paths are found
    auto const max_matching_distance = 10.0;
    auto const max_cost = 900;
    return osr::route(get_parameters(profile), w, l, profile,
                      location{.pos_ = from, .lvl_ = kNoLevel},
                      {location{.pos_ = to, .lvl_ = kNoLevel}}, max_cost,
                      osr::direction::kForward, max_matching_distance, nullptr,
                      nullptr, nullptr, osr::routing_algorithm::kDijkstra);
  };
  auto const center = geo::latlng{.lat_ = 0.0, .lng_ = 0.0};
  auto const north = geo::latlng{.lat_ = 0.0004, .lng_ = 0.0};
  auto const east = geo::latlng{.lat_ = 0.0, .lng_ = 0.0004};
  auto const south = geo::latlng{.lat_ = -0.0004, .lng_ = 0.0};
  auto const west = geo::latlng{.lat_ = 0.0, .lng_ = -0.0004};
  auto const path_dist = 44.11949;

  // North: kerb=lowered: Reachable by all
  {
    for (auto const [profile, cost, dist] :
         std::initializer_list<std::tuple<search_profile, cost_t, double>>{
             {search_profile::kFoot, 36U, path_dist},
             {search_profile::kWheelchair, 55U, path_dist},
             {search_profile::kBike, 11U, path_dist},
             {search_profile::kCar, 8U, path_dist},
             {search_profile::kBus, 12U, path_dist},
             {search_profile::kHgv, 8U, path_dist},
         }) {
      auto const p = route(profile, center, north);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_NEAR(dist, p->dist_, 10e-6);
    }
  }
  // East: kerb=raised: Only reachable by foot + bike
  {
    for (auto const [profile, cost, dist] :
         std::initializer_list<std::tuple<search_profile, cost_t, double>>{
             {search_profile::kFoot, 36U, path_dist},
             {search_profile::kBike, 12U + 30U, path_dist},
         }) {
      auto const p = route(profile, center, east);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_NEAR(dist, p->dist_, 10e-6);
    }
    for (auto const profile :
         {search_profile::kWheelchair, search_profile::kCar}) {
      auto const p = route(profile, center, east);
      ASSERT_FALSE(p.has_value());
    }
  }
  // South: kerb=rolled: Not reachable by wheelchair
  {
    auto const south_dist = 44.23899;
    for (auto const [profile, cost, dist] :
         std::initializer_list<std::tuple<search_profile, cost_t, double>>{
             {search_profile::kFoot, 36U, south_dist},
             {search_profile::kBike, 12U, south_dist},
             {search_profile::kCar, 8U, south_dist},
         }) {
      auto const p = route(profile, center, south);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_NEAR(dist, p->dist_, 10e-6);
    }
    for (auto const profile : {search_profile::kWheelchair}) {
      auto const p = route(profile, center, south);
      ASSERT_FALSE(p.has_value());
    }
  }
  // West: kerb=yes: Not reachable by car
  {
    for (auto const [profile, cost, dist] :
         std::initializer_list<std::tuple<search_profile, cost_t, double>>{
             {search_profile::kFoot, 36U, path_dist},
             {search_profile::kWheelchair, 56U, path_dist},
             {search_profile::kBike, 12U + 30U, path_dist},
         }) {
      auto const p = route(profile, center, west);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_NEAR(dist, p->dist_, 10e-6);
    }
    for (auto const profile : {search_profile::kCar}) {
      auto const p = route(profile, center, west);
      ASSERT_FALSE(p.has_value());
    }
  }
}

}  // namespace
