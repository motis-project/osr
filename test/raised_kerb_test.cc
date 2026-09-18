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

// Star like ways
// North, east, south, west: Kerb on interior node => needs to be pushed
// Diagonals: Kerbs on junctions, as possibly not pushed
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

  <node id="51" lat="0.0001" lon="0.0001"/>
  <node id="52" lat="0.0002" lon="0.0002">
    <tag k="kerb" v="raised"/>
    <tag k="highway" v="bus_stop"/>
  </node>
  <node id="53" lat="0.0003" lon="0.0003"/>
  <node id="54" lat="0.0004" lon="0.0004"/>

  <node id="61" lat="-0.0001" lon="0.0001"/>
  <node id="62" lat="-0.0002" lon="0.0002">
    <tag k="kerb" v="raised"/>
    <tag k="wheelchair" v="yes"/>
  </node>
  <node id="63" lat="-0.0003" lon="0.0003"/>
  <node id="64" lat="-0.0004" lon="0.0004"/>

  <node id="71" lat="-0.0001" lon="-0.0001"/>
  <node id="72" lat="-0.0002" lon="-0.0002">
    <tag k="kerb" v="no"/>
    <tag k="wheelchair" v="no"/>
  </node>
  <node id="73" lat="-0.0003" lon="-0.0003"/>
  <node id="74" lat="-0.0004" lon="-0.0004"/>

  <node id="81" lat="0.0001" lon="-0.0001"/>
  <node id="82" lat="0.0002" lon="-0.0002">
    <tag k="kerb" v="raised"/>
    <tag k="wheelchair" v="limited"/>
  </node>
  <node id="83" lat="0.0003" lon="-0.0003"/>
  <node id="84" lat="0.0004" lon="-0.0004"/>

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
  <way id="3">
    <nd ref="71"/>
    <nd ref="0"/>
    <nd ref="51"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="4">
    <nd ref="81"/>
    <nd ref="0"/>
    <nd ref="61"/>
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
  <way id="51">
    <nd ref="51"/>
    <nd ref="52"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="52">
    <nd ref="52"/>
    <nd ref="53"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="53">
    <nd ref="53"/>
    <nd ref="54"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="61">
    <nd ref="61"/>
    <nd ref="62"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="62">
    <nd ref="62"/>
    <nd ref="63"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="63">
    <nd ref="63"/>
    <nd ref="64"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="71">
    <nd ref="71"/>
    <nd ref="72"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="72">
    <nd ref="72"/>
    <nd ref="73"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="73">
    <nd ref="73"/>
    <nd ref="74"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="81">
    <nd ref="81"/>
    <nd ref="82"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="82">
    <nd ref="82"/>
    <nd ref="83"/>
    <tag k="highway" v="service"/>
  </way>
  <way id="83">
    <nd ref="83"/>
    <nd ref="84"/>
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

  auto const center = geo::latlng{.lat_ = 0.0, .lng_ = 0.0};
  auto const north = geo::latlng{.lat_ = 0.0004, .lng_ = 0.0};
  auto const east = geo::latlng{.lat_ = 0.0, .lng_ = 0.0004};
  auto const south = geo::latlng{.lat_ = -0.0004, .lng_ = 0.0};
  auto const west = geo::latlng{.lat_ = 0.0, .lng_ = -0.0004};
  auto const north_east = geo::latlng{.lat_ = 0.0004, .lng_ = 0.0004};
  auto const south_east = geo::latlng{.lat_ = -0.0004, .lng_ = 0.0004};
  auto const south_west = geo::latlng{.lat_ = -0.0004, .lng_ = -0.0004};
  auto const north_west = geo::latlng{.lat_ = 0.0004, .lng_ = -0.0004};
  auto const min_dist = 44.0;

  auto const route = [&](search_profile const profile,
                         geo::latlng const& from) {
    // Use small matching distance to ensure only complete paths are found
    auto const max_matching_distance = 10.0;
    auto const max_cost = 900;
    return osr::route(get_parameters(profile), w, l, profile,
                      location{.pos_ = from, .lvl_ = kNoLevel},
                      {location{.pos_ = center, .lvl_ = kNoLevel}}, max_cost,
                      osr::direction::kForward, max_matching_distance, nullptr,
                      nullptr, nullptr, osr::routing_algorithm::kDijkstra);
  };

  // North: kerb=lowered: Reachable by all
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 36U},
             {search_profile::kWheelchair, 55U},
             {search_profile::kBike, 11U},
             {search_profile::kCar, 8U},
             {search_profile::kBus, 12U},
             {search_profile::kHgv, 8U},
         }) {
      auto const p = route(profile, north);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
  }
  // East: kerb=raised: Only reachable by foot + bike
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 36U},
             {search_profile::kBike, 12U + 30U},
         }) {
      auto const p = route(profile, east);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
    for (auto const profile :
         {search_profile::kWheelchair, search_profile::kCar}) {
      auto const p = route(profile, east);
      ASSERT_FALSE(p.has_value());
    }
  }
  // South: kerb=rolled: Not reachable by wheelchair
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 36U},
             {search_profile::kBike, 12U},
             {search_profile::kCar, 8U},
         }) {
      auto const p = route(profile, south);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
    for (auto const profile : {search_profile::kWheelchair}) {
      auto const p = route(profile, south);
      ASSERT_FALSE(p.has_value());
    }
  }
  // West: kerb=yes: Not reachable by car
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 36U},
             {search_profile::kWheelchair, 56U},
             {search_profile::kBike, 12U + 30U},
         }) {
      auto const p = route(profile, west);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
    for (auto const profile : {search_profile::kCar}) {
      auto const p = route(profile, west);
      ASSERT_FALSE(p.has_value());
    }
  }
  // North-East: kerb=raised + highway=bus_stop: Wheelchair allowed
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 51U},
             {search_profile::kWheelchair, 79U},
             {search_profile::kBike, 16U + 30U},
         }) {
      auto const p = route(profile, north_east);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
  }
  // South-East: kerb=raised + wheelchair=yes: Wheelchair allowed
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 51U},
             {search_profile::kWheelchair, 79U},
             {search_profile::kBike, 16U + 30U},
         }) {
      auto const p = route(profile, south_east);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
  }
  // South-West: kerb=no + wheelchair=no: Wheelchair disallowed
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 51U},
             {search_profile::kBike, 16U},
         }) {
      auto const p = route(profile, south_west);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
    for (auto const profile : {search_profile::kWheelchair}) {
      auto const p = route(profile, south_west);
      ASSERT_FALSE(p.has_value());
    }
  }
  // North-West: kerb=raised + wheelchair=limited: Wheelchair allowed
  {
    for (auto const [profile, cost] :
         std::initializer_list<std::tuple<search_profile, cost_t>>{
             {search_profile::kFoot, 51U},
             {search_profile::kWheelchair, 79U},
             {search_profile::kBike, 16U + 30U},
         }) {
      auto const p = route(profile, north_west);
      ASSERT_TRUE(p.has_value());
      EXPECT_EQ(cost, p->cost_);
      EXPECT_TRUE(p->dist_ > min_dist);
    }
  }
}

}  // namespace
