#include "gtest/gtest.h"

#include <filesystem>
#include <limits>
#include <string_view>
#include <vector>

#include "utl/helpers/algorithm.h"
#include "utl/to_vec.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/reachability.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {

// Query position of the matching tests.
constexpr auto const kQuery = geo::latlng{49.000000, 8.000000};

struct match_fixture : ::testing::Test {
  void load(char const* name, std::string_view const osm) {
    dir_ = fs::temp_directory_path() / name;
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf(name, osm), dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  void TearDown() override {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  way_idx_t way(std::int64_t const osm) const {
    return *w_->find_way(osm_way_idx_t{static_cast<std::uint64_t>(osm)});
  }

  std::optional<std::uint32_t> component(component_class const c,
                                         std::int64_t const osm,
                                         std::uint16_t const edge = 1U) const {
    return w_->r_->get_class_components(c).get(way(osm), edge);
  }

  template <Profile P = foot<false>>
  match_view_t match(location const& query,
                     double const max_match_distance = 50.0,
                     typename P::parameters const& params = {}) {
    out_.clear();
    l_->match<P>(params, query, true, direction::kForward, max_match_distance,
                 nullptr, out_);
    return out_[match_idx_t{0U}];
  }

  // OSM ids of the matched ways.
  template <Profile P = foot<false>>
  std::vector<std::int64_t> matched_ways(
      location const& query,
      double const max_match_distance = 50.0,
      typename P::parameters const& params = {}) {
    return utl::to_vec(match<P>(query, max_match_distance, params).way_,
                       [&](way_idx_t const x) { return *w_->get_osm_way(x); });
  }

  fs::path dir_;
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
  match_result out_;
};

// Two foot islands {100, 110} and {300, 310, 320}, joined by the motorway 200.
// Way 400 is detached.
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.001000"/>
  <node id="3" lat="49.000000" lon="8.002000"/>
  <node id="4" lat="49.000000" lon="8.003000"/>
  <node id="5" lat="49.000000" lon="8.004000"/>
  <node id="6" lat="49.000000" lon="8.005000"/>
  <node id="7" lat="49.000000" lon="8.006000"/>
  <node id="10" lat="49.010000" lon="8.000000"/>
  <node id="11" lat="49.010000" lon="8.001000"/>
  <way id="100">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="110">
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="200">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="highway" v="motorway"/>
    <tag k="foot" v="no"/>
  </way>
  <way id="300">
    <nd ref="4"/>
    <nd ref="5"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="310">
    <nd ref="5"/>
    <nd ref="6"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="320">
    <nd ref="6"/>
    <nd ref="7"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="400">
    <nd ref="10"/>
    <nd ref="11"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct class_components_test : match_fixture {
  void SetUp() override { load("osr-class-components", kOsm); }
};

TEST_F(class_components_test, components_per_class) {
  // Foot may not use the motorway 200: two islands.
  auto const foot_a = component(component_class::kFoot, 100);
  auto const foot_b = component(component_class::kFoot, 300);
  ASSERT_TRUE(foot_a.has_value());
  ASSERT_TRUE(foot_b.has_value());
  EXPECT_NE(*foot_a, *foot_b);

  // Car may: one component.
  auto const car_a = component(component_class::kCar, 200);
  ASSERT_TRUE(car_a.has_value());
  EXPECT_EQ(car_a, component(component_class::kCar, 300));

  // Not accessible for car, and a single-way component.
  EXPECT_FALSE(component(component_class::kCar, 100).has_value());
  EXPECT_FALSE(component(component_class::kFoot, 400).has_value());
}

// Around the query, each pair connected outside the match radius:
//   way 1  footway, bicycle=no   ~5.6 m north
//   way 2  footway, bicycle=no  ~16.7 m north
//   way 3  cycleway              ~6.7 m south
//   way 4  cycleway             ~17.8 m south
constexpr auto const kSharingOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000050" lon="7.999000"/>
  <node id="2" lat="49.000050" lon="8.001000"/>
  <node id="3" lat="49.000150" lon="7.999000"/>
  <node id="4" lat="49.000150" lon="8.001000"/>
  <node id="5" lat="48.999940" lon="7.999000"/>
  <node id="6" lat="48.999940" lon="8.001000"/>
  <node id="7" lat="48.999840" lon="7.999000"/>
  <node id="8" lat="48.999840" lon="8.001000"/>
  <way id="1">
    <nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="2">
    <nd ref="3"/><nd ref="4"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="3">
    <nd ref="5"/><nd ref="6"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="4">
    <nd ref="7"/><nd ref="8"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="5">
    <nd ref="2"/><nd ref="4"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="6">
    <nd ref="6"/><nd ref="8"/>
    <tag k="highway" v="cycleway"/>
  </way>
</osm>
)";

struct sharing_components_test : match_fixture {
  void SetUp() override { load("osr-sharing-components", kSharingOsm); }
};

TEST_F(sharing_components_test, bike_sharing_uses_foot_components) {
  auto const query = location{kQuery, kNoLevel};
  // One per foot component.
  EXPECT_EQ((std::vector<std::int64_t>{1, 3}), matched_ways(query));
  EXPECT_EQ((std::vector<std::int64_t>{1, 3}),
            matched_ways<bike_sharing>(query));
}

TEST_F(sharing_components_test,
       exact_return_filters_foot_and_bike_independently) {
  // Bike also filters: 3 again, but not the foot-only 2.
  auto params = bike_sharing::parameters{};
  params.vehicle_return_allowed_ = [](geo::latlng const&) { return true; };
  EXPECT_EQ(
      (std::vector<std::int64_t>{1, 3}),
      matched_ways<bike_sharing>(location{kQuery, kNoLevel}, 50.0, params));
}

// One foot component around the query, joined by a connector ~73 m east
// (outside the match radius):
//   way 4  cycleway            ~4.4 m south  penalised for foot
//   way 1  footway             ~5.6 m north
//   way 2  footway             ~6.1 m north  near-tie of way 1
//   way 3  footway            ~16.7 m north
//   way 5  cycleway           ~16.7 m south
constexpr auto const kFilterRulesOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="11" lat="49.000050" lon="7.999000"/>
  <node id="12" lat="49.000050" lon="8.001000"/>
  <node id="21" lat="49.000055" lon="7.999000"/>
  <node id="22" lat="49.000055" lon="8.001000"/>
  <node id="31" lat="49.000150" lon="7.999000"/>
  <node id="32" lat="49.000150" lon="8.001000"/>
  <node id="41" lat="48.999960" lon="7.999000"/>
  <node id="42" lat="48.999960" lon="8.001000"/>
  <node id="51" lat="48.999850" lon="7.999000"/>
  <node id="52" lat="48.999850" lon="8.001000"/>
  <way id="1">
    <nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="2">
    <nd ref="21"/><nd ref="22"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="3">
    <nd ref="31"/><nd ref="32"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="4">
    <nd ref="41"/><nd ref="42"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="5">
    <nd ref="51"/><nd ref="52"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="9">
    <nd ref="52"/><nd ref="42"/><nd ref="12"/><nd ref="22"/><nd ref="32"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
</osm>
)";

struct filter_rules_test : match_fixture {
  void SetUp() override { load("osr-filter-rules", kFilterRulesOsm); }
};

TEST_F(filter_rules_test, penalised_way_does_not_shadow_and_ties_survive) {
  // 4 is the closest but penalised for foot, so it does not shadow 1; 2 is a
  // near-tie of 1; 3 is shadowed by 1 and 5 by 4.
  EXPECT_EQ((std::vector<std::int64_t>{4, 1, 2}),
            matched_ways(location{kQuery, kNoLevel}));
}

TEST_F(filter_rules_test, penalised_way_stays_penalty_reference) {
  auto const m = match(location{kQuery, kNoLevel});
  ASSERT_FALSE(m.empty());
  EXPECT_EQ(std::optional<std::int64_t>{4}, w_->get_osm_way(m.way_[0]));
  EXPECT_FLOAT_EQ(m.dist_to_way_[0], m.penalty_ref_[0]);
}

// Stacked ways around the query, all one foot component via steps ~73 m east
// (outside the match radius):
//   way 1  level -1   ~1.1 m north
//   way 2  level -2   ~2.2 m north
//   way 3  level -1   ~3.3 m north
//   way 4  level  0   ~5.6 m north
//   way 5  level  0   ~8.9 m north
//   way 6  level -1  ~12.2 m north
constexpr auto const kLevelsOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="11" lat="49.000010" lon="7.999000"/>
  <node id="12" lat="49.000010" lon="8.001000"/>
  <node id="21" lat="49.000020" lon="7.999000"/>
  <node id="22" lat="49.000020" lon="8.001000"/>
  <node id="31" lat="49.000030" lon="7.999000"/>
  <node id="32" lat="49.000030" lon="8.001000"/>
  <node id="41" lat="49.000050" lon="7.999000"/>
  <node id="42" lat="49.000050" lon="8.001000"/>
  <node id="51" lat="49.000080" lon="7.999000"/>
  <node id="52" lat="49.000080" lon="8.001000"/>
  <node id="61" lat="49.000110" lon="7.999000"/>
  <node id="62" lat="49.000110" lon="8.001000"/>
  <way id="1">
    <nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="2">
    <nd ref="21"/><nd ref="22"/>
    <tag k="highway" v="footway"/><tag k="level" v="-2"/>
  </way>
  <way id="3">
    <nd ref="31"/><nd ref="32"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="4">
    <nd ref="41"/><nd ref="42"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="5">
    <nd ref="51"/><nd ref="52"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="6">
    <nd ref="61"/><nd ref="62"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="7">
    <nd ref="22"/><nd ref="12"/>
    <tag k="highway" v="steps"/><tag k="level" v="-2;-1"/>
  </way>
  <way id="8">
    <nd ref="12"/><nd ref="32"/><nd ref="62"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="9">
    <nd ref="62"/><nd ref="42"/>
    <tag k="highway" v="steps"/><tag k="level" v="-1;0"/>
  </way>
  <way id="10">
    <nd ref="42"/><nd ref="52"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct levels_test : match_fixture {
  void SetUp() override { load("osr-filter-levels", kLevelsOsm); }
};

TEST_F(levels_test, without_level_keeps_closest_per_level_and_ground) {
  // 3 is shadowed by 1 on the same level, 5 and 6 by the ground-level 4.
  EXPECT_EQ((std::vector<std::int64_t>{1, 2, 4}),
            matched_ways(location{kQuery, kNoLevel}));
}

TEST_F(levels_test, with_level_only_that_level_matches) {
  EXPECT_EQ((std::vector<std::int64_t>{1}),
            matched_ways(location{kQuery, level_t{-1.0F}}));
  EXPECT_EQ((std::vector<std::int64_t>{4}),
            matched_ways(location{kQuery, level_t{0.0F}}));
}

TEST_F(levels_test, penalty_is_measured_from_closest_on_ground) {
  auto const no_level = match(location{kQuery, kNoLevel});
  ASSERT_EQ(3U, no_level.size());
  EXPECT_FLOAT_EQ(no_level.dist_to_way_[2], no_level.penalty_ref_[0]);

  auto const with_level = match(location{kQuery, level_t{-1.0F}});
  ASSERT_FALSE(with_level.empty());
  EXPECT_FLOAT_EQ(with_level.dist_to_way_[0], with_level.penalty_ref_[0]);
}

// A dual carriageway around the query, one component via two-way roads ~73 m
// to the east and west (outside the match radius):
//   way 1  oneway eastbound   ~5.6 m north
//   way 2  oneway westbound  ~11.1 m south
//   way 3  oneway eastbound  ~16.7 m north
//   way 4  two-way           ~27.8 m north
constexpr auto const kOnewayOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="11" lat="49.000050" lon="7.999000"/>
  <node id="12" lat="49.000050" lon="8.001000"/>
  <node id="21" lat="48.999900" lon="7.999000"/>
  <node id="22" lat="48.999900" lon="8.001000"/>
  <node id="31" lat="49.000150" lon="7.999000"/>
  <node id="32" lat="49.000150" lon="8.001000"/>
  <node id="41" lat="49.000250" lon="7.999000"/>
  <node id="42" lat="49.000250" lon="8.001000"/>
  <way id="1">
    <nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="residential"/><tag k="oneway" v="yes"/>
  </way>
  <way id="2">
    <nd ref="22"/><nd ref="21"/>
    <tag k="highway" v="residential"/><tag k="oneway" v="yes"/>
  </way>
  <way id="3">
    <nd ref="31"/><nd ref="32"/>
    <tag k="highway" v="residential"/><tag k="oneway" v="yes"/>
  </way>
  <way id="4">
    <nd ref="41"/><nd ref="42"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="8">
    <nd ref="21"/><nd ref="11"/><nd ref="31"/><nd ref="41"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="9">
    <nd ref="22"/><nd ref="12"/><nd ref="32"/><nd ref="42"/>
    <tag k="highway" v="residential"/>
  </way>
</osm>
)";

struct oneway_test : match_fixture {
  void SetUp() override { load("osr-filter-oneway", kOnewayOsm); }
};

TEST_F(oneway_test, oneway_only_shadows_same_direction) {
  // 1 shadows 3 (same direction), but not 2 (opposite) or the two-way 4.
  EXPECT_EQ((std::vector<std::int64_t>{1, 2, 4}),
            matched_ways<car>(location{kQuery, kNoLevel}));
  EXPECT_EQ((std::vector<std::int64_t>{1}),
            matched_ways(location{kQuery, kNoLevel}));
}

// An elevator (levels -3, -2, -1) next to the query, one foot component:
//   way 2  level -2  ~2.2 m north, ends at the elevator
//   way 1  level -1  ~5.6 m north, ends at the elevator
// Nothing is mapped on level -3.
constexpr auto const kElevatorOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="10" lat="49.000020" lon="8.000100">
    <tag k="highway" v="elevator"/><tag k="level" v="-3;-2;-1"/>
  </node>
  <node id="11" lat="49.000050" lon="7.999000"/>
  <node id="12" lat="49.000050" lon="8.000100"/>
  <node id="21" lat="49.000020" lon="7.999000"/>
  <way id="1">
    <nd ref="11"/><nd ref="12"/><nd ref="10"/>
    <tag k="highway" v="footway"/><tag k="level" v="-1"/>
  </way>
  <way id="2">
    <nd ref="21"/><nd ref="10"/>
    <tag k="highway" v="footway"/><tag k="level" v="-2"/>
  </way>
</osm>
)";

struct elevator_levels_test : match_fixture {
  void SetUp() override { load("osr-filter-elevator", kElevatorOsm); }
};

TEST_F(elevator_levels_test, elevator_candidate_does_not_shadow_on_level_one) {
  // Way 1 is behind the on-level way 2.
  EXPECT_EQ((std::vector<std::int64_t>{2}),
            matched_ways(location{kQuery, level_t{-2.0F}}));
  // Way 2 is closer but needs the elevator.
  EXPECT_EQ((std::vector<std::int64_t>{2, 1}),
            matched_ways(location{kQuery, level_t{-1.0F}}));
  // Nothing on level -3: reachable through the elevator.
  EXPECT_FALSE(matched_ways(location{kQuery, level_t{-3.0F}}).empty());
}

// Two parallel footways ~111 m apart (1 along 49.000, 2 along 49.001), one
// component through way 3 at their western end.
constexpr auto const kParallelOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000"/>
  <node id="3" lat="49.001000" lon="8.000000"/>
  <node id="4" lat="49.001000" lon="8.002000"/>
  <way id="1">
    <nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="2">
    <nd ref="3"/><nd ref="4"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="3">
    <nd ref="1"/><nd ref="3"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct tie_tolerance_test : match_fixture {
  void SetUp() override { load("osr-filter-tie", kParallelOsm); }
};

TEST_F(tie_tolerance_test, tolerance_grows_with_the_distance) {
  // 2 m from way 1: way 2 (109 m) is shadowed.
  EXPECT_EQ((std::vector<std::int64_t>{1}),
            matched_ways(location{{49.000020, 8.001000}, kNoLevel}, 150.0));
  // 53 m / 58 m: within 25 % of 53 m, both are kept.
  EXPECT_EQ((std::vector<std::int64_t>{1, 2}),
            matched_ways(location{{49.000480, 8.001000}, kNoLevel}, 150.0));
}

// Way 500 (way nodes 31, 33, 34, 35) has a wall at 33: edge 1 lies before it,
// edges 2 and 3 after it. Way 600 (way nodes 41, 42, 43) has a bollard at 42.
// The closed way 700 (way nodes 51, 52, 51) has a wall at 52, but both pieces
// meet at 51. Way 800 (way nodes 81, 82, 83, 82, 85) and the dead end 900 (way
// nodes 91, 92, 92) pass a node twice. The detached footway 1000 (way nodes
// 102, 103, 104) has a wall at 103 between two bollards.
constexpr auto const kBarrierOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="31" lat="49.020000" lon="8.000000"/>
  <node id="32" lat="49.020000" lon="8.001000"/>
  <node id="33" lat="49.020000" lon="8.002000">
    <tag k="barrier" v="wall"/>
  </node>
  <node id="34" lat="49.020000" lon="8.003000"/>
  <node id="35" lat="49.020000" lon="8.004000"/>
  <node id="36" lat="49.021000" lon="8.000000"/>
  <node id="37" lat="49.021000" lon="8.004000"/>
  <node id="38" lat="49.021000" lon="8.003000"/>
  <node id="41" lat="49.030000" lon="8.000000"/>
  <node id="42" lat="49.030000" lon="8.001000">
    <tag k="barrier" v="bollard"/>
  </node>
  <node id="43" lat="49.030000" lon="8.002000"/>
  <node id="44" lat="49.031000" lon="8.000000"/>
  <node id="45" lat="49.031000" lon="8.002000"/>
  <node id="51" lat="49.040000" lon="8.000000"/>
  <node id="52" lat="49.040000" lon="8.001000">
    <tag k="barrier" v="wall"/>
  </node>
  <node id="53" lat="49.041000" lon="8.001000"/>
  <node id="54" lat="49.041000" lon="8.000000"/>
  <node id="55" lat="49.040000" lon="7.999000"/>
  <node id="80" lat="49.049000" lon="8.000000"/>
  <node id="81" lat="49.050000" lon="8.000000"/>
  <node id="82" lat="49.050000" lon="8.001000"/>
  <node id="83" lat="49.050500" lon="8.001500">
    <tag k="barrier" v="wall"/>
  </node>
  <node id="84" lat="49.050500" lon="8.000500"/>
  <node id="85" lat="49.050000" lon="8.003000"/>
  <node id="86" lat="49.049000" lon="8.003000"/>
  <node id="90" lat="49.059000" lon="8.000000"/>
  <node id="91" lat="49.060000" lon="8.000000"/>
  <node id="92" lat="49.060000" lon="8.001000"/>
  <node id="93" lat="49.060500" lon="8.001500"/>
  <node id="94" lat="49.060500" lon="8.000500"/>
  <node id="95" lat="49.060000" lon="8.003000"/>
  <node id="101" lat="49.070000" lon="8.000000"/>
  <node id="102" lat="49.070000" lon="8.001000">
    <tag k="barrier" v="bollard"/>
  </node>
  <node id="103" lat="49.070000" lon="8.002000">
    <tag k="barrier" v="wall"/>
  </node>
  <node id="104" lat="49.070000" lon="8.003000">
    <tag k="barrier" v="bollard"/>
  </node>
  <node id="105" lat="49.070000" lon="8.004000"/>
  <way id="500">
    <nd ref="31"/><nd ref="32"/><nd ref="33"/><nd ref="34"/><nd ref="35"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="510">
    <nd ref="36"/><nd ref="31"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="520">
    <nd ref="35"/><nd ref="37"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="530">
    <nd ref="34"/><nd ref="38"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="600">
    <nd ref="41"/><nd ref="42"/><nd ref="43"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="610">
    <nd ref="44"/><nd ref="41"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="620">
    <nd ref="43"/><nd ref="45"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="700">
    <nd ref="51"/><nd ref="52"/><nd ref="53"/><nd ref="54"/><nd ref="51"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="710">
    <nd ref="55"/><nd ref="51"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="800">
    <nd ref="81"/><nd ref="82"/><nd ref="83"/><nd ref="84"/><nd ref="82"/>
    <nd ref="85"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="810">
    <nd ref="80"/><nd ref="81"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="820">
    <nd ref="85"/><nd ref="86"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="900">
    <nd ref="91"/><nd ref="92"/><nd ref="93"/><nd ref="94"/><nd ref="92"/>
    <nd ref="95"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="910">
    <nd ref="90"/><nd ref="91"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="1000">
    <nd ref="101"/><nd ref="102"/><nd ref="103"/><nd ref="104"/><nd ref="105"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct barrier_split_test : match_fixture {
  void SetUp() override { load("osr-barrier-split", kBarrierOsm); }

  // Edge of the candidate on way `osm` matched at `pos`.
  std::uint16_t edge_at(std::int64_t const osm, geo::latlng const& pos) {
    auto const m = match(location{pos, kNoLevel});
    auto const it = utl::find(m.way_, way(osm));
    EXPECT_NE(it, end(m.way_));
    return it == end(m.way_)
               ? std::uint16_t{0U}
               : candidate_edge(*w_, *it,
                                m.nodes_[static_cast<std::size_t>(
                                    std::distance(begin(m.way_), it))]);
  }
};

TEST_F(barrier_split_test, impassable_node_splits_way) {
  for (auto const c : kAllComponentClasses) {
    SCOPED_TRACE(static_cast<int>(c));
    auto const before = component(c, 500, 1U);
    auto const after = component(c, 500, 2U);
    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());
    EXPECT_NE(*before, *after);
    EXPECT_EQ(before, component(c, 510));
    EXPECT_EQ(after, component(c, 500, 3U));
    EXPECT_EQ(after, component(c, 520));
    EXPECT_EQ(after, component(c, 530));
  }
}

TEST_F(barrier_split_test, split_depends_on_class) {
  // The bollard blocks cars only.
  auto const foot = component(component_class::kFoot, 600, 1U);
  ASSERT_TRUE(foot.has_value());
  EXPECT_EQ(foot, component(component_class::kFoot, 600, 2U));
  EXPECT_EQ(foot, component(component_class::kFoot, 610));
  EXPECT_EQ(foot, component(component_class::kFoot, 620));

  auto const car_before = component(component_class::kCar, 600, 1U);
  auto const car_after = component(component_class::kCar, 600, 2U);
  ASSERT_TRUE(car_before.has_value());
  ASSERT_TRUE(car_after.has_value());
  EXPECT_NE(*car_before, *car_after);
  EXPECT_EQ(car_before, component(component_class::kCar, 610));
  EXPECT_EQ(car_after, component(component_class::kCar, 620));
}

TEST_F(barrier_split_test, closed_way_pieces_meet_at_closing_node) {
  for (auto const c : kAllComponentClasses) {
    SCOPED_TRACE(static_cast<int>(c));
    auto const first = component(c, 700, 1U);
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first, component(c, 700, 2U));
    EXPECT_EQ(first, component(c, 710));
  }
}

TEST_F(barrier_split_test, single_pieces_have_distinct_keys) {
  auto const& foot = w_->r_->get_class_components(component_class::kFoot);
  auto const w = way(1000);
  // Each side of the wall is a component of its own, without an id.
  EXPECT_FALSE(foot.get(w, 1U).has_value());
  EXPECT_FALSE(foot.get(w, 2U).has_value());
  EXPECT_NE(foot.key(w, 1U), foot.key(w, 2U));
  EXPECT_EQ(foot.key(w, 0U), foot.key(w, 1U));
  EXPECT_EQ(foot.key(w, 2U), foot.key(w, 3U));
  // Single-way keys come after all ids.
  EXPECT_GT(foot.key(w, 1U), std::numeric_limits<std::uint32_t>::max());
}

TEST_F(barrier_split_test, split_way_counts_once_per_component) {
  for (auto const c : kAllComponentClasses) {
    SCOPED_TRACE(static_cast<int>(c));
    auto const& cc = w_->r_->get_class_components(c);
    // 500 + 510 before the wall, 500 + 520 + 530 after it.
    EXPECT_EQ(2U, cc.size(*component(c, 500, 1U)));
    EXPECT_EQ(3U, cc.size(*component(c, 500, 2U)));
    // Both pieces of 700 lie in the component of 700 + 710.
    EXPECT_EQ(2U, cc.size(*component(c, 700, 1U)));
  }
}

TEST_F(barrier_split_test, candidate_edge_selects_piece) {
  EXPECT_EQ(1U, edge_at(500, {49.020050, 8.001000}));
  EXPECT_EQ(2U, edge_at(500, {49.020050, 8.002500}));
  EXPECT_EQ(3U, edge_at(500, {49.020050, 8.003500}));
}

TEST_F(barrier_split_test, candidate_edge_with_repeated_node) {
  // Edge 4 (second 82 to 85), not edge 2 after the first 82.
  EXPECT_EQ(4U, edge_at(800, {49.049950, 8.002000}));
  // After the last routing node (the second 92): no right node.
  EXPECT_EQ(3U, edge_at(900, {49.059950, 8.002000}));
}

TEST_F(barrier_split_test, pruning_respects_pieces) {
  using P = foot<false>;
  auto const match_at = [&](geo::latlng const& pos, match_result& out) {
    l_->match<P>({}, location{pos, kNoLevel}, true, direction::kForward, 50.0,
                 nullptr, out);
    return out[match_idx_t{0U}];
  };
  auto before_m = match_result{};
  auto after_m = match_result{};
  auto after2_m = match_result{};
  auto const before = match_at({49.020050, 8.001000}, before_m);
  auto const after = match_at({49.020050, 8.003500}, after_m);
  auto const after2 = match_at({49.020050, 8.002500}, after2_m);
  ASSERT_NE(utl::find(before.way_, way(500)), end(before.way_));
  ASSERT_NE(utl::find(after.way_, way(500)), end(after.way_));

  // Same way, but on both sides of the wall.
  EXPECT_TRUE(
      connected_candidates<P>(*w_, nullptr, before, after)[match_idx_t{0U}]
          .empty());
  EXPECT_TRUE(
      connected_candidates<P>(*w_, nullptr, after, before)[match_idx_t{0U}]
          .empty());
  EXPECT_FALSE(
      connected_candidates<P>(*w_, nullptr, after, after2)[match_idx_t{0U}]
          .empty());
}

}  // namespace osr
