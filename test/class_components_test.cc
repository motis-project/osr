#include "gtest/gtest.h"

#include <filesystem>
#include <string_view>
#include <vector>

#include "utl/to_vec.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

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

  std::optional<std::uint32_t> component(component_class const c,
                                         std::int64_t const osm) const {
    return w_->r_->get_class_components(c).get(
        *w_->find_way(osm_way_idx_t{static_cast<std::uint64_t>(osm)}));
  }

  template <Profile P = foot<false>>
  match_view_t match(location const& query,
                     bool const exact_return_allowed = false,
                     double const max_match_distance = 50.0) {
    out_.clear();
    l_->match<P>(typename P::parameters{}, query, true, direction::kForward,
                 max_match_distance, nullptr, exact_return_allowed, out_);
    return out_[match_idx_t{0U}];
  }

  // OSM ids of the matched ways.
  template <Profile P = foot<false>>
  std::vector<std::int64_t> matched_ways(
      location const& query,
      bool const exact_return_allowed = false,
      double const max_match_distance = 50.0) {
    return utl::to_vec(
        match<P>(query, exact_return_allowed, max_match_distance).way_,
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
//   way 3  cycleway, foot=no     ~6.7 m south
//   way 4  cycleway, foot=no    ~17.8 m south
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
    <tag k="highway" v="cycleway"/><tag k="foot" v="no"/>
  </way>
  <way id="4">
    <nd ref="7"/><nd ref="8"/>
    <tag k="highway" v="cycleway"/><tag k="foot" v="no"/>
  </way>
  <way id="5">
    <nd ref="2"/><nd ref="4"/>
    <tag k="highway" v="footway"/><tag k="bicycle" v="no"/>
  </way>
  <way id="6">
    <nd ref="6"/><nd ref="8"/>
    <tag k="highway" v="cycleway"/><tag k="foot" v="no"/>
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
  EXPECT_EQ((std::vector<std::int64_t>{1, 3}),
            matched_ways<bike_sharing>(location{kQuery, kNoLevel}, true));
}

// One foot component around the query, joined by a connector ~73 m east
// (outside the match radius):
//   way 4  cycleway, foot=no   ~4.4 m south  penalised for foot
//   way 1  footway             ~5.6 m north
//   way 2  footway             ~6.1 m north  near-tie of way 1
//   way 3  footway            ~16.7 m north
//   way 5  cycleway, foot=no  ~16.7 m south
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
    <tag k="highway" v="cycleway"/><tag k="foot" v="no"/>
  </way>
  <way id="5">
    <nd ref="51"/><nd ref="52"/>
    <tag k="highway" v="cycleway"/><tag k="foot" v="no"/>
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
  EXPECT_FLOAT_EQ(m.dist_to_way_[0], m.penalty_ref_);
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
  EXPECT_FLOAT_EQ(no_level.dist_to_way_[2], no_level.penalty_ref_);

  auto const with_level = match(location{kQuery, level_t{-1.0F}});
  ASSERT_FALSE(with_level.empty());
  EXPECT_FLOAT_EQ(with_level.dist_to_way_[0], with_level.penalty_ref_);
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
  EXPECT_EQ(
      (std::vector<std::int64_t>{1}),
      matched_ways(location{{49.000020, 8.001000}, kNoLevel}, false, 150.0));
  // 53 m / 58 m: within 25 % of 53 m, both are kept.
  EXPECT_EQ(
      (std::vector<std::int64_t>{1, 2}),
      matched_ways(location{{49.000480, 8.001000}, kNoLevel}, false, 150.0));
}

}  // namespace
}  // namespace osr
