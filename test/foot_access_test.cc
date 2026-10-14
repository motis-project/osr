#include "gtest/gtest.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {

// Separate roads, 1.1 km apart, each of two ways meeting at 8.003:
// - 100, 101: trunk without foot tags
// - 200, 201: trunk with sidewalk=both
// - 300, 301: trunk with motorroad=yes
// - 400, 401: footway with a gate (node 41)
// - 500, 501: footway with an open gate (node 51)
// - 600, 601: trunk, 610, 611: cycleway 10 m north of it, connected at both
//   ends by the footways 620, 621
// - 700, 701: trunk with maxspeed=50
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="10" lat="49.000000" lon="8.000000"/>
  <node id="11" lat="49.000000" lon="8.003000"/>
  <node id="12" lat="49.000000" lon="8.006000"/>
  <node id="20" lat="49.010000" lon="8.000000"/>
  <node id="21" lat="49.010000" lon="8.003000"/>
  <node id="22" lat="49.010000" lon="8.006000"/>
  <node id="30" lat="49.020000" lon="8.000000"/>
  <node id="31" lat="49.020000" lon="8.003000"/>
  <node id="32" lat="49.020000" lon="8.006000"/>
  <node id="40" lat="49.030000" lon="8.000000"/>
  <node id="41" lat="49.030000" lon="8.003000">
    <tag k="barrier" v="gate"/>
  </node>
  <node id="42" lat="49.030000" lon="8.006000"/>
  <node id="50" lat="49.040000" lon="8.000000"/>
  <node id="51" lat="49.040000" lon="8.003000">
    <tag k="barrier" v="gate"/>
    <tag k="access" v="yes"/>
  </node>
  <node id="52" lat="49.040000" lon="8.006000"/>
  <way id="100">
    <nd ref="10"/><nd ref="11"/>
    <tag k="highway" v="trunk"/>
  </way>
  <way id="101">
    <nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="trunk"/>
  </way>
  <way id="200">
    <nd ref="20"/><nd ref="21"/>
    <tag k="highway" v="trunk"/><tag k="sidewalk" v="both"/>
  </way>
  <way id="201">
    <nd ref="21"/><nd ref="22"/>
    <tag k="highway" v="trunk"/><tag k="sidewalk" v="both"/>
  </way>
  <way id="300">
    <nd ref="30"/><nd ref="31"/>
    <tag k="highway" v="trunk"/><tag k="motorroad" v="yes"/>
  </way>
  <way id="301">
    <nd ref="31"/><nd ref="32"/>
    <tag k="highway" v="trunk"/><tag k="motorroad" v="yes"/>
  </way>
  <way id="400">
    <nd ref="40"/><nd ref="41"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="401">
    <nd ref="41"/><nd ref="42"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="500">
    <nd ref="50"/><nd ref="51"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="501">
    <nd ref="51"/><nd ref="52"/>
    <tag k="highway" v="footway"/>
  </way>
  <node id="60" lat="49.050000" lon="8.000000"/>
  <node id="61" lat="49.050000" lon="8.003000"/>
  <node id="62" lat="49.050000" lon="8.006000"/>
  <node id="63" lat="49.050090" lon="8.000000"/>
  <node id="64" lat="49.050090" lon="8.003000"/>
  <node id="65" lat="49.050090" lon="8.006000"/>
  <way id="600">
    <nd ref="60"/><nd ref="61"/>
    <tag k="highway" v="trunk"/>
  </way>
  <way id="601">
    <nd ref="61"/><nd ref="62"/>
    <tag k="highway" v="trunk"/>
  </way>
  <way id="610">
    <nd ref="63"/><nd ref="64"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="611">
    <nd ref="64"/><nd ref="65"/>
    <tag k="highway" v="cycleway"/>
  </way>
  <way id="620">
    <nd ref="60"/><nd ref="63"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="621">
    <nd ref="62"/><nd ref="65"/>
    <tag k="highway" v="footway"/>
  </way>
  <node id="70" lat="49.060000" lon="8.000000"/>
  <node id="71" lat="49.060000" lon="8.003000"/>
  <node id="72" lat="49.060000" lon="8.006000"/>
  <way id="700">
    <nd ref="70"/><nd ref="71"/>
    <tag k="highway" v="trunk"/><tag k="maxspeed" v="50"/>
  </way>
  <way id="701">
    <nd ref="71"/><nd ref="72"/>
    <tag k="highway" v="trunk"/><tag k="maxspeed" v="50"/>
  </way>
</osm>
)";

struct foot_access_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-foot-access";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("osr-foot-access", kOsm), dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static std::optional<path> route(
      double const lat,
      foot<false, elevator_tracking>::parameters const& params) {
    return osr::route(params, *w_, *l_, search_profile::kFoot,
                      location{{lat, 8.001}, kNoLevel},
                      location{{lat, 8.005}, kNoLevel},
                      std::chrono::seconds{3600}, direction::kForward, 20.0,
                      nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
  }

  static std::optional<path> bike_route(
      double const lat,
      bike<bike_costing::kSafe, kElevationNoCost>::parameters const& params) {
    return osr::route(params, *w_, *l_, search_profile::kBike,
                      location{{lat, 8.001}, kNoLevel},
                      location{{lat, 8.005}, kNoLevel},
                      std::chrono::seconds{3600}, direction::kForward, 20.0,
                      nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
  }

  static way_properties props(osm_way_idx_t const id) {
    return w_->r_->way_properties_[*w_->find_way(id)];
  }

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

TEST_F(foot_access_test, trunk_is_walkable_with_penalty) {
  auto const p = props(osm_way_idx_t{100U});
  EXPECT_FALSE(p.is_foot_accessible());
  EXPECT_TRUE(p.is_foot_accessible_with_penalty());
  EXPECT_TRUE(is_accessible(component_class::kFoot, p));

  auto const once = route(49.0, {.trunk_cost_factor_ = 1.0F});
  auto const twice = route(49.0, {.trunk_cost_factor_ = 2.0F});
  ASSERT_TRUE(once.has_value());
  ASSERT_TRUE(twice.has_value());
  EXPECT_NEAR(292.0, once->dist_, 2.0);
  EXPECT_EQ(once->duration_, twice->duration_);
  EXPECT_NEAR(2.0 * once->cost_, twice->cost_, 2.0);
}

TEST_F(foot_access_test, trunk_with_sidewalk_has_no_penalty) {
  auto const p = props(osm_way_idx_t{200U});
  EXPECT_TRUE(p.is_foot_accessible());
  EXPECT_FALSE(p.is_foot_accessible_with_penalty());

  auto const once = route(49.01, {.trunk_cost_factor_ = 1.0F});
  auto const twice = route(49.01, {.trunk_cost_factor_ = 2.0F});
  ASSERT_TRUE(once.has_value());
  ASSERT_TRUE(twice.has_value());
  EXPECT_EQ(once->cost_, twice->cost_);
}

TEST_F(foot_access_test, motorroad_is_not_walkable) {
  auto const p = props(osm_way_idx_t{300U});
  EXPECT_FALSE(p.is_foot_accessible());
  EXPECT_FALSE(p.is_foot_accessible_with_penalty());
  EXPECT_FALSE(route(49.02, {}).has_value());
}

TEST_F(foot_access_test, gate_costs) {
  auto const free = route(49.03, {.gate_cost_ = 0U});
  auto const gate = route(49.03, {.gate_cost_ = 60U});
  ASSERT_TRUE(free.has_value());
  ASSERT_TRUE(gate.has_value());
  EXPECT_EQ(free->cost_ + 60U, gate->cost_);
  EXPECT_EQ(free->duration_, gate->duration_);

  auto const open_free = route(49.04, {.gate_cost_ = 0U});
  auto const open_gate = route(49.04, {.gate_cost_ = 60U});
  ASSERT_TRUE(open_free.has_value());
  ASSERT_TRUE(open_gate.has_value());
  EXPECT_EQ(open_free->cost_, open_gate->cost_);
}

// A closer trunk candidate must not drop the cycleway (less penalised).
TEST_F(foot_access_test, trunk_does_not_shadow_cycleway) {
  using profile_t = foot<false, elevator_tracking>;
  auto m = match_result{};
  l_->match<profile_t>(profile_t::parameters{},
                       location{{49.050030, 8.0015}, kNoLevel}, false,
                       direction::kForward, 20.0, nullptr, m);
  auto const cycleway = *w_->find_way(osm_way_idx_t{610U});
  auto const ways = m[match_idx_t{0U}].way_;
  EXPECT_NE(end(ways), std::find(begin(ways), end(ways), cycleway));
}

// Neither does a closer cycleway drop the trunk: both are penalised, but for
// different reasons.
TEST_F(foot_access_test, cycleway_does_not_shadow_trunk) {
  using profile_t = foot<false, elevator_tracking>;
  auto m = match_result{};
  l_->match<profile_t>(profile_t::parameters{},
                       location{{49.050080, 8.0015}, kNoLevel}, false,
                       direction::kForward, 20.0, nullptr, m);
  auto const trunk = *w_->find_way(osm_way_idx_t{600U});
  auto const ways = m[match_idx_t{0U}].way_;
  EXPECT_NE(end(ways), std::find(begin(ways), end(ways), trunk));
}

TEST_F(foot_access_test, slow_trunk_has_no_factor) {
  auto const once = route(49.06, {.trunk_cost_factor_ = 1.0F});
  auto const twice = route(49.06, {.trunk_cost_factor_ = 2.0F});
  ASSERT_TRUE(once.has_value());
  ASSERT_TRUE(twice.has_value());
  EXPECT_EQ(once->cost_, twice->cost_);
}

TEST_F(foot_access_test, bike_on_trunk) {
  auto const p = props(osm_way_idx_t{100U});
  EXPECT_FALSE(p.is_bike_accessible());
  EXPECT_TRUE(p.is_bike_accessible_with_penalty());
  EXPECT_TRUE(is_accessible(component_class::kBike, p));
  EXPECT_FALSE(props(osm_way_idx_t{300U}).is_bike_accessible_with_penalty());

  auto const once = bike_route(49.0, {.trunk_cost_factor_ = 1.0F});
  auto const twice = bike_route(49.0, {.trunk_cost_factor_ = 2.0F});
  ASSERT_TRUE(once.has_value());
  ASSERT_TRUE(twice.has_value());
  EXPECT_EQ(once->duration_, twice->duration_);
  EXPECT_NEAR(2.0 * once->cost_, twice->cost_, 2.0);
  EXPECT_FALSE(bike_route(49.02, {}).has_value());
}

}  // namespace osr
