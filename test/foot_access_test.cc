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
// - 600, 601: trunk, 610, 611: cycleway 10 m north of it, connected at both
//   ends by the footways 620, 621
// - 800, 801: path with a private gate (node 81)
// - 900, 901: path with a foot=no gate (node 91)
// - 1000, 1001: trunk with foot=no
// - 1100, 1101: path with a private gate and kerb=raised (node 111)
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
  <node id="80" lat="49.070000" lon="8.000000"/>
  <node id="81" lat="49.070000" lon="8.003000">
    <tag k="barrier" v="gate"/>
    <tag k="access" v="private"/>
  </node>
  <node id="82" lat="49.070000" lon="8.006000"/>
  <node id="90" lat="49.080000" lon="8.000000"/>
  <node id="91" lat="49.080000" lon="8.003000">
    <tag k="barrier" v="gate"/>
    <tag k="foot" v="no"/>
    <tag k="bicycle" v="no"/>
  </node>
  <node id="92" lat="49.080000" lon="8.006000"/>
  <way id="800">
    <nd ref="80"/><nd ref="81"/>
    <tag k="highway" v="path"/>
  </way>
  <way id="801">
    <nd ref="81"/><nd ref="82"/>
    <tag k="highway" v="path"/>
  </way>
  <way id="900">
    <nd ref="90"/><nd ref="91"/>
    <tag k="highway" v="path"/>
  </way>
  <way id="901">
    <nd ref="91"/><nd ref="92"/>
    <tag k="highway" v="path"/>
  </way>
  <node id="100" lat="49.090000" lon="8.000000"/>
  <node id="101" lat="49.090000" lon="8.003000"/>
  <node id="102" lat="49.090000" lon="8.006000"/>
  <way id="1000">
    <nd ref="100"/><nd ref="101"/>
    <tag k="highway" v="trunk"/><tag k="foot" v="no"/>
  </way>
  <way id="1001">
    <nd ref="101"/><nd ref="102"/>
    <tag k="highway" v="trunk"/><tag k="foot" v="no"/>
  </way>
  <node id="110" lat="49.100000" lon="8.000000"/>
  <node id="111" lat="49.100000" lon="8.003000">
    <tag k="barrier" v="gate"/>
    <tag k="access" v="private"/>
    <tag k="kerb" v="raised"/>
  </node>
  <node id="112" lat="49.100000" lon="8.006000"/>
  <way id="1100">
    <nd ref="110"/><nd ref="111"/>
    <tag k="highway" v="path"/>
  </way>
  <way id="1101">
    <nd ref="111"/><nd ref="112"/>
    <tag k="highway" v="path"/>
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

  static std::optional<path> route(double const lat) {
    return osr::route(foot<false, elevator_tracking>::parameters{}, *w_, *l_,
                      search_profile::kFoot, location{{lat, 8.001}, kNoLevel},
                      location{{lat, 8.005}, kNoLevel},
                      std::chrono::seconds{3600}, direction::kForward, 20.0,
                      nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
  }

  static std::optional<path> bike_route(double const lat) {
    return osr::route(bike<bike_costing::kSafe, kElevationNoCost>::parameters{},
                      *w_, *l_, search_profile::kBike,
                      location{{lat, 8.001}, kNoLevel},
                      location{{lat, 8.005}, kNoLevel},
                      std::chrono::seconds{3600}, direction::kForward, 20.0,
                      nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
  }

  static std::optional<path> wheelchair_route(double const lat) {
    return osr::route(foot<true, elevator_tracking>::parameters{}, *w_, *l_,
                      search_profile::kWheelchair,
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
  EXPECT_TRUE(p.is_foot_accessible());
  EXPECT_TRUE(p.has_foot_penalty());
  EXPECT_TRUE(is_accessible(component_class::kFoot, p));

  auto const r = route(49.0);
  ASSERT_TRUE(r.has_value());
  EXPECT_NEAR(292.0, r->dist_, 2.0);
}

TEST_F(foot_access_test, trunk_with_sidewalk_has_no_penalty) {
  auto const p = props(osm_way_idx_t{200U});
  EXPECT_TRUE(p.is_foot_accessible());
  EXPECT_FALSE(p.has_foot_penalty());
}

TEST_F(foot_access_test, motorroad_is_not_walkable) {
  auto const p = props(osm_way_idx_t{300U});
  EXPECT_FALSE(p.is_foot_accessible());
  EXPECT_FALSE(p.has_foot_penalty());
  EXPECT_FALSE(route(49.02).has_value());
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

TEST_F(foot_access_test, bike_on_trunk) {
  auto const p = props(osm_way_idx_t{100U});
  EXPECT_TRUE(p.is_bike_accessible());
  EXPECT_TRUE(p.has_bike_penalty());
  EXPECT_TRUE(is_accessible(component_class::kBike, p));
  EXPECT_FALSE(props(osm_way_idx_t{300U}).has_bike_penalty());

  EXPECT_TRUE(bike_route(49.0).has_value());
  EXPECT_FALSE(bike_route(49.02).has_value());
}

// A trunk is no bike way for foot, even though bikes may use it.
TEST_F(foot_access_test, trunk_with_foot_no_is_not_walkable) {
  auto const p = props(osm_way_idx_t{1000U});
  EXPECT_FALSE(p.is_foot_accessible());
  EXPECT_TRUE(p.is_bike_accessible());
  EXPECT_TRUE(p.has_bike_penalty());
  EXPECT_FALSE(is_accessible(component_class::kFoot, p));
  EXPECT_FALSE(route(49.09).has_value());
  EXPECT_TRUE(bike_route(49.09).has_value());
}

TEST_F(foot_access_test, private_barriers) {
  EXPECT_TRUE(route(49.07).has_value());
  EXPECT_TRUE(bike_route(49.07).has_value());
  EXPECT_TRUE(wheelchair_route(49.07).has_value());

  EXPECT_FALSE(route(49.08).has_value());
  EXPECT_FALSE(bike_route(49.08).has_value());
  EXPECT_FALSE(wheelchair_route(49.08).has_value());

  EXPECT_TRUE(route(49.10).has_value());
  EXPECT_FALSE(wheelchair_route(49.10).has_value());
}

}  // namespace osr
