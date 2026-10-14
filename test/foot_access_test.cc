#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {

// Separate footways, 1.1 km apart, each of two ways meeting at 8.003:
// - 400, 401: with a gate (node 41)
// - 500, 501: with an open gate (node 51)
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
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

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

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

}  // namespace osr
