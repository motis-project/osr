#include "gtest/gtest.h"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <memory>

#include "geo/latlng.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

// A straight path along the latitude 49.0 (ways 1 and 2, joined at node 3)
// and a detour south (way 3) so that nodes 1, 2 and 3 are routing nodes.
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000"/>
  <node id="3" lat="49.000000" lon="8.001000"/>
  <node id="4" lat="48.999000" lon="8.001000"/>
  <way id="1">
    <nd ref="1"/><nd ref="3"/>
    <tag k="highway" v="path"/><tag k="bicycle" v="yes"/>
  </way>
  <way id="2">
    <nd ref="3"/><nd ref="2"/>
    <tag k="highway" v="path"/><tag k="bicycle" v="yes"/>
  </way>
  <way id="3">
    <nd ref="2"/><nd ref="4"/><nd ref="1"/>
    <tag k="highway" v="path"/><tag k="bicycle" v="yes"/>
  </way>
</osm>
)";

// ~20 m north of the way; the other end lies on the way.
constexpr auto const kOff = geo::latlng{49.000180, 8.000500};
constexpr auto const kOn = geo::latlng{49.000000, 8.001500};

struct must_reach_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-must-reach";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("osr-must-reach", kOsm), dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  template <typename Params>
  static path route_between(Params const& params,
                            search_profile const profile,
                            location const& from,
                            location const& to,
                            direction const dir,
                            routing_algorithm const algo) {
    auto const p =
        route(params, *w_, *l_, profile, from, to, std::chrono::seconds{3600},
              dir, 50.0, nullptr, nullptr, nullptr, algo);
    EXPECT_TRUE(p.has_value());
    return p.value_or(path{});
  }

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

TEST_F(must_reach_test,
       connector_counts_only_when_the_location_must_be_reached) {
  auto const connector = geo::distance(kOff, {49.0, kOff.lng_});
  ASSERT_NEAR(20.0, connector, 0.5);

  for (auto const algo :
       {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      for (auto const off_is_start : {true, false}) {
        auto const run = [&](bool const must_reach) {
          auto const off = location{kOff, kNoLevel, must_reach};
          auto const on = location{kOn, kNoLevel, false};
          return off_is_start
                     ? route_between(
                           foot<false, elevator_tracking>::parameters{},
                           search_profile::kFoot, off, on, dir, algo)
                     : route_between(
                           foot<false, elevator_tracking>::parameters{},
                           search_profile::kFoot, on, off, dir, algo);
        };
        SCOPED_TRACE(::testing::Message()
                     << "algo=" << static_cast<int>(algo) << " dir=" << dir
                     << " off_is_start=" << off_is_start);
        auto const snapped = run(false);
        auto const reached = run(true);

        // Walked at 1.2 m/s; the snapped variant pays nothing for it.
        EXPECT_EQ(snapped.duration_.count() +
                      static_cast<int>(std::round(connector / 1.2)),
                  reached.duration_.count());
        EXPECT_NEAR(snapped.dist_ + connector, reached.dist_, 1.0);
        EXPECT_EQ(snapped.cost_, reached.cost_);

        // Only the reached variant draws the connector to the query position
        // (segment order depends on the search direction, so check both ends).
        auto const off_end_distance = [&](path const& p) {
          return std::min(
              geo::distance(kOff, p.segments_.front().polyline_.front()),
              geo::distance(kOff, p.segments_.back().polyline_.back()));
        };
        EXPECT_NEAR(0.0, off_end_distance(reached), 0.01);
        EXPECT_NEAR(connector, off_end_distance(snapped), 0.5);
      }
    }
  }
}

TEST_F(must_reach_test, connector_is_walked_after_a_bike_leg) {
  using bike_t = bike<bike_costing::kSafe, kElevationNoCost>;
  auto const connector = geo::distance(kOff, {49.0, kOff.lng_});
  auto const snapped =
      route_between(bike_t::parameters{}, search_profile::kBike,
                    location{kOn, kNoLevel}, location{kOff, kNoLevel},
                    direction::kForward, routing_algorithm::kAStarBi);
  auto const reached =
      route_between(bike_t::parameters{}, search_profile::kBike,
                    location{kOn, kNoLevel}, location{kOff, kNoLevel, true},
                    direction::kForward, routing_algorithm::kAStarBi);
  EXPECT_EQ(
      snapped.duration_.count() + static_cast<int>(std::round(connector / 1.2)),
      reached.duration_.count());
}

}  // namespace
}  // namespace osr
