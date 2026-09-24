#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <vector>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/hgv.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

// A loop: way 1 from node 1 (8.000) to node 2 (8.002) along 49.000 (a oneway
// for cars, west to east), back via nodes 4 and 3 south of it. A second loop
// further north: way 3 along 49.002 is too low for an hgv (maxheight 3 m),
// the way back via 49.003 is not.
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000"/>
  <node id="3" lat="48.999000" lon="8.000000"/>
  <node id="4" lat="48.999000" lon="8.002000"/>
  <way id="1">
    <nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="residential"/><tag k="oneway" v="yes"/>
  </way>
  <way id="2">
    <nd ref="2"/><nd ref="4"/><nd ref="3"/><nd ref="1"/>
    <tag k="highway" v="residential"/>
  </way>
  <node id="5" lat="49.002000" lon="8.000000"/>
  <node id="6" lat="49.002000" lon="8.002000"/>
  <node id="7" lat="49.003000" lon="8.000000"/>
  <node id="8" lat="49.003000" lon="8.002000"/>
  <way id="3">
    <nd ref="5"/><nd ref="6"/>
    <tag k="highway" v="residential"/><tag k="maxheight" v="3"/>
  </way>
  <way id="4">
    <nd ref="6"/><nd ref="8"/><nd ref="7"/><nd ref="5"/>
    <tag k="highway" v="residential"/>
  </way>
</osm>
)";

// Both 10 m north of way 1, 73 m apart along it, between the same two nodes.
constexpr auto const kWest = geo::latlng{49.00009, 8.0005};
constexpr auto const kEast = geo::latlng{49.00009, 8.0015};

// The same next to way 3.
constexpr auto const kLowWest = geo::latlng{49.00209, 8.0005};
constexpr auto const kLowEast = geo::latlng{49.00209, 8.0015};

struct same_stretch_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-same-stretch";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("osr-same-stretch", kOsm), dir_, {});
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
  static std::optional<path> route_between(Params const& params,
                                           search_profile const profile,
                                           geo::latlng const& from,
                                           geo::latlng const& to,
                                           direction const dir,
                                           routing_algorithm const algo) {
    return route(params, *w_, *l_, profile, location{from, kNoLevel},
                 location{to, kNoLevel}, std::chrono::seconds{3600}, dir, 50.0,
                 nullptr, nullptr, nullptr, algo);
  }

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

TEST_F(same_stretch_test, foot_walks_directly_along_the_stretch) {
  for (auto const algo :
       {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      for (auto const west_to_east : {true, false}) {
        SCOPED_TRACE(::testing::Message()
                     << "algo=" << static_cast<int>(algo) << " dir=" << dir
                     << " west_to_east=" << west_to_east);
        auto const p =
            route_between(foot<false, elevator_tracking>::parameters{},
                          search_profile::kFoot, west_to_east ? kWest : kEast,
                          west_to_east ? kEast : kWest, dir, algo);
        ASSERT_TRUE(p.has_value());
        // Not via node 1 or 2 and back (146 m).
        EXPECT_NEAR(73.0, p->dist_, 1.0);
        ASSERT_EQ(1U, p->segments_.size());
        EXPECT_EQ(61, p->duration_.count());
      }
    }
  }
}

TEST_F(same_stretch_test, must_reach_includes_the_connector) {
  auto const connector = geo::distance(kWest, {49.0, kWest.lng_});
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    SCOPED_TRACE(::testing::Message() << "dir=" << dir);
    auto const p =
        route(foot<false, elevator_tracking>::parameters{}, *w_, *l_,
              search_profile::kFoot, location{kWest, kNoLevel, true},
              location{kEast, kNoLevel}, std::chrono::seconds{3600}, dir, 50.0,
              nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(1U, p->segments_.size());
    EXPECT_NEAR(73.0 + connector, p->dist_, 1.0);
    auto const& polyline = p->segments_.front().polyline_;
    EXPECT_NEAR(0.0,
                std::min(geo::distance(kWest, polyline.front()),
                         geo::distance(kWest, polyline.back())),
                0.01);
  }
}

TEST_F(same_stretch_test, car_respects_the_oneway) {
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    SCOPED_TRACE(::testing::Message() << "dir=" << dir);
    // A backward search starts at the destination of travel.
    auto const drive = [&](geo::latlng const& a, geo::latlng const& b) {
      return dir == direction::kForward
                 ? route_between(car::parameters{}, search_profile::kCar, a, b,
                                 dir, routing_algorithm::kDijkstra)
                 : route_between(car::parameters{}, search_profile::kCar, b, a,
                                 dir, routing_algorithm::kDijkstra);
    };

    auto const with = drive(kWest, kEast);
    ASSERT_TRUE(with.has_value());
    EXPECT_NEAR(73.0, with->dist_, 1.0);

    // Against the oneway: around the loop, not straight back along way 1.
    auto const against = drive(kEast, kWest);
    ASSERT_TRUE(against.has_value());
    EXPECT_GT(against->dist_, 400.0);
  }
}

TEST_F(same_stretch_test, hgv_respects_oneway_and_dimensions) {
  for (auto const dir : {direction::kForward, direction::kBackward}) {
    SCOPED_TRACE(::testing::Message() << "dir=" << dir);
    auto const drive = [&](auto const& params, search_profile const profile,
                           geo::latlng const& a, geo::latlng const& b) {
      return dir == direction::kForward
                 ? route_between(params, profile, a, b, dir,
                                 routing_algorithm::kDijkstra)
                 : route_between(params, profile, b, a, dir,
                                 routing_algorithm::kDijkstra);
    };

    auto const with =
        drive(hgv::parameters{}, search_profile::kHgv, kWest, kEast);
    ASSERT_TRUE(with.has_value());
    EXPECT_NEAR(73.0, with->dist_, 1.0);
    auto const against =
        drive(hgv::parameters{}, search_profile::kHgv, kEast, kWest);
    ASSERT_TRUE(against.has_value());
    EXPECT_GT(against->dist_, 400.0);

    // Way 3 is too low for the hgv: no direct piece along it, unlike a car.
    auto const car_low =
        drive(car::parameters{}, search_profile::kCar, kLowWest, kLowEast);
    ASSERT_TRUE(car_low.has_value());
    EXPECT_NEAR(73.0, car_low->dist_, 1.0);
    auto const hgv_low =
        drive(hgv::parameters{}, search_profile::kHgv, kLowWest, kLowEast);
    ASSERT_TRUE(hgv_low.has_value());
    EXPECT_GT(hgv_low->dist_, 150.0);
    for (auto const& s : hgv_low->segments_) {
      EXPECT_TRUE(s.way_ == way_idx_t::invalid() ||
                  w_->get_osm_way(s.way_) != std::optional<std::int64_t>{3});
    }
  }
}

TEST_F(same_stretch_test, one_to_many_uses_the_stretch) {
  auto const to = std::vector<location>{location{kEast, kNoLevel},
                                        location{{48.99995, 8.001}, kNoLevel}};
  auto const results =
      route(foot<false, elevator_tracking>::parameters{}, *w_, *l_,
            search_profile::kFoot, location{kWest, kNoLevel}, to,
            std::chrono::seconds{3600}, direction::kForward, 50.0, nullptr,
            nullptr, nullptr, [](path const&) { return true; });
  ASSERT_EQ(2U, results.size());
  ASSERT_TRUE(results[0].has_value());
  EXPECT_NEAR(73.0, results[0]->dist_, 1.0);
  ASSERT_TRUE(results[1].has_value());
  EXPECT_LT(results[1]->dist_, 60.0);
}

// Both ends 67 m north of way 10, 143 m apart along it. The closest ways are
// 11 m away, each on its own short way (11 and 12), connected through a 2 km
// detour to the north (13). Way 10 is a separate component, so it stays a
// candidate, but with a high matching penalty: the direct piece along it is
// fast (~2 min) but more expensive than the ~31 min detour.
constexpr auto const kDetourOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="50.000000" lon="9.000000"/>
  <node id="2" lat="50.000000" lon="9.010000"/>
  <node id="3" lat="49.999000" lon="9.000000"/>
  <node id="4" lat="49.999000" lon="9.010000"/>
  <way id="10">
    <nd ref="1"/><nd ref="2"/><tag k="highway" v="footway"/>
  </way>
  <way id="14">
    <nd ref="1"/><nd ref="3"/><tag k="highway" v="footway"/>
  </way>
  <way id="15">
    <nd ref="2"/><nd ref="4"/><tag k="highway" v="footway"/>
  </way>
  <node id="11" lat="50.000700" lon="9.003500"/>
  <node id="12" lat="50.000700" lon="9.004500"/>
  <node id="13" lat="50.000700" lon="9.005500"/>
  <node id="14" lat="50.000700" lon="9.006500"/>
  <node id="15" lat="50.010000" lon="9.004500"/>
  <node id="16" lat="50.010000" lon="9.005500"/>
  <way id="11">
    <nd ref="11"/><nd ref="12"/><tag k="highway" v="footway"/>
  </way>
  <way id="12">
    <nd ref="13"/><nd ref="14"/><tag k="highway" v="footway"/>
  </way>
  <way id="13">
    <nd ref="12"/><nd ref="15"/><nd ref="16"/><nd ref="13"/>
    <tag k="highway" v="footway"/>
  </way>
</osm>
)";

struct same_stretch_detour_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-same-stretch-detour";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("osr-same-stretch-detour", kDetourOsm),
            dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

constexpr auto const kDetourFrom = geo::latlng{50.0006, 9.004};
constexpr auto const kDetourTo = geo::latlng{50.0006, 9.006};

// The cheapest route is the detour. Within its duration it is returned, and
// below it there is no route: the faster but more expensive direct piece must
// not be returned instead, although it is within the limit.
TEST_F(same_stretch_detour_test, cheaper_route_over_the_limit_wins) {
  for (auto const algo :
       {routing_algorithm::kDijkstra, routing_algorithm::kAStarBi}) {
    for (auto const dir : {direction::kForward, direction::kBackward}) {
      auto const bwd = dir == direction::kBackward;
      auto const route_with = [&](std::chrono::seconds const max_duration) {
        return route(foot<false, elevator_tracking>::parameters{}, *w_, *l_,
                     search_profile::kFoot,
                     location{bwd ? kDetourTo : kDetourFrom, kNoLevel},
                     location{bwd ? kDetourFrom : kDetourTo, kNoLevel},
                     max_duration, dir, 100.0, nullptr, nullptr, nullptr, algo);
      };
      SCOPED_TRACE(::testing::Message()
                   << "algo=" << static_cast<int>(algo) << " dir=" << dir);

      auto const detour = route_with(std::chrono::seconds{3600});
      ASSERT_TRUE(detour.has_value());
      EXPECT_GT(detour->dist_, 2000.0);
      EXPECT_GT(detour->duration_.count(), 1800);

      EXPECT_FALSE(route_with(std::chrono::seconds{1800}).has_value());

      auto const many =
          route(foot<false, elevator_tracking>::parameters{}, *w_, *l_,
                search_profile::kFoot,
                location{bwd ? kDetourTo : kDetourFrom, kNoLevel},
                std::vector{location{bwd ? kDetourFrom : kDetourTo, kNoLevel}},
                std::chrono::seconds{1800}, dir, 100.0, nullptr, nullptr,
                nullptr, [](path const&) { return true; });
      ASSERT_EQ(1U, many.size());
      EXPECT_FALSE(many.front().has_value());
    }
  }
}

}  // namespace
}  // namespace osr
