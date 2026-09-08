#ifdef _WIN32
#include "windows.h"
#endif

#include "gmock/gmock-more-matchers.h"
#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;
using std::string_view;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;
constexpr auto const kMaxAllowedPathDifferenceRatio = 0.5;

namespace {
void load(string_view raw_data, std::string_view data_dir) {
  if (fs::exists(raw_data)) {
    auto const p = fs::path{data_dir};
    auto ec = std::error_code{};
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
}

MATCHER(LatLngMatchesE5, "matches latlng") {
  const auto& [actual, expected] = arg;

  return testing::ExplainMatchResult(
      testing::AllOf(
          testing::Property("lat", &geo::latlng::lat,
                            testing::DoubleNear(expected.lat(), 10e-5)),
          testing::Property("lng", &geo::latlng::lng,
                            testing::DoubleNear(expected.lng(), 10e-5))),
      actual, result_listener);
}
}  // namespace

TEST(car_parking, monaco_fwd) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto constexpr dir = direction::kForward;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto const start = geo::latlng(43.728311, 7.417984);
  auto const end = geo::latlng(43.730710, 7.414288);
  auto const start_loc = location{.pos_ = start, .lvl_ = kNoLevel};
  auto const end_loc = location{.pos_ = end, .lvl_ = kNoLevel};
  auto const max_cost = cost_t{900};
  auto const max_matching_dist = 250.0;

  using P = car_parking<false, true>;
  // using P = car;
  auto const res =
      route(P::parameters{}, w, l, search_profile::kCarParking, start_loc,
            end_loc, max_cost, direction::kForward, max_matching_dist, nullptr,
            nullptr, nullptr, routing_algorithm::kDijkstra);

  ASSERT_TRUE(res.has_value());
  EXPECT_NEAR(res->dist_, 2'261, 0.5);
  EXPECT_EQ(res->duration_, duration_t{587});
  ASSERT_EQ(res->segments_.size(), 42);
  auto const& parking_segment = res->segments_[40];
  EXPECT_EQ(parking_segment.mode_, mode::kParking);
  fmt::println("Polyline: >>{}<<", parking_segment.polyline_);
  auto const expected_polyline = geo::polyline{
      // Path to closest point on road way
      {43.7301, 7.41293},
      {43.7301, 7.41291},
      {43.7301, 7.41289},
      {43.7301, 7.41289},
      {43.7302, 7.41289},
      {43.7302, 7.41292},
      {43.7302, 7.41294},
      {43.7302, 7.41297},
      {43.7302, 7.41303},
      {43.7302, 7.41308},
      {43.7302, 7.41316},
      {43.7304, 7.41369},
      {43.7305, 7.41391},
      {43.7305, 7.41401},
      {43.7305, 7.41417},
      {43.7306, 7.41425},
      {43.7306, 7.41432},
      {43.7306, 7.41435},
      {43.7307, 7.41436},
      {43.7307, 7.41436},
      {43.7307, 7.41436},
      {43.7307, 7.41434},
      {43.7307, 7.41431},
      {43.7308, 7.41427},
      {43.7308, 7.41422},
      {43.7308, 7.41416},
      {43.7306, 7.41387},  // TODO Duplicate
      // Path to parking site and back
      {43.7306, 7.41387},
      {43.7306, 7.4139},
      {43.7306, 7.4139},  // TODO Duplicate
      {43.7306, 7.41387},
      // Path to closest point on footpath way
      {43.7306, 7.41387},
      {43.7305, 7.41358},
      {43.7305, 7.41350},
      {43.7305, 7.41341},
      {43.7304, 7.41315},
      {43.7304, 7.41311},
  };
  EXPECT_EQ(parking_segment.polyline_.size(), expected_polyline.size());
  EXPECT_THAT(parking_segment.polyline_,
              testing::Pointwise(LatLngMatchesE5(), expected_polyline));
}
