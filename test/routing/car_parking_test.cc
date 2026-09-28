#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <cmath>
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

geo::polyline round(geo::polyline const& polyline) {
  auto rounded = geo::polyline{};
  rounded.reserve(polyline.size());
  auto const round = [&](double const d) {
    return std::round(d * 100'000) / 100'000.0;
  };
  for (auto& p : polyline) {
    rounded.emplace_back(round(p.lat()), round(p.lng()));
  }
  return rounded;
}
}  // namespace

TEST(car_parking, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto constexpr dir = direction::kForward;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // Forward search, curved path to parking space
  {
    auto const start = geo::latlng{.lat_ = 43.729852, .lng_ = 7.413038};
    auto const end = geo::latlng{.lat_ = 43.730710, .lng_ = 7.414288};
    auto const start_loc = location{.pos_ = start, .lvl_ = kNoLevel};
    auto const end_loc = location{.pos_ = end, .lvl_ = kNoLevel};
    auto const max_cost = cost_t{900};
    auto const max_matching_dist = 250.0;

    using P = car_parking<false, true>;
    auto const res =
        route(P::parameters{}, w, l, search_profile::kCarParking, start_loc,
              end_loc, max_cost, direction::kForward, max_matching_dist,
              nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);

    ASSERT_TRUE(res.has_value());
    EXPECT_NEAR(res->dist_, 772, 0.5);
    EXPECT_EQ(res->duration_, duration_t{455});
    ASSERT_EQ(res->segments_.size(), 10);
    auto const& parking_segment = res->segments_[8];
    EXPECT_EQ(parking_segment.mode_, mode::kParking);
    fmt::println("Polyline: >>{}<<", parking_segment.polyline_);
    auto const expected_polyline = geo::polyline{
        // Path to closest point on road way
        {43.73008, 7.41293},
        {43.73009, 7.41291},
        {43.73011, 7.41289},
        {43.73014, 7.41289},
        {43.73016, 7.41289},
        {43.73019, 7.41292},
        {43.73020, 7.41294},
        {43.73021, 7.41297},
        {43.73022, 7.41303},
        {43.73022, 7.41308},
        {43.73023, 7.41316},
        {43.73041, 7.41369},
        {43.73049, 7.41391},
        {43.73051, 7.41401},
        {43.73054, 7.41417},
        {43.73057, 7.41425},
        {43.73060, 7.41432},
        {43.73063, 7.41435},
        {43.73066, 7.41436},
        {43.73069, 7.41436},
        {43.73071, 7.41436},
        {43.73073, 7.41434},
        {43.73075, 7.41431},
        {43.73076, 7.41427},
        {43.73077, 7.41422},
        {43.73076, 7.41416},
        // Path to parking site and back
        {43.73064, 7.41387},
        {43.73061, 7.4139},
        {43.73064, 7.41387},
        // Path to closest point on footpath way
        {43.73053, 7.41358},
        {43.73051, 7.41350},
        {43.73049, 7.41341},
        {43.73043, 7.41315},
        {43.73042, 7.41311},
    };
    EXPECT_EQ(round(parking_segment.polyline_), expected_polyline);
  }
}
