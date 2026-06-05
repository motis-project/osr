#include "gtest/gtest.h"

#include "osr/elevation_storage.h"
#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"

namespace fs = std::filesystem;
using namespace osr;
using elevation = elevation_storage::elevation;
using mono = elevation_monotonic_t;

// tobler_speed = speed * exp(-3.5 * |grad + (0.05 ? has_elevation : 0)|)
static float factor(float dist, elevation e = {}, bool exist = true) {
  auto const dx = std::max(to_idx(e.up_), to_idx(e.down_));
  auto const grad =
      dist > 0U ? static_cast<float>(dx) / static_cast<float>(dist) : 0.F;
  return std::exp(-3.5F * std::abs(grad + (!exist ? 0 : 0.05F)));
}

TEST(elevation_cost, no_storage_zero_cost) {
  auto f = factor(100.F, {}, false);
  EXPECT_EQ(1, f);
}
TEST(elevation_cost, no_elevation_small_cost) {
  auto f = factor(100, {});
  EXPECT_NEAR(1, f, 0.2);
}

TEST(elevation_cost, ascend_descend_eq) {
  auto u = factor(100.F, {mono{5}, mono{0}});
  auto d = factor(100.F, {mono{0}, mono{5}});
  EXPECT_EQ(u, d);
}

TEST(elevation_cost, alpspitze) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);
  osr::extract(false, "test/garmisch.osm.pbf", p, "test/elevation_cost_test");
  auto w = osr::ways{p, cista::mmap::protection::READ};
  auto l = osr::lookup{w, p, cista::mmap::protection::READ};
  auto const elevations = elevation_storage::try_open(p);
  ASSERT_TRUE(elevations);
  auto const from = location{{47.4845447, 11.0732626}, kNoLevel};
  auto const to = location{{47.4223324, 11.0633797}, kNoLevel};
  auto const param1 = get_parameters(osr::search_profile::kFoot);
  auto const param2 = get_parameters(osr::search_profile::kFootNoElevation);

  auto const path_no_elevation =
      route(param2, w, l, search_profile::kFootNoElevation, from, to,
            360000 * 5, direction::kForward, 400, nullptr, nullptr, nullptr);
  auto const path_uphill =
      route(param1, w, l, search_profile::kFoot, from, to, 360000 * 5,
            direction::kForward, 400, nullptr, nullptr, elevations.get());
  auto const path_downhill =
      route(param1, w, l, search_profile::kFoot, to, from, 360000 * 5,
            direction::kForward, 400, nullptr, nullptr, elevations.get());

  ASSERT_TRUE(path_no_elevation);
  ASSERT_TRUE(path_uphill);
  EXPECT_EQ(path_no_elevation->dist_, path_uphill->dist_);
  EXPECT_EQ(path_no_elevation->cost_, 10977U);  // 3h
  EXPECT_EQ(path_uphill->cost_, 19075U);  // 5.3h
  EXPECT_EQ(path_downhill->cost_, 19075U);  // 5.3h
}
