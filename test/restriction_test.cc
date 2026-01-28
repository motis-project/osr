#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(false, "test/map.osm", p, "test/restriction_test_elevation/");

  auto w = osr::ways{p, cista::mmap::protection::READ};
  auto l = osr::lookup{w, p, cista::mmap::protection::READ};
  auto const elevations = elevation_storage::try_open(p);
  ASSERT_TRUE(elevations);

  auto const n = w.find_node_idx(osm_node_idx_t{528944});
  auto const rhoenring = w.find_way(osm_way_idx_t{120682496});
  auto const arheilger = w.find_way(osm_way_idx_t{1068971150});
  // Crossing Eckhardt / Barkhaus
  auto const n_dst = w.find_node_idx(osm_node_idx_t{586157});

  ASSERT_TRUE(n.has_value());
  ASSERT_TRUE(n_dst.has_value());
  auto const from = location{w.get_node_pos(*n), kNoLevel};
  auto const to = location{w.get_node_pos(*n_dst), kNoLevel};
  constexpr auto const kMaxCost = cost_t{3600};
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const kParamsNoCosts =
      bike<bike_costing::kSafe, kElevationNoCost>::parameters{};
  constexpr auto const kParamsHighCosts =
      bike<bike_costing::kSafe, kElevationHighCost>::parameters{};
  auto const route_no_costs =
      route(kParamsNoCosts, w, l, search_profile::kBike, from, to, kMaxCost,
            direction::kForward, kMaxMatchDistance, nullptr, nullptr,
            elevations.get());
  auto const route_high_costs =
      route(kParamsHighCosts, w, l, search_profile::kBikeElevationHigh, from,
            to, kMaxCost, direction::kForward, kMaxMatchDistance, nullptr,
            nullptr, elevations.get());

  auto const is_restricted = w.r_->is_restricted<osr::direction::kForward>(
      n.value(), w.r_->get_way_pos(n.value(), rhoenring.value()),
      w.r_->get_way_pos(n.value(), arheilger.value()));
  EXPECT_TRUE(is_restricted);

  constexpr auto const kShortestDistance = 163.0;
  ASSERT_TRUE(route_no_costs.has_value());
  EXPECT_TRUE(std::abs(route_no_costs->dist_ - kShortestDistance) < 2.0);
  // Upper bounds for elevations on each segment
  EXPECT_EQ(elevation_monotonic_t{4U + 1U}, route_no_costs->elevation_.up_);
  EXPECT_EQ(elevation_monotonic_t{0U + 6U}, route_no_costs->elevation_.down_);

  ASSERT_TRUE(route_high_costs.has_value());
  EXPECT_TRUE(route_high_costs->dist_ - kShortestDistance > 2.0);
  // Upper bounds for elevations on each segment
  EXPECT_EQ(elevation_monotonic_t{1U + 0U}, route_high_costs->elevation_.up_);
  EXPECT_EQ(elevation_monotonic_t{4U + 0U}, route_high_costs->elevation_.down_);
}
