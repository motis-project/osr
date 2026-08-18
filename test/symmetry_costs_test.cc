#include <filesystem>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

namespace osr {

TEST(symmetry_costs, way_aware_profiles_price_both_directions_alike) {
  auto const dir = fs::temp_directory_path() / "osr-symmetry-costs-test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(false, "test/station-border.osm.pbf", dir, {});
  auto w = ways{dir, cista::mmap::protection::READ};
  auto const l = lookup{w, dir, cista::mmap::protection::READ};

  auto const from = location{48.7258, 2.25826, kNoLevel};
  auto const to = location{48.7235, 2.25462, kNoLevel};

  for (auto const profile :
       {search_profile::kBus, search_profile::kHgv, search_profile::kRailway}) {
    auto const params = get_parameters(profile);
    auto const fwd = route(params, w, l, profile, from, to, cost_t{100'000U},
                           direction::kForward, 250.0);
    auto const bwd = route(params, w, l, profile, to, from, cost_t{100'000U},
                           direction::kBackward, 250.0);
    ASSERT_TRUE(fwd.has_value()) << to_str(profile);
    ASSERT_TRUE(bwd.has_value()) << to_str(profile);
    EXPECT_EQ(fwd->cost_, bwd->cost_) << to_str(profile);
    EXPECT_EQ(fwd->duration_, bwd->duration_) << to_str(profile);
  }
}

TEST(symmetry_costs, node_penalties_are_charged_in_both_directions) {
  struct testcase {
    std::string_view fixture_;
    search_profile profile_;
    location from_, to_;
  };
  auto const cases = std::vector<testcase>{
      // walks up to an elevator node, no turn costs anywhere
      {"test/station-border.osm.pbf", search_profile::kFoot,
       location{48.7265456, 2.259178, kNoLevel},
       location{48.7252129, 2.2595882, kNoLevel}},
      // ends next to a gate node that charges `private_gate_penalty_`
      {"test/karlsruhe-kit-nord.osm.pbf", search_profile::kBus,
       location{49.1027884, 8.4348879, kNoLevel},
       location{49.1011031, 8.4367592, kNoLevel}}};

  for (auto const& c : cases) {
    auto const dir = fs::temp_directory_path() / "osr-node-penalty-test";
    auto ec = std::error_code{};
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);
    extract(false, std::string{c.fixture_}, dir, {});
    auto w = ways{dir, cista::mmap::protection::READ};
    auto const l = lookup{w, dir, cista::mmap::protection::READ};
    auto const params = get_parameters(c.profile_);

    auto const fwd = route(params, w, l, c.profile_, c.from_, c.to_,
                           cost_t{100'000U}, direction::kForward, 250.0);
    auto const bwd = route(params, w, l, c.profile_, c.to_, c.from_,
                           cost_t{100'000U}, direction::kBackward, 250.0);
    ASSERT_TRUE(fwd.has_value()) << c.fixture_;
    ASSERT_TRUE(bwd.has_value()) << c.fixture_;
    EXPECT_EQ(fwd->cost_, bwd->cost_) << c.fixture_;
    EXPECT_EQ(fwd->duration_, bwd->duration_) << c.fixture_;
  }
}

TEST(symmetry_levels, elevator_endpoint_is_direction_independent) {
  auto const dir = fs::temp_directory_path() / "osr-symmetry-levels-test";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(false, "test/station-border.osm.pbf", dir, {});
  auto w = ways{dir, cista::mmap::protection::READ};
  auto const l = lookup{w, dir, cista::mmap::protection::READ};
  auto const params = get_parameters(search_profile::kFoot);

  auto const from = location{48.7265456, 2.259178, kNoLevel};
  auto const to = location{48.7263761, 2.2576106, kNoLevel};

  auto const fwd = route(params, w, l, search_profile::kFoot, from, to,
                         cost_t{100'000U}, direction::kForward, 250.0);
  auto const bwd = route(params, w, l, search_profile::kFoot, to, from,
                         cost_t{100'000U}, direction::kBackward, 250.0);
  ASSERT_TRUE(fwd.has_value());
  ASSERT_TRUE(bwd.has_value());
  EXPECT_EQ(fwd->cost_, bwd->cost_);
  EXPECT_EQ(fwd->duration_, bwd->duration_);
}

}  // namespace osr
