#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

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
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(false, "test/map.osm", "/tmp/osr_test", "test/restriction_test_elevation/");

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};
  auto l = osr::lookup{w, "/tmp/osr_test", cista::mmap::protection::READ};

  auto const n = w.find_node_idx(osm_node_idx_t{528944});
  auto const rhoenring = w.find_way(osm_way_idx_t{120682496});
  auto const arheilger = w.find_way(osm_way_idx_t{1068971150});
  // Node Eckhardt / Barkhaus
  auto const n2 = w.find_node_idx(osm_node_idx_t{586157});

  ASSERT_TRUE(n.has_value());
  ASSERT_TRUE(n2.has_value());
  auto const from = location{w.get_node_pos(*n), kNoLevel};
  auto const to = location{w.get_node_pos(*n2), kNoLevel};
  // auto const profile = osr::search_profile::kCar;
  constexpr auto const kMaxCost = cost_t{3600};
  constexpr auto const kMaxMatchDistance = 100;
  // auto const p2 = route(w, l, search_profile::kFoot, from, to, kMaxCost,
  auto const p2 = route(w, l, search_profile::kBike, from, to, kMaxCost,
                        direction::kForward, kMaxMatchDistance);
  auto const p3 = route(w, l, search_profile::kBikeElevationLow, from, to, kMaxCost,
                        direction::kForward, kMaxMatchDistance);

  auto const is_restricted = w.r_->is_restricted<osr::direction::kForward>(
      n.value(), w.r_->get_way_pos(n.value(), rhoenring.value()),
      w.r_->get_way_pos(n.value(), arheilger.value()));
  EXPECT_TRUE(is_restricted);

  constexpr auto const kShortestDistance = 163.0;
  ASSERT_TRUE(p2.has_value());
  ASSERT_TRUE(std::abs(p2->dist_ - kShortestDistance) < 2.0);
  EXPECT_DOUBLE_EQ(3, p2->elevation_up_);
  EXPECT_DOUBLE_EQ(5, p2->elevation_down_);

  ASSERT_TRUE(p3.has_value());
  // std::cout << "DEBUG: " << p2->dist_ << " / " << p2->cost_ << "\n";
  // std::cout << "DEBUG: " << p3->dist_ << " / " << p3->cost_ << "\n";
  // for (auto const x : p3->segments_) std::cout << x.elevation_down_ << ", " << x.elevation_up_ << " / ";
  // std::cout << "\n";
  // for (auto const x : p2->segments_) std::cout << x.from_ << " -> " << x.to_ << " / ";
  // std::cout << "\n";
  // for (auto const x : p3->segments_) std::cout << x.from_ << " -> " << x.to_ << " / ";
  // std::cout << "\n";
  // std::cout << w.get_node_pos(node_idx_t{0}) << " / " << w.get_node_pos(node_idx_t{17}) << "\n";
  ASSERT_TRUE(p3->dist_ - kShortestDistance > 2.0);
  EXPECT_DOUBLE_EQ(1, p3->elevation_up_);
  EXPECT_DOUBLE_EQ(3, p3->elevation_down_);
}
