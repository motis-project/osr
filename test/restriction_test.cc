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

  osr::extract(false, "test/map.osm", "/tmp/osr_test");

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
  auto const profile = osr::search_profile::kBike;
  auto const max = cost_t{3600};
  auto const p2 = route(w, l, profile, from, to, max, direction::kForward, 100);

  auto const is_restricted = w.r_->is_restricted<osr::direction::kForward>(
      n.value(), w.r_->get_way_pos(n.value(), rhoenring.value()),
      w.r_->get_way_pos(n.value(), arheilger.value()));
  EXPECT_TRUE(is_restricted);

  ASSERT_TRUE(p2.has_value());
  std::cout << "Distance: " << p2->dist_ << "\n";
  for (auto const& segment : p2->segments_) {
    std::cout << "Segment: " << segment.cost_ << "  (";
    std::cout << (segment.from_ != node_idx_t::invalid() ? location{w.get_node_pos(segment.from_), segment.from_level_} : location{{0, 0}, kNoLevel});
    std::cout << " -> ";
    std::cout << (segment.to_!= node_idx_t::invalid() ? location{w.get_node_pos(segment.to_), segment.to_level_} : location{{0, 0}, kNoLevel});
    std::cout << ")  (" << segment.dist_ << ")\n";
  }
}
