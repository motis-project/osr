#include "gtest/gtest.h"

#include <filesystem>

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/dijkstra.h"
#include "osr/extract.h"
#include "osr/lookup.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract("test/map.osm", "/tmp/osr_test");

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};

  auto const n = w.find_node_idx(osm_node_idx_t{528944});
  auto const rhoenring = w.find_way(osm_way_idx_t{120682496});
  auto const arheilger = w.find_way(osm_way_idx_t{1068971150});

  auto const is_restricted =
      w.is_turn_restricted(n.value(), rhoenring.value(), arheilger.value());
  EXPECT_TRUE(is_restricted);
}