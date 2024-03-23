#include "gtest/gtest.h"

#include <filesystem>

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/dijkstra.h"
#include "osr/extract.h"
#include "osr/lookup.h"
#include "osr/to_geojson.h"
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

  auto s = dijkstra_state{};
  auto l = lookup{w};

  auto const max_dist = 900;
  auto r = std::vector<start_dist>{};
  l.find({49.87810348736298, 8.653064959841231}, r);
}