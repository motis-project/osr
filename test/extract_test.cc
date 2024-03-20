#include "gtest/gtest.h"

#include "fmt/core.h"

#include <filesystem>

#include "osr/extract.h"
#include "osr/to_geojson.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  auto const c = config{"test/map.osm", "/tmp/osr_test"};
  osr::extract(c);

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};
  fmt::println("{}", osr::to_geojson(w));
}