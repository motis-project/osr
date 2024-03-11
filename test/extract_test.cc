#include "gtest/gtest.h"

#include <filesystem>

#include "osr/extract.h"
#include "osr/meta.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  auto const c = config{"test/map.osm", "/tmp/osr_test"};
  osr::extract(c);

  auto m = meta{c};
  std::cout << m.write() << "\n";
}