#include <gtest/gtest.h>

#include "osr/extract/extract.h"
#include "osr/lookup.h"

namespace fs = std::filesystem;

TEST(bike_parkings, darmstadt_bike_parkings) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(false, "test/darmstadt_bike_parkings.osm", p, {});

  auto w = osr::ways{p, cista::mmap::protection::READ};
}
