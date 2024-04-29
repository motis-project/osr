#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, platform) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(true, "test/ffm_hbf.osm", "/tmp/osr_test");

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};
}