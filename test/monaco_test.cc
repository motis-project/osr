#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>

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

constexpr auto const kNumSamples = 10;

TEST(contraction_hierarchies, monaco) {
  // load data
  constexpr auto const kMonacoDir = "/tmp/osr_monaco";
  auto const p = fs::path{kMonacoDir};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);
  osr::extract(false, "test/monaco.osm.pbf", kMonacoDir, fs::path{});
  auto const w = osr::ways{kMonacoDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kMonacoDir, cista::mmap::protection::READ};

  // PRNG
  auto prng = std::mt19937{};
  auto dist = std::uniform_int_distribution<std::uint32_t>{0U, w.n_nodes()};

  for(auto const i : std::views::iota(0,kNumSamples)) {
    fmt::println("{}",i);
  }

}