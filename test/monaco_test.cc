#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>

#include "cista/mmap.h"

#include "fmt/core.h"
#include "fmt/printf.h"
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

constexpr auto const kMaxMatchDistance = 100;

void load(std::string_view tmp, std::string_view data) {
  auto const p = fs::path{tmp};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);
  osr::extract(false, data, tmp, fs::path{});
}

void run(ways const& w,
         lookup const& l,
         unsigned const num_samples,
         unsigned const max_cost) {
  auto prng = std::mt19937{};
  auto dist = std::uniform_int_distribution<std::uint32_t>{0U, w.n_nodes()};
  for (auto i = 0U; i != num_samples; ++i) {
    auto const from = node_idx_t{dist(prng)};
    auto const to = node_idx_t{dist(prng)};
    auto const ert_start = std::chrono::steady_clock::now();
    auto const baseline =
        route(w, l, search_profile::kCar, location{w.get_node_pos(from)},
              location{w.get_node_pos(to)}, max_cost, direction::kForward,
              kMaxMatchDistance, nullptr, nullptr, nullptr);
    auto const ert = std::chrono::steady_clock::now() - ert_start;
    fmt::println(
        "{:11} --> {:11} | {} | ert: {}:{:0>3}:{:0>3} s", w.node_to_osm_[from],
        w.node_to_osm_[to],
        baseline ? fmt::format("cost: {:5} | dist: {:>10.2f}", baseline->cost_,
                             baseline->dist_)
                 : "no result",
        std::chrono::duration_cast<std::chrono::seconds>(ert).count(),
        std::chrono::duration_cast<std::chrono::milliseconds>(ert).count() %
            1000,
        std::chrono::duration_cast<std::chrono::microseconds>(ert).count() %
            1000);
  }
}

TEST(contraction_hierarchies, monaco) {
  constexpr auto const kTmpDir = "/tmp/osr_monaco";
  constexpr auto const kData = "test/monaco.osm.pbf";

  constexpr auto const kNumSamples = 10U;
  constexpr auto const kMaxCost = 3600U;

  load(kTmpDir, kData);
  auto const w = osr::ways{kTmpDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kTmpDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}