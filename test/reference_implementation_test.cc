#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>

#include "cista/mmap.h"

#include "fmt/core.h"

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
         unsigned const n_samples,
         unsigned const max_cost) {
  auto prng = std::mt19937{};
  auto distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
  auto n_matches = 0U;
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};
  for (auto i = 0U; i != n_samples; ++i) {
    auto const from = node_idx_t{distr(prng)};
    auto const to = node_idx_t{distr(prng)};

    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference =
        route(w, l, search_profile::kCar, location{w.get_node_pos(from)},
              location{w.get_node_pos(to)}, max_cost, direction::kForward,
              kMaxMatchDistance, nullptr, nullptr, nullptr);
    reference_times.emplace_back(std::chrono::steady_clock::now() -
                                 reference_start);

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment =
        route(w, l, search_profile::kCar, location{w.get_node_pos(from)},
              location{w.get_node_pos(to)}, max_cost, direction::kForward,
              kMaxMatchDistance, nullptr, nullptr, nullptr);
    experiment_times.emplace_back(std::chrono::steady_clock::now() -
                                  experiment_start);

    EXPECT_EQ(reference.has_value(), experiment.has_value());
    if (reference && experiment) {
      EXPECT_EQ(reference->cost_, experiment->cost_);
      EXPECT_EQ(reference->dist_, experiment->dist_);
    }

    if (reference.has_value() != experiment.has_value() ||
        (reference && experiment &&
         (reference->cost_ != experiment->cost_ ||
          reference->dist_ != experiment->dist_))) {
      auto const print_result = [&](std::string_view name, auto const& p,
                                    auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: {}:{:0>3}:{:0>3} s", name,
            w.node_to_osm_[from], w.node_to_osm_[to],
            reference ? fmt::format("cost: {:5} | distr: {:>10.2f}",
                                    reference->cost_, reference->dist_)
                      : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() %
                1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() %
                1000);
      };

      print_result("reference", reference, reference_times.back());
      print_result("experiment", experiment, experiment_times.back());
    } else {
      ++n_matches;
    }
  }
  fmt::println(
      "{}/{} ({:3.1f}%) match", n_matches, n_samples,
      (static_cast<double>(n_matches) / static_cast<double>(n_samples)) * 100);
  if (n_matches == n_samples) {
    fmt::println(
        "speedup: {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(experiment_times), end(experiment_times))
                    .count()));
  }
}

TEST(reference_implementation, monaco) {
  constexpr auto const kDataDir = "test/osr_monaco";
  constexpr auto const kNumSamples = 1000U;
  constexpr auto const kMaxCost = 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, hamburg) {
  constexpr auto const kDataDir = "test/osr_hamburg";
  constexpr auto const kNumSamples = 100U;
  constexpr auto const kMaxCost = 2 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, switzerland) {
  constexpr auto const kDataDir = "test/osr_switzerland";

  constexpr auto const kNumSamples = 10U;
  constexpr auto const kMaxCost = 5 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, germany) {
  constexpr auto const kDataDir = "test/osr_germany";

  constexpr auto const kNumSamples = 1U;
  constexpr auto const kMaxCost = 12 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}