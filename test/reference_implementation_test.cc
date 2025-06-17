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
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
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
  auto n_congruent = 0U;
  auto n_empty_matches = 0U;
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};

  auto const node_pinned_matches = [&](location const& loc,
                                       node_idx_t const n) {
    auto matches = l.match<car>(loc, false, direction::kForward,
                                kMaxMatchDistance, nullptr);
    std::erase_if(matches, [&](auto const& wc) {
      return wc.left_.node_ != n && wc.right_.node_ != n;
    });
    return matches;
  };

  for (auto i = 0U; i != n_samples; ++i) {
    auto const from_node = node_idx_t{distr(prng)};
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const from_matches = node_pinned_matches(from_loc, from_node);
    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};

    auto const to_node = node_idx_t{distr(prng)};
    auto const to_loc = location{w.get_node_pos(to_node)};
    auto const to_matches = node_pinned_matches(to_loc, to_node);
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference_path =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr);
    reference_times.emplace_back(std::chrono::steady_clock::now() -
                                 reference_start);

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment_path =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr);
    experiment_times.emplace_back(std::chrono::steady_clock::now() -
                                  experiment_start);

    EXPECT_EQ(reference_path.has_value(), experiment_path.has_value());
    if (reference_path && experiment_path) {
      EXPECT_EQ(reference_path->cost_, experiment_path->cost_);
      EXPECT_EQ(reference_path->dist_, experiment_path->dist_);
    }

    if (reference_path.has_value() != experiment_path.has_value() ||
        (reference_path && experiment_path &&
         (reference_path->cost_ != experiment_path->cost_ ||
          reference_path->dist_ != experiment_path->dist_))) {
      auto const print_result = [&](std::string_view name, auto const& p,
                                    auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: {}:{:0>3}:{:0>3} s", name,
            w.node_to_osm_[from_node], w.node_to_osm_[to_node],
            reference_path
                ? fmt::format("cost: {:5} | distr: {:>10.2f}",
                              reference_path->cost_, reference_path->dist_)
                : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() %
                1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() %
                1000);
      };

      print_result("reference_path", reference_path, reference_times.back());
      print_result("experiment_path", experiment_path, experiment_times.back());
    } else {
      ++n_congruent;
    }
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;

  fmt::println("congruent on non-empty start/dest matches: {}/{} ({:3.1f}%)",
               non_empty_congruent, non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) *
                   100);
  if (n_congruent == n_samples) {
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
  if (!fs::exists(kDataDir)) {
    GTEST_SKIP() << kDataDir << " does not exist";
  }

  constexpr auto const kNumSamples = 10000U;
  constexpr auto const kMaxCost = 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, hamburg) {
  constexpr auto const kDataDir = "test/osr_hamburg";
  if (!fs::exists(kDataDir)) {
    GTEST_SKIP() << kDataDir << " does not exist";
  }

  constexpr auto const kNumSamples = 1000U;
  constexpr auto const kMaxCost = 2 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, switzerland) {
  constexpr auto const kDataDir = "test/osr_switzerland";
  if (!fs::exists(kDataDir)) {
    GTEST_SKIP() << kDataDir << " does not exist";
  }

  constexpr auto const kNumSamples = 100U;
  constexpr auto const kMaxCost = 5 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}

TEST(reference_implementation, germany) {
  constexpr auto const kDataDir = "test/osr_germany";
  if (!fs::exists(kDataDir)) {
    GTEST_SKIP() << kDataDir << " does not exist";
  }

  constexpr auto const kNumSamples = 50U;
  constexpr auto const kMaxCost = 12 * 3600U;

  auto const w = osr::ways{kDataDir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, kDataDir, cista::mmap::protection::READ};

  run(w, l, kNumSamples, kMaxCost);
}