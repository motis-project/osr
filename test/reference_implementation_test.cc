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
#include "utl/parallel_for.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kMaxMatchDistance = 100;

void load(std::string_view raw_data, std::string_view data_dir) {
  if (!fs::exists(data_dir)) {
    if (fs::exists(raw_data)) {
      auto const p = fs::path{data_dir};
      auto ec = std::error_code{};
      fs::remove_all(p, ec);
      fs::create_directories(p, ec);
      osr::extract(false, raw_data, data_dir, fs::path{});
    }
  }
}

void run(ways const& w,
         lookup const& l,
         unsigned const n_samples,
         unsigned const max_cost) {

  auto const from_tos = [&]() {
    auto prng = std::mt19937{};
    auto distr =
        std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto from_tos = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    for (auto i = 0U; i != n_samples; ++i) {
      from_tos.emplace_back(distr(prng), distr(prng));
    }
    return from_tos;
  }();

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto reference_times = std::vector<std::chrono::steady_clock::duration>{};
  auto experiment_times = std::vector<std::chrono::steady_clock::duration>{};

  auto m = std::mutex{};
  utl::parallel_for(from_tos, [&](std::pair<node_idx_t, node_idx_t> const
                                      from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w.get_node_pos(to_node)};

    auto const node_pinned_matches = [&](location const& loc,
                                         node_idx_t const n) {
      auto matches = l.match<car>(loc, false, direction::kForward,
                                  kMaxMatchDistance, nullptr);
      std::erase_if(matches, [&](auto const& wc) {
        return wc.left_.node_ != n && wc.right_.node_ != n;
      });
      return matches;
    };
    auto const from_matches = node_pinned_matches(from_loc, from_node);
    auto const to_matches = node_pinned_matches(to_loc, to_node);
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference_path =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr);
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment_path =
        route(w, l, search_profile::kCar, from_loc, to_loc, from_matches_span,
              to_matches_span, max_cost, direction::kForward, nullptr, nullptr,
              nullptr);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

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

      print_result("reference_path", reference_path, reference_time);
      print_result("experiment_path", experiment_path, experiment_time);
    } else {
      ++n_congruent;
    }

    auto const guard = std::lock_guard{m};
    reference_times.emplace_back(reference_time);
    experiment_times.emplace_back(experiment_time);
  });

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;

  fmt::println("congruent on non-empty start/dest matches: {}/{} ({:3.1f}%)",
               non_empty_congruent, non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) *
                   100);
}

TEST(reference_implementation, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 10000U;
  auto const max_cost = 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(reference_implementation, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 5000U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(reference_implementation, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 1000U;
  auto const max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}

TEST(reference_implementation, germany) {
  auto const raw_data = "test/germany.osm.pbf";
  auto const data_dir = "test/germany";
  constexpr auto const num_samples = 50U;
  constexpr auto const max_cost = 12 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run(w, l, num_samples, max_cost);
}