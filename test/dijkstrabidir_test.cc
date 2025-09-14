#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>
#include <atomic>
#include <mutex>
#include <span>
#include <numeric>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kUseMultithreading = true;
constexpr auto const kPrintDebugGeojson = false;
constexpr auto const kMaxMatchDistance = 100;     // meters
constexpr auto const kMaxAllowedPathDifferenceRatio = 0.5;

static void load(std::string_view raw_data, std::string_view data_dir) {
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

// Compare classic Dijkstra vs bidirectional Dijkstra (kBiDijkstra) for consistency.
static void run_bidijkstra_consistency(ways const& w,
                                       lookup const& l,
                                       unsigned const n_samples,
                                       unsigned const max_cost) {
  auto const pairs = [&]() {
    auto prng = std::mt19937{};
    auto distr = std::uniform_int_distribution<std::uint32_t>{0, w.n_nodes() - 1};
    auto v = std::vector<std::pair<node_idx_t, node_idx_t>>{};
    v.reserve(n_samples);
    for (auto i = 0U; i != n_samples; ++i) {
      v.emplace_back(distr(prng), distr(prng));
    }
    return v;
  }();

  auto n_congruent = std::atomic<unsigned>{0U};
  auto n_empty_matches = std::atomic<unsigned>{0U};
  auto baseline_times = std::vector<std::chrono::steady_clock::duration>{};
  auto bidir_times = std::vector<std::chrono::steady_clock::duration>{};
  auto m = std::mutex{};

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const to_node = from_to.second;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_loc = location{w.get_node_pos(to_node)};

    auto const node_pinned_matches = [&](location const& loc, node_idx_t const n, bool const reverse) {
      auto matches = l.match<car>(car::parameters{}, loc, reverse, direction::kForward, kMaxMatchDistance, nullptr);
      std::erase_if(matches, [&](auto const& wc) { return wc.left_.node_ != n && wc.right_.node_ != n; });
      return matches;  // keep all (usually 0-1)
    };

    auto from_matches = node_pinned_matches(from_loc, from_node, false);
    auto to_matches = node_pinned_matches(to_loc, to_node, true);

    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_span = std::span{begin(from_matches), end(from_matches)};
    auto const to_span = std::span{begin(to_matches), end(to_matches)};

    // Baseline classic Dijkstra (forward search)
    auto const baseline_start = std::chrono::steady_clock::now();
    auto const baseline_route = route(car::parameters{}, w, l, search_profile::kCar, from_loc, to_loc, from_span, to_span, max_cost, direction::kForward, nullptr, nullptr, nullptr, routing_algorithm::kDijkstra);
    auto const baseline_time = std::chrono::steady_clock::now() - baseline_start;

    // Bidirectional Dijkstra implementation
    auto const bidir_start = std::chrono::steady_clock::now();
    auto const bidir_route = route(car::parameters{}, w, l, search_profile::kCar, from_loc, to_loc, from_span, to_span, max_cost, direction::kForward, nullptr, nullptr, nullptr, routing_algorithm::kBiDijkstra);
    auto const bidir_time = std::chrono::steady_clock::now() - bidir_start;

    auto congruent = false;
    if (baseline_route.has_value() != bidir_route.has_value()) {
      congruent = false;
    } else if (baseline_route && bidir_route) {
      congruent = baseline_route->cost_ == bidir_route->cost_;
    } else {
      congruent = true; // both infeasible
    }

    if (!congruent) {
      auto const print_result = [&](std::string_view name, auto const& p, auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: {}:{:0>3}:{:0>3} s", name,
            w.node_to_osm_[from_node], w.node_to_osm_[to_node],
            p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_) : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() % 1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() % 1000);
        if (p.has_value() && kPrintDebugGeojson) {
          fmt::println("{}\n", to_featurecollection(w, p));
        }
      };
      print_result("dijkstra", baseline_route, baseline_time);
      print_result("bi-dijk", bidir_route, bidir_time);
    } else {
      ++n_congruent;
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      baseline_times.emplace_back(baseline_time);
      bidir_times.emplace_back(bidir_time);
    }
  };

  if (kUseMultithreading) {
    utl::parallel_for(pairs, single_run);
  } else {
    for (auto const& p : pairs) { single_run(p); }
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;

  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("dijkstra vs bidir-dijkstra congruent (non-empty): {}/{} ({:3.1f}%)", non_empty_congruent, non_empty_samples,
               non_empty_samples == 0 ? 0.0 : (static_cast<double>(non_empty_congruent) / non_empty_samples) * 100.0);
  if (non_empty_congruent == non_empty_samples && !baseline_times.empty()) {
    fmt::println("avg speed ratio (baseline/bidir): {:.2f}",
                 static_cast<double>(std::reduce(begin(baseline_times), end(baseline_times)).count()) /
                     static_cast<double>(std::reduce(begin(bidir_times), end(bidir_times)).count()));
  }
}

TEST(bidirectional_dijkstra, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const num_samples = 5000U;
  auto const max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_bidijkstra_consistency(w, l, num_samples, max_cost);
}

TEST(bidirectional_dijkstra, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const num_samples = 3000U;
  auto const max_cost = 3 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_bidijkstra_consistency(w, l, num_samples, max_cost);
}

TEST(bidirectional_dijkstra, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const num_samples = 500U; // large map -> smaller sample for time
  auto const max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_bidijkstra_consistency(w, l, num_samples, max_cost);
}

TEST(bidirectional_dijkstra, DISABLED_germany) {
  auto const raw_data = "test/germany.osm.pbf";
  auto const data_dir = "test/germany";
  auto const num_samples = 50U;
  auto const max_cost = 12 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  run_bidijkstra_consistency(w, l, num_samples, max_cost);
}
