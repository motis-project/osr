#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>
#include <random>

#include "cista/mmap.h"

#include "utl/parallel_for.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/preprocessing/contraction_hierarchies/preprocessor.h"
#include "osr/routing/bidirectional.h"
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
constexpr auto const kMaxMatchDistance = 100;
constexpr auto const kMaxAllowedPathDifferenceRatio = 0.5;

void load_with_ch(std::string_view const raw_data,
                  std::string_view const data_dir,
                  std::string_view const ch_dir,
                  size_t const stall = 95) {
  if (!fs::exists(data_dir)) {
    if (fs::exists(raw_data)) {
      auto const p = fs::path{data_dir};
      auto ec = std::error_code{};
      fs::remove_all(p, ec);
      fs::create_directories(p, ec);
      osr::extract(false, raw_data, data_dir, fs::path{});
    }
  }
  if (fs::exists(data_dir) && !fs::exists(ch_dir)) {
    auto const p = fs::path{data_dir};
    auto const ch = fs::path{ch_dir};

    if (!fs::exists(ch)) {
      fmt::println("Creating new contraction hierarchie directory: {}", ch);
      fs::create_directory(ch);
      for (const auto& entry : fs::directory_iterator(p)) {
        fs::copy(entry.path(), ch / entry.path().filename(),
                 fs::copy_options::recursive);
      }
    }
    std::unique_ptr<ch::order_strategy> node_order_strategy =
        std::make_unique<osr::ch::node_importance_order_strategy>(-1);
    ;
    osr::ch::process_ch(data_dir, ch, node_order_strategy, stall);
  }
}

void run(ways const& w,
         ways const& w_shortcuts,
         lookup const& l,
         unsigned const n_samples,
         unsigned const max_cost,
         ch::shortcut_storage const* shortcut_storage) {

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

  auto const single_run = [&](std::pair<node_idx_t, node_idx_t> const from_to) {
    auto const from_node = from_to.first;
    auto const from_loc = location{w.get_node_pos(from_node)};
    auto const to_node = from_to.second;
    auto const to_loc = location{w.get_node_pos(to_node)};

    auto const node_pinned_matches =
        [&](location const& loc, node_idx_t const n, bool const reverse) {
          auto matches =
              l.match<car>(car::parameters{}, loc, reverse, direction::kForward,
                           kMaxMatchDistance, nullptr);
          std::erase_if(matches, [&](auto const& wc) {
            return wc.left_.node_ != n && wc.right_.node_ != n;
          });
          if (matches.size() > 1) {
            // matches.resize(1);
          }
          return matches;
        };
    auto const from_matches = node_pinned_matches(from_loc, from_node, false);
    auto const to_matches = node_pinned_matches(to_loc, to_node, true);
    if (from_matches.empty() || to_matches.empty()) {
      ++n_empty_matches;
    }

    auto const from_matches_span =
        std::span{begin(from_matches), end(from_matches)};
    auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

    auto const reference_start = std::chrono::steady_clock::now();
    auto const reference =
        route(car::parameters{}, w, l, search_profile::kCar, from_loc, to_loc,
              from_matches_span, to_matches_span, max_cost, direction::kForward,
              nullptr, nullptr, nullptr, routing_algorithm::kDijkstra, nullptr);
    auto const reference_time =
        std::chrono::steady_clock::now() - reference_start;

    auto const experiment_start = std::chrono::steady_clock::now();
    auto const experiment = route(
        car::parameters{}, w, l, search_profile::kCar, from_loc, to_loc,
        from_matches_span, to_matches_span, max_cost, direction::kForward,
        nullptr, nullptr, nullptr, routing_algorithm::kContractionHierarchy,
        shortcut_storage, &w_shortcuts);
    auto const experiment_time =
        std::chrono::steady_clock::now() - experiment_start;

    if (reference.has_value() != experiment.has_value() ||
        (reference && experiment &&
         (reference->cost_ != experiment->cost_ /*||
          std::abs(reference->dist_ - experiment->dist_) / reference->dist_ >
              kMaxAllowedPathDifferenceRatio*/))) {
      auto const print_result = [&](std::string_view name, auto const& p,
                                    auto const& t) {
        fmt::println(
            "{:10}: {:11} --> {:11} | {} | time: "
            "{}:{:0>3}:{:0>3} s",
            name, from_node, to_node,
            p ? fmt::format("cost: {:5} | dist: {:>10.2f}", p->cost_, p->dist_)
              : "no result",
            std::chrono::duration_cast<std::chrono::seconds>(t).count(),
            std::chrono::duration_cast<std::chrono::milliseconds>(t).count() %
                1000,
            std::chrono::duration_cast<std::chrono::microseconds>(t).count() %
                1000);
        if (p.has_value() && kPrintDebugGeojson) {
          fmt::println("{}\n", to_featurecollection(w, p));
        }
      };

      print_result("dijkstra", reference, reference_time);
      print_result("bidir ch", experiment, experiment_time);

    } else {
      ++n_congruent;
    }

    if (!from_matches.empty() && !to_matches.empty()) {
      auto const guard = std::lock_guard{m};
      reference_times.emplace_back(reference_time);
      experiment_times.emplace_back(experiment_time);
    }
  };

  if (kUseMultithreading) {
    utl::parallel_for(from_tos, single_run);
  } else {
    std::for_each(begin(from_tos), end(from_tos), single_run);
  }

  auto const non_empty_congruent = n_congruent - n_empty_matches;
  auto const non_empty_samples = n_samples - n_empty_matches;

  EXPECT_EQ(non_empty_samples, non_empty_congruent);

  fmt::println("congruent on non-empty: {}/{} ({:3.1f}%)", non_empty_congruent,
               non_empty_samples,
               (static_cast<double>(non_empty_congruent) /
                static_cast<double>(non_empty_samples)) *
                   100);
  if (non_empty_congruent == non_empty_samples) {
    fmt::println(
        "speedup on non-empty: {:.2f}",
        static_cast<double>(
            std::reduce(begin(reference_times), end(reference_times)).count()) /
            static_cast<double>(
                std::reduce(begin(experiment_times), end(experiment_times))
                    .count()));
  }
}

TEST(dijkstra_ch_bidir, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  auto const data_dir_contraction_hierarchie =
      "test/monaco_contraction_hierarchie";
  auto constexpr num_samples = 10000U;
  auto constexpr max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_with_ch(raw_data, data_dir, data_dir_contraction_hierarchie);
  auto w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto w_shortcuts =
      osr::ways{data_dir_contraction_hierarchie, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto shortcuts = std::make_unique<ch::shortcut_storage>();
  if (shortcuts != nullptr) {
    shortcuts->load(data_dir_contraction_hierarchie);
    shortcuts->load_all_shortcuts_in_graph(w);
    fmt::print("loaded {} shortcuts\n", shortcuts->shortcuts_.size());
  }
  auto node_order = std::make_unique<ch::node_order>(
      data_dir_contraction_hierarchie, cista::mmap::protection::READ);
  if (node_order != nullptr) {
    fmt::print("loaded node order\n");
  }
  fmt::print("using contraction hierarchy\n");
  shortcuts->node_order_ = std::move(node_order);

  run(w, w_shortcuts, l, num_samples, max_cost, shortcuts.get());
}

TEST(dijkstra_ch_bidir, aachen) {
  auto const raw_data = "test/aachen.osm.pbf";
  auto const data_dir = "build/osr-aachen";
  auto const data_dir_contraction_hierarchie = "build/osr-aachen-ch";
  auto constexpr num_samples = 5000U;
  auto constexpr max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_with_ch(raw_data, data_dir, data_dir_contraction_hierarchie);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto w_shortcuts =
      osr::ways{data_dir_contraction_hierarchie, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto const shortcuts = std::make_unique<ch::shortcut_storage>();
  if (shortcuts != nullptr) {
    shortcuts->load(data_dir_contraction_hierarchie);
    shortcuts->load_all_shortcuts_in_graph(w_shortcuts);
    fmt::print("loaded {} shortcuts\n", shortcuts->shortcuts_.size());
  }
  auto node_order = std::make_unique<ch::node_order>(
      data_dir_contraction_hierarchie, cista::mmap::protection::READ);
  if (node_order != nullptr) {
    fmt::print("loaded node order\n");
  }
  fmt::print("using contraction hierarchy\n");
  shortcuts->node_order_ = std::move(node_order);

  run(w, w_shortcuts, l, num_samples, max_cost, shortcuts.get());
}

TEST(dijkstra_ch_bidir, hamburg) {
  auto const raw_data = "test/hamburg.osm.pbf";
  auto const data_dir = "test/hamburg";
  auto const data_dir_contraction_hierarchie = "test/hamburg-ch";
  auto constexpr num_samples = 5000U;
  auto constexpr max_cost = 2.5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_with_ch(raw_data, data_dir, data_dir_contraction_hierarchie, 90);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto w_shortcuts =
      osr::ways{data_dir_contraction_hierarchie, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto const shortcuts = std::make_unique<ch::shortcut_storage>();
  if (shortcuts != nullptr) {
    shortcuts->load(data_dir_contraction_hierarchie);
    shortcuts->load_all_shortcuts_in_graph(w_shortcuts);
    fmt::print("loaded {} shortcuts\n", shortcuts->shortcuts_.size());
  }
  auto node_order = std::make_unique<ch::node_order>(
      data_dir_contraction_hierarchie, cista::mmap::protection::READ);
  if (node_order != nullptr) {
    fmt::print("loaded node order\n");
  }
  fmt::print("using contraction hierarchy\n");
  shortcuts->node_order_ = std::move(node_order);

  run(w, w_shortcuts, l, num_samples, max_cost, shortcuts.get());
}

TEST(dijkstra_ch_bidir, switzerland) {
  auto const raw_data = "test/switzerland.osm.pbf";
  auto const data_dir = "test/switzerland";
  auto const data_dir_contraction_hierarchie =
      "test/switzerland_contraction_hierarchie";
  auto constexpr num_samples = 1000U;
  auto constexpr max_cost = 5 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_with_ch(raw_data, data_dir, data_dir_contraction_hierarchie, 88);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto w_shortcuts =
      osr::ways{data_dir_contraction_hierarchie, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto const shortcuts = std::make_unique<ch::shortcut_storage>();
  if (shortcuts != nullptr) {
    shortcuts->load(data_dir_contraction_hierarchie);
    shortcuts->load_all_shortcuts_in_graph(w_shortcuts);
    fmt::print("loaded {} shortcuts\n", shortcuts->shortcuts_.size());
  }
  auto node_order = std::make_unique<ch::node_order>(
      data_dir_contraction_hierarchie, cista::mmap::protection::READ);
  if (node_order != nullptr) {
    fmt::print("loaded node order\n");
  }
  fmt::print("using contraction hierarchy\n");
  shortcuts->node_order_ = std::move(node_order);

  run(w, w_shortcuts, l, num_samples, max_cost, shortcuts.get());
}

TEST(dijkstra_ch_bidir, DISABLED_germany) {
  auto const raw_data = "test/germany.osm.pbf";
  auto const data_dir = "test/germany";
  auto const data_dir_contraction_hierarchie =
      "test/germany_contraction_hierarchie";
  constexpr auto num_samples = 50U;
  constexpr auto max_cost = 12 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load_with_ch(raw_data, data_dir, data_dir_contraction_hierarchie);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto w_shortcuts =
      osr::ways{data_dir_contraction_hierarchie, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  auto const shortcuts = std::make_unique<ch::shortcut_storage>();
  if (shortcuts != nullptr) {
    shortcuts->load(data_dir_contraction_hierarchie);
    shortcuts->load_all_shortcuts_in_graph(w_shortcuts);
    fmt::print("loaded {} shortcuts\n", shortcuts->shortcuts_.size());
  }
  auto node_order = std::make_unique<ch::node_order>(
      data_dir_contraction_hierarchie, cista::mmap::protection::READ);
  if (node_order != nullptr) {
    fmt::print("loaded node order\n");
  }
  fmt::print("using contraction hierarchy\n");
  shortcuts->node_order_ = std::move(node_order);

  run(w, w_shortcuts, l, num_samples, max_cost, shortcuts.get());
}