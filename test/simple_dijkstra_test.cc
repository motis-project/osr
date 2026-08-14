#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <filesystem>

#include "cista/mmap.h"

#include "fmt/core.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profile.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

constexpr auto const kMaxMatchDistance = 100;

static void load(std::string_view raw_data, std::string_view data_dir) {
  if (fs::exists(raw_data)) {
    auto const p = fs::path{data_dir};
    auto ec = std::error_code{};
    fs::remove_all(p, ec);
    fs::create_directories(p, ec);
    osr::extract(false, raw_data, data_dir, fs::path{});
  }
}

// A single, debugger-friendly Dijkstra run between two real-world locations.
// Pick `from`/`to` below (lat/lng), set a breakpoint in osr::dijkstra and
// step through the search.
TEST(simple_dijkstra, monaco) {
  auto const raw_data = "test/monaco.osm.pbf";
  auto const data_dir = "test/monaco";
  // auto const raw_data = "test/darmstadt-bismarckstr.osm.pbf";
  // auto const data_dir = "test/darmstadt-bismarckstr";
  auto constexpr dir = direction::kForward;
  auto constexpr max_cost = 4 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // <-- pick your real-world start/end here (lat, lng) -->
  // const start = [7.418807157337369, 43.73319419824978]
  // const destination = [7.4261553024191755, 43.73175634804065];
  //
  auto const from =
      location{geo::latlng{43.73319419824978, 7.418807157337369}};
  auto const to = location{geo::latlng{43.73175634804065, 7.4261553024191755}};
  using profile = car;
  auto const params = profile::parameters{};

  // Snap the locations onto the routing graph.
  auto const from_matches =
      l.match<profile>(params, from, false, dir, kMaxMatchDistance, nullptr);
  auto const to_matches =
      l.match<profile>(params, to, true, dir, kMaxMatchDistance, nullptr);

  ASSERT_FALSE(from_matches.empty()) << "no graph match near 'from'";
  ASSERT_FALSE(to_matches.empty()) << "no graph match near 'to'";

  auto const from_matches_span =
      std::span{begin(from_matches), end(from_matches)};
  auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

  // Set a breakpoint here (or inside osr::dijkstra) and step into the search.
  auto const dijkstra_result =
      route(params, w, l, search_profile::kCar, from, to, from_matches_span,
            to_matches_span, max_cost, dir, nullptr, nullptr, nullptr,
            routing_algorithm::kDijkstra);
  auto const bidir_result =
      route(params, w, l, search_profile::kCar, from, to, from_matches_span,
            to_matches_span, max_cost, dir, nullptr, nullptr, nullptr,
            routing_algorithm::kDijkstraBi);
  auto const cch_result =
      route(params, w, l, search_profile::kCar, from, to, from_matches_span,
            to_matches_span, max_cost, dir, nullptr, nullptr, nullptr,
            routing_algorithm::kCCH);

  if (dijkstra_result.has_value()) {
    fmt::println("dijkstra found path | cost: {} | dist: {:.2f}",
                 dijkstra_result->cost_, dijkstra_result->dist_);
  } else {
    fmt::println("dijkstra found no path");
  }
  if (bidir_result.has_value()) {
    fmt::println("bidir found path | cost: {} | dist: {:.2f}",
                 bidir_result->cost_, bidir_result->dist_);
  } else {
    fmt::println("bidir found no path");
  }
  if (cch_result.has_value()) {
    fmt::println("cch found path | cost: {} | dist: {:.2f}", cch_result->cost_,
                 cch_result->dist_);
  } else {
    fmt::println("cch found no path");
  }

  if (dijkstra_result.has_value() != bidir_result.has_value()) {
    fmt::println("comparison mismatch | dijkstra_has_path: {} | bidir_has_path: {}",
                 dijkstra_result.has_value(), bidir_result.has_value());
  } else if (dijkstra_result.has_value() && bidir_result.has_value()) {
    fmt::println("comparison | cost equal: {} | dist equal: {}",
                 dijkstra_result->cost_ == bidir_result->cost_,
                 dijkstra_result->dist_ == bidir_result->dist_);
  }
  if (dijkstra_result.has_value() != cch_result.has_value()) {
    fmt::println("cch comparison mismatch | dijkstra_has_path: {} | cch_has_path: {}",
                 dijkstra_result.has_value(), cch_result.has_value());
  } else if (dijkstra_result.has_value() && cch_result.has_value()) {
    fmt::println("cch comparison | cost equal: {} | dist equal: {}",
                 dijkstra_result->cost_ == cch_result->cost_,
                 dijkstra_result->dist_ == cch_result->dist_);
  }
}
