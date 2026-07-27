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
  auto constexpr max_cost = 2 * 3600U;

  if (!fs::exists(raw_data) && !fs::exists(data_dir)) {
    GTEST_SKIP() << raw_data << " not found";
  }

  load(raw_data, data_dir);
  auto const w = osr::ways{data_dir, cista::mmap::protection::READ};
  auto const l = osr::lookup{w, data_dir, cista::mmap::protection::READ};

  // <-- pick your real-world start/end here (lat, lng) -->
  // const start = [7.423440329846727, 43.729952584237395]
  // const destination = [7.4261553024191755, 43.73175634804065];
  //
  auto const from =
      location{geo::latlng{43.729952584237395, 7.423440329846727}};
  auto const to = location{geo::latlng{43.73175634804065, 7.4261553024191755}};

  // Snap the locations onto the routing graph.
  auto const from_matches = l.match<car>(car::parameters{}, from, false, dir,
                                         kMaxMatchDistance, nullptr);
  auto const to_matches = l.match<car>(car::parameters{}, to, true, dir,
                                       kMaxMatchDistance, nullptr);

  ASSERT_FALSE(from_matches.empty()) << "no graph match near 'from'";
  ASSERT_FALSE(to_matches.empty()) << "no graph match near 'to'";

  auto const from_matches_span =
      std::span{begin(from_matches), end(from_matches)};
  auto const to_matches_span = std::span{begin(to_matches), end(to_matches)};

  // Set a breakpoint here (or inside osr::dijkstra) and step into the search.
  auto const result =
      route(car::parameters{}, w, l, search_profile::kCar, from, to,
            from_matches_span, to_matches_span, max_cost, dir, nullptr, nullptr,
            nullptr, routing_algorithm::kDijkstraBi);

  if (result.has_value()) {
    fmt::println("found path | cost: {} | dist: {:.2f}", result->cost_,
                 result->dist_);
  } else {
    fmt::println("no path found");
  }
}
