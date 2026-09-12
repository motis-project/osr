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
#include "osr/routing/cch.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/dijkstra_bidir.h"
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
  // const start = [7.418969078064066, 43.7330953688176]
  // const destination = [7.4261553024191755, 43.73175634804065];
  //
  auto const from = location{geo::latlng{43.7330953688176, 7.418969078064066}};
  auto const to = location{geo::latlng{43.73175634804065, 7.4261553024191755}};
  using profile = car;
  auto const params = profile::parameters{};

  // Snap the locations onto the routing graph.
  auto from_storage = match_result{};
  auto to_storage = match_result{};
  l.match<profile>(params, from, false, dir, kMaxMatchDistance, nullptr,
                   from_storage);
  l.match<profile>(params, to, true, dir, kMaxMatchDistance, nullptr,
                   to_storage);
  auto const from_matches = from_storage[match_idx_t{0U}];
  auto const to_matches = to_storage[match_idx_t{0U}];

  ASSERT_FALSE(from_matches.empty()) << "no graph match near 'from'";
  ASSERT_FALSE(to_matches.empty()) << "no graph match near 'to'";

  auto const dump_matches = [&](std::string_view name,
                                match_view_t const& matches) {
    fmt::println("{} matches: {}", name, matches.size());
    for (auto i = std::size_t{0U}; i != matches.size(); ++i) {
      fmt::println("  [{}] way={} osm_way={} dist_to_way={:.2f}", i,
                   to_idx(matches.way_[i]),
                   to_idx(w.way_osm_idx_[matches.way_[i]]),
                   matches.dist_to_way_[i]);
      auto const dump_node = [&](std::string_view side,
                                 candidate_node const& nc) {
        if (!nc.valid()) {
          fmt::println("    {}: invalid", side);
          return;
        }
        auto const pos = w.get_node_pos(nc.node_).as_latlng();
        fmt::println(
            "    {}: node={} osm_node={} rank={} way_dir={} "
            "dist_to_node={:.2f} "
            "cost={} lat={} lng={}",
            side, to_idx(nc.node_), to_idx(w.node_to_osm_[nc.node_]),
            w.r_->node_importance_[nc.node_], to_str(nc.way_dir_),
            nc.dist_to_node_, nc.cost_, pos.lat_, pos.lng_);
      };
      dump_node("left", matches.left(i));
      dump_node("right", matches.right(i));
    }
  };
  dump_matches("start", from_matches);
  dump_matches("end", to_matches);

  auto const run_exact_node_query = [&](osm_node_idx_t const start_osm,
                                        osm_node_idx_t const dest_osm) {
    auto const start_node = w.get_node_idx(start_osm);
    auto const dest_node = w.get_node_idx(dest_osm);

    auto d = dijkstra<profile>{};
    d.reset(search_params<profile::parameters>{
        .profile_ = params, .w_ = &w, .max_ = max_cost, .dir_ = dir});
    profile::resolve_all(*w.r_, start_node, kNoLevel, [&](auto const node) {
      d.add_start({node, 0U}, duration_t{0});
    });
    d.run();

    auto dijkstra_cost = kInfeasible;
    profile::resolve_all(*w.r_, dest_node, kNoLevel, [&](auto const node) {
      dijkstra_cost = std::min(dijkstra_cost, d.get_cost(node));
    });

    auto bd = dijkstra_bidir<profile>{};
    bd.reset(max_cost);
    profile::resolve_all(*w.r_, start_node, kNoLevel,
                         [&](auto const node) { bd.add_start(w, {node, 0U}); });
    profile::resolve_all(*w.r_, dest_node, kNoLevel, [&](auto const node) {
      bd.add_destination(w, {node, 0U});
    });
    bd.run(params, w, *w.r_, max_cost, nullptr, nullptr, nullptr, dir);

    auto c = cch<profile>{};
    c.reset(max_cost);
    profile::resolve_all(*w.r_, start_node, kNoLevel,
                         [&](auto const node) { c.add_start(w, {node, 0U}); });
    profile::resolve_all(*w.r_, dest_node, kNoLevel, [&](auto const node) {
      c.add_destination(w, {node, 0U});
    });
    c.run(params, w, *w.r_, max_cost, nullptr, nullptr, nullptr, dir);

    fmt::println(
        "exact node query osm {} -> {} | internal {} -> {} | dijkstra {} | "
        "bidir {} | cch {}",
        to_idx(start_osm), to_idx(dest_osm), to_idx(start_node),
        to_idx(dest_node), dijkstra_cost, bd.mu_, c.mu_);
  };

  run_exact_node_query(osm_node_idx_t{25194304U}, osm_node_idx_t{7787103278U});
  run_exact_node_query(osm_node_idx_t{25194304U}, osm_node_idx_t{2737814240U});

  auto const from_matches_span = from_matches;
  auto const to_matches_span = to_matches;

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
    fmt::println(
        "comparison mismatch | dijkstra_has_path: {} | bidir_has_path: {}",
        dijkstra_result.has_value(), bidir_result.has_value());
  } else if (dijkstra_result.has_value() && bidir_result.has_value()) {
    fmt::println("comparison | cost equal: {} | dist equal: {}",
                 dijkstra_result->cost_ == bidir_result->cost_,
                 dijkstra_result->dist_ == bidir_result->dist_);
  }
  if (dijkstra_result.has_value() != cch_result.has_value()) {
    fmt::println(
        "cch comparison mismatch | dijkstra_has_path: {} | cch_has_path: {}",
        dijkstra_result.has_value(), cch_result.has_value());
  } else if (dijkstra_result.has_value() && cch_result.has_value()) {
    fmt::println("cch comparison | cost equal: {} | dist equal: {}",
                 dijkstra_result->cost_ == cch_result->cost_,
                 dijkstra_result->dist_ == cch_result->dist_);
  }
}
