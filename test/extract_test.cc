#include "gtest/gtest.h"

#include <filesystem>

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/dijkstra.h"
#include "osr/extract.h"
#include "osr/lookup.h"
#include "osr/to_geojson.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::path{"/tmp/osr_test"};
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  auto const c = config{"test/map.osm", "/tmp/osr_test"};
  osr::extract(c);

  auto w = osr::ways{"/tmp/osr_test", cista::mmap::protection::READ};

  auto s = dijkstra_state{};
  auto l = lookup{w};

  auto const max_dist = 900;
  auto r = std::vector<start_dist>{};
  l.find({49.87810348736298, 8.653064959841231}, r);
  for (auto const& d : r) {
    s.reset(w, max_dist);

    auto const [right_node, right_dist] = d.init_right_;
    if (right_node != node_idx_t::invalid()) {
      s.add_start(right_node, right_dist);
      fmt::println("right node: {} [osm_id={}], dist={}, way={}, path={}",
                   right_node, w.node_to_osm_[right_node], right_dist, d.way_,
                   d.right_path_);
    }

    auto const [left_node, left_dist] = d.init_left_;
    if (left_node != node_idx_t::invalid()) {
      s.add_start(left_node, left_dist);
      fmt::println("left node: {} [osm_id={}], dist={}, way={}, path={}",
                   left_node, w.node_to_osm_[left_node], left_dist, d.way_,
                   d.left_path_);
    }

    dijkstra(w, s, max_dist, pedestrian{});

    fmt::println(
        "{}",
        osr::to_geojson(
            w, &s, left_node != node_idx_t::invalid() ? &d.left_path_ : nullptr,
            right_node != node_idx_t::invalid() ? &d.right_path_ : nullptr));

    break;
  }
}