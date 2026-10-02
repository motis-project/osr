#include "gtest/gtest.h"

#include <array>
#include <vector>

#include "osr/ways.h"

namespace osr {

TEST(component_preprocessing, loops_accessibility_and_islands) {
  auto r = ways::routing{};
  auto const way_nodes = std::vector<std::vector<std::uint32_t>>{
      {0U, 1U, 0U}, {1U, 2U}, {2U, 3U}, {3U, 4U}, {4U, 5U},
      {5U, 4U, 6U}, {6U, 7U}, {7U, 8U}, {8U, 9U}, {9U, 8U}};
  auto node_ways = std::vector<std::vector<way_idx_t>>(10U);
  for (auto i = 0U; i != way_nodes.size(); ++i) {
    auto nodes = std::vector<node_idx_t>{};
    for (auto const n : way_nodes[i]) {
      nodes.emplace_back(n);
      node_ways[n].emplace_back(i);
    }
    r.way_nodes_.emplace_back(nodes);
    r.way_properties_.push_back(
        {.is_foot_accessible_ = i == 0U || i == 4U || i == 5U || i >= 8U,
         .is_bike_accessible_ = i <= 1U,
         .is_car_accessible_ = i == 2U || i == 3U || i == 7U});
  }
  for (auto const& ways : node_ways) {
    r.node_ways_.emplace_back(ways);
  }
  auto const expected =
      std::array{std::array{1U, 1U, 0U, 0U, 2U, 2U, 0U, 0U, 3U, 3U},
                 std::array{1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U},
                 std::array{0U, 0U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U}};
  for (auto cls = 0U; cls != kNumComponentClasses; ++cls) {
    auto const c = compute_class_components(r, kAllComponentClasses[cls]);
    for (auto w = 0U; w != way_nodes.size(); ++w) {
      EXPECT_EQ(c.get(way_idx_t{w}).value_or(0U), expected[cls][w]);
    }
  }
}

TEST(component_preprocessing, empty_graph) {
  for (auto const cls : kAllComponentClasses) {
    auto const c = compute_class_components(ways::routing{}, cls);
    EXPECT_TRUE(c.way_component_.empty());
  }
}

TEST(component_preprocessing, reverse_order_chain) {
  constexpr auto kWays = 16'384U;
  auto r = ways::routing{};
  r.way_properties_.resize(kWays, way_properties{.is_car_accessible_ = true});
  auto way_nodes = std::vector<std::vector<node_idx_t>>(kWays);
  for (auto n = 0U; n != kWays - 1U; ++n) {
    auto const lo = kWays - 2U - n;
    r.node_ways_.emplace_back(std::array{way_idx_t{lo}, way_idx_t{lo + 1U}});
    way_nodes[lo].emplace_back(n);
    way_nodes[lo + 1U].emplace_back(n);
  }
  for (auto const& nodes : way_nodes) {
    r.way_nodes_.emplace_back(nodes);
  }
  auto const c = compute_class_components(r, component_class::kCar);
  for (auto i = 0U; i != kWays; ++i) {
    EXPECT_EQ(c.get(way_idx_t{i}), 1U);
  }
}

}  // namespace osr
