#include "gtest/gtest.h"

#include "osr/routing/profiles/car.h"
#include "osr/ways.h"

namespace osr {
namespace {

TEST(endpoint_transition, skips_restricted_duplicate_way_occurrence) {
  auto w = ways::routing{};
  auto const n = node_idx_t{0U};
  auto const way = way_idx_t{0U};

  w.node_ways_.resize(1U);
  w.node_ways_[n].push_back(way);
  w.node_ways_[n].push_back(way);
  w.node_turn_bearings_.resize(1U);
  w.node_turn_bearings_[n].push_back(turn_bearing{});
  w.node_turn_bearings_[n].push_back(turn_bearing{});
  w.node_is_restricted_.resize(1U);
  w.node_is_restricted_.set(n, true);
  w.node_restrictions_.resize(1U);
  w.node_restrictions_[n].push_back(restriction{.from_ = 0U,
                                                .to_ = 1U,
                                                .applies_to_default_ = true,
                                                .applies_to_bus_ = true,
                                                .applies_to_hgv_ = true});

  auto const params = car::parameters{};
  auto const incoming = car::node{n, way_pos_t{0U}, direction::kForward};
  auto const transition = car::endpoint_transition_cost(
      params, w, timezone_cache_t{}, incoming, way, direction::kBackward,
      direction::kForward, std::nullopt, duration_t{0U});

  EXPECT_EQ(params.uturn_penalty_, transition.cost_);
}

}  // namespace
}  // namespace osr
