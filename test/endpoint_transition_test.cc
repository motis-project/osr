#include "gtest/gtest.h"

#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

namespace osr {
namespace {

template <typename P>
struct endpoint_level_transition : testing::Test {};

using foot_endpoint_profiles = testing::Types<foot<false>,
                                              foot<true>,
                                              bike_sharing,
                                              car_sharing<>,
                                              car_parking<false>,
                                              car_parking<true>,
                                              car_parking<false, false>,
                                              car_parking<true, false>>;
TYPED_TEST_SUITE(endpoint_level_transition, foot_endpoint_profiles);

TYPED_TEST(endpoint_level_transition, rejects_incompatible_foot_levels) {
  using P = TypeParam;
  auto w = ways::routing{};
  auto const way = way_idx_t{0U};
  w.way_properties_.resize(1U);
  w.way_properties_[way].from_level_ = to_idx(level_t{1.F});
  w.way_properties_[way].to_level_ = to_idx(level_t{1.F});
  w.node_properties_.resize(1U);
  auto n = typename P::node{.n_ = node_idx_t{0U}};
  auto const check = [&]() {
    for (auto const lvl : {kNoLevel, level_t{0.F}, level_t{1.F}}) {
      n.lvl_ = lvl;
      for (auto const dir : {direction::kForward, direction::kBackward}) {
        auto const transition = P::endpoint_transition_cost(
            typename P::parameters{}, w, timezone_cache_t{}, n, way,
            direction::kForward, dir, std::nullopt, duration_t{0U});
        EXPECT_EQ(transition.feasible(), lvl != level_t{0.F});
        if (transition.feasible()) {
          EXPECT_EQ(transition.cost_, 0U);
          EXPECT_EQ(transition.duration_, duration_t{0U});
        }
      }
    }
  };
  if constexpr (requires { P::node_type::kInitialFoot; }) {
    for (auto const type :
         {P::node_type::kInitialFoot, P::node_type::kTrailingFoot}) {
      n.type_ = type;
      check();
    }
  } else {
    if constexpr (requires { P::node_type::kFoot; }) {
      n.type_ = P::node_type::kFoot;
    }
    check();
  }
}

TEST(endpoint_transition, additional_edge_turn_uses_physical_travel_order) {
  auto w = ways::routing{};
  auto const n = node_idx_t{0U};
  w.node_ways_.resize(1U);
  w.node_ways_[n].push_back(way_idx_t{0U});
  w.node_ways_[n].push_back(way_idx_t{1U});
  w.node_turn_bearings_.resize(1U);
  // The incoming way bends at the junction. Its unused outgoing bearing
  // must not affect the turn onto the additional edge.
  w.node_turn_bearings_[n].push_back(
      turn_bearing{quantize_angle(0.0), quantize_angle(90.0)});
  w.node_turn_bearings_[n].push_back(
      turn_bearing{quantize_angle(180.0), quantize_angle(180.0)});
  w.node_properties_.resize(1U);
  w.node_properties_[n].is_bus_accessible_ = true;
  auto const coordinates = std::vector<geo::latlng>{};
  auto const edges = hash_map<node_idx_t, std::vector<additional_edge>>{};
  auto const additional =
      sharing_data{.additional_node_offset_ = 1U,
                   .additional_node_coordinates_ = coordinates,
                   .additional_edges_ = edges};
  auto const params = bus::parameters{};
  auto const forward = get_adjacent_additional_node<bus, direction::kForward>(
      params, w, {n, 0U, direction::kForward}, &additional,
      {.to_ = node_idx_t{1U}, .underlying_way_ = way_idx_t{1U}},
      direction::kForward, {}, params.uturn_penalty_);
  auto const backward = get_adjacent_additional_node<bus, direction::kBackward>(
      params, w, {n, 1U, direction::kForward}, &additional,
      {.to_ = node_idx_t{1U}, .underlying_way_ = way_idx_t{0U}},
      direction::kForward, {}, params.uturn_penalty_);
  EXPECT_EQ(std::get<1>(forward), 0U);
  EXPECT_EQ(std::get<1>(backward), std::get<1>(forward));
}

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
