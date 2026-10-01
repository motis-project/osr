#include <chrono>
#include <algorithm>
#include <iterator>
#include <tuple>

#include "gtest/gtest.h"

#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"

#include "sharing_routing_fixture.h"

namespace osr {
namespace {

using endpoint_matching_test = test::sharing_routing_fixture;

geo::latlng project_to_osm_way(ways const& w,
                               std::int64_t const osm_way,
                               geo::latlng const& pos) {
  auto const way_it = std::lower_bound(
      begin(w.way_osm_idx_), end(w.way_osm_idx_), to_osm_way_idx(osm_way));
  utl::verify(
      way_it != end(w.way_osm_idx_) && *way_it == to_osm_way_idx(osm_way),
      "OSM way {} not found", osm_way);
  auto const way = way_idx_t{static_cast<way_idx_t::value_t>(
      std::distance(begin(w.way_osm_idx_), way_it))};
  return std::get<1>(geo::approx_squared_distance_to_polyline<
                     std::tuple<double, geo::latlng, std::size_t>>(
      pos, w.way_polylines_[way], geo::approx_distance_lng_degrees(pos)));
}

TEST_F(endpoint_matching_test, endpoint_matches_work_with_all_algorithms) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const from = location{{49.010000, 8.000000}, kNoLevel};
  auto const to = location{{49.010045, 8.002800}, kNoLevel};
  auto const params = get_parameters(search_profile::kFoot);

  auto const dijkstra =
      route_dijkstra(params, w, l, search_profile::kFoot, from, to,
                     std::chrono::seconds{3600}, direction::kForward, 50.0);
  auto const astar =
      route_astar(params, w, l, search_profile::kFoot, from, to,
                  std::chrono::seconds{3600}, direction::kForward, 50.0);
  auto const bidirectional = route_bidirectional(
      params, w, l, search_profile::kFoot, from, to, std::chrono::seconds{3600},
      direction::kForward, 50.0);

  ASSERT_TRUE(dijkstra.has_value());
  ASSERT_TRUE(astar.has_value());
  ASSERT_TRUE(bidirectional.has_value());
  EXPECT_EQ(dijkstra->cost_, astar->cost_);
  EXPECT_EQ(dijkstra->cost_, bidirectional->cost_);
  EXPECT_EQ(dijkstra->duration_, astar->duration_);
  EXPECT_EQ(dijkstra->duration_, bidirectional->duration_);
}

TEST_F(endpoint_matching_test,
       reconstructed_duration_excludes_matching_penalty) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const from = location{{49.040000, 8.000000}, kNoLevel};
  auto const to = location{{49.040100, 8.004000}, kNoLevel};
  auto const result = route_dijkstra(
      get_parameters(search_profile::kFoot), w, l, search_profile::kFoot, from,
      to, std::chrono::seconds{3600}, direction::kForward, 50.0, nullptr,
      nullptr, nullptr, std::nullopt, route_options{});

  ASSERT_TRUE(result.has_value());
  auto segment_duration = duration_t{0U};
  for (auto const& segment : result->segments_) {
    segment_duration = clamp_add_duration(segment_duration, segment.duration_);
  }
  EXPECT_EQ(segment_duration, result->duration_);
  EXPECT_NE(duration_from_cost(result->cost_), result->duration_);
}

TEST_F(endpoint_matching_test,
       endpoint_access_penalties_do_not_increase_duration) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const from = location{{49.060000, 8.000250}, kNoLevel};
  auto const to = location{{49.060000, 8.001750}, kNoLevel};
  auto const result = route_dijkstra(
      profile_parameters{car::parameters{}}, w, l, search_profile::kCar, from,
      to, std::chrono::seconds{3600}, direction::kForward, 50.0);

  ASSERT_TRUE(result.has_value());
  // Both ends are on the same stretch of way 1100 (access=destination): the
  // direct piece carries the access penalty in its cost only.
  ASSERT_EQ(1U, result->segments_.size());
  auto const& piece = result->segments_.front();
  EXPECT_GT(piece.cost_, piece.duration_.count());
  EXPECT_NEAR(geo::distance(from.pos_, to.pos_), result->dist_, 1.0);

  auto total_duration = duration_t{0U};
  for (auto const& segment : result->segments_) {
    total_duration = clamp_add_duration(total_duration, segment.duration_);
  }
  EXPECT_EQ(total_duration, result->duration_);
}

// Way 901 is a shortcut to the destination, but in the same component as the
// closer way 900: only 900 is matched, whatever the matching penalty.
TEST_F(endpoint_matching_test,
       closest_match_in_component_wins_over_graph_shortcut) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const params = get_parameters(search_profile::kFoot);
  auto const from = location{{49.040000, 8.000000}, kNoLevel};
  auto const to = location{{49.040100, 8.004000}, kNoLevel};

  auto from_matches = match_result{};
  l.match<foot<false, elevator_tracking>>(
      std::get<foot<false, elevator_tracking>::parameters>(params), from, false,
      direction::kForward, 50.0, nullptr, false, from_matches);
  auto const m = from_matches[match_idx_t{0U}];
  ASSERT_EQ(1U, m.size());
  EXPECT_EQ(std::optional<std::int64_t>{900}, w.get_osm_way(m.way_[0]));

  for (auto const factor : {0.0, kDefaultMatchingPenaltyFactor}) {
    auto const result =
        route(params, w, l, search_profile::kFoot, from, to,
              std::chrono::seconds{3600}, direction::kForward, 50.0, nullptr,
              nullptr, nullptr, routing_algorithm::kDijkstra, std::nullopt,
              route_options{.matching_penalty_factor_ = factor});
    ASSERT_TRUE(result.has_value());
    ASSERT_FALSE(result->segments_.empty());
    ASSERT_FALSE(result->segments_.front().polyline_.empty());
    EXPECT_EQ(project_to_osm_way(w, 900, from.pos_),
              result->segments_.front().polyline_.front());
  }
}

TEST_F(endpoint_matching_test,
       closest_destination_match_in_component_wins_over_graph_shortcut) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const params = get_parameters(search_profile::kFoot);
  auto const from = location{{49.040100, 8.004000}, kNoLevel};
  auto const to = location{{49.040000, 8.000000}, kNoLevel};

  for (auto const factor : {0.0, kDefaultMatchingPenaltyFactor}) {
    auto const result =
        route(params, w, l, search_profile::kFoot, from, to,
              std::chrono::seconds{3600}, direction::kForward, 50.0, nullptr,
              nullptr, nullptr, routing_algorithm::kDijkstra, std::nullopt,
              route_options{.matching_penalty_factor_ = factor});
    ASSERT_TRUE(result.has_value());
    ASSERT_FALSE(result->segments_.empty());
    ASSERT_FALSE(result->segments_.back().polyline_.empty());
    EXPECT_EQ(project_to_osm_way(w, 900, to.pos_),
              result->segments_.back().polyline_.back());
  }
}

// The closest match 402 is on a car island {402, 403}: it does not shadow way
// 300 in the network, but 403.
TEST_F(endpoint_matching_test, unreachable_closest_match_uses_farther_match) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const profile_params = car::parameters{};
  auto const params = profile_parameters{profile_params};
  auto const from = location{{49.010000, 8.000000}, kNoLevel};
  auto const to = location{{49.010025, 8.001400}, kNoLevel};

  auto to_matches = match_result{};
  l.match<car>(profile_params, to, true, direction::kForward, 50.0, nullptr,
               false, to_matches);
  auto const matches = to_matches[match_idx_t{0U}];
  ASSERT_EQ(2U, matches.size());
  EXPECT_EQ(std::optional<std::int64_t>{402}, w.get_osm_way(matches.way_[0]));
  EXPECT_EQ(std::optional<std::int64_t>{300}, w.get_osm_way(matches.way_[1]));

  auto const result =
      route(params, w, l, search_profile::kCar, from, to,
            std::chrono::seconds{3600}, direction::kForward, 50.0);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->segments_.empty());
  ASSERT_FALSE(result->segments_.back().polyline_.empty());
  EXPECT_EQ(project_to_osm_way(w, 300, to.pos_),
            result->segments_.back().polyline_.back());
}

}  // namespace
}  // namespace osr
