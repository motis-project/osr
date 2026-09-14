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
                     cost_t{3600U}, direction::kForward, 50.0);
  auto const astar = route_astar(params, w, l, search_profile::kFoot, from, to,
                                 cost_t{3600U}, direction::kForward, 50.0);
  auto const bidirectional =
      route_bidirectional(params, w, l, search_profile::kFoot, from, to,
                          cost_t{3600U}, direction::kForward, 50.0);

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
      to, cost_t{3600U}, direction::kForward, 50.0, nullptr, nullptr, nullptr,
      std::nullopt, route_options{});

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
  auto const result = route_dijkstra(profile_parameters{car::parameters{}}, w,
                                     l, search_profile::kCar, from, to,
                                     cost_t{3600U}, direction::kForward, 50.0);

  ASSERT_TRUE(result.has_value());
  ASSERT_GE(result->segments_.size(), 2U);
  auto const& start = result->segments_.front();
  auto const& destination = result->segments_.back();
  EXPECT_GT(start.cost_, start.duration_.count());
  EXPECT_GT(destination.cost_, destination.duration_.count());

  auto total_duration = duration_t{0U};
  for (auto const& segment : result->segments_) {
    total_duration = clamp_add_duration(total_duration, segment.duration_);
  }
  EXPECT_EQ(total_duration, result->duration_);
}

TEST_F(endpoint_matching_test, closer_match_wins_over_graph_shortcut) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const params = get_parameters(search_profile::kFoot);
  auto const from = location{{49.040000, 8.000000}, kNoLevel};
  auto const to = location{{49.040100, 8.004000}, kNoLevel};

  auto const without_preference =
      route(params, w, l, search_profile::kFoot, from, to, cost_t{3600U},
            direction::kForward, 50.0, nullptr, nullptr, nullptr,
            routing_algorithm::kDijkstra, std::nullopt,
            route_options{.matching_penalty_factor_ = 0.0});
  ASSERT_TRUE(without_preference.has_value());
  ASSERT_FALSE(without_preference->segments_.empty());
  ASSERT_FALSE(without_preference->segments_.front().polyline_.empty());
  EXPECT_EQ(project_to_osm_way(w, 901, from.pos_),
            without_preference->segments_.front().polyline_.front());

  auto const preferred = route(params, w, l, search_profile::kFoot, from, to,
                               cost_t{3600U}, direction::kForward, 50.0);
  ASSERT_TRUE(preferred.has_value());
  ASSERT_FALSE(preferred->segments_.empty());
  ASSERT_FALSE(preferred->segments_.front().polyline_.empty());
  EXPECT_EQ(project_to_osm_way(w, 900, from.pos_),
            preferred->segments_.front().polyline_.front());
}

TEST_F(endpoint_matching_test,
       closer_destination_match_wins_over_graph_shortcut) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const params = get_parameters(search_profile::kFoot);
  auto const from = location{{49.040100, 8.004000}, kNoLevel};
  auto const to = location{{49.040000, 8.000000}, kNoLevel};

  auto const without_preference =
      route(params, w, l, search_profile::kFoot, from, to, cost_t{3600U},
            direction::kForward, 50.0, nullptr, nullptr, nullptr,
            routing_algorithm::kDijkstra, std::nullopt,
            route_options{.matching_penalty_factor_ = 0.0});
  ASSERT_TRUE(without_preference.has_value());
  ASSERT_FALSE(without_preference->segments_.empty());
  ASSERT_FALSE(without_preference->segments_.back().polyline_.empty());
  EXPECT_EQ(project_to_osm_way(w, 901, to.pos_),
            without_preference->segments_.back().polyline_.back());

  auto const preferred = route(params, w, l, search_profile::kFoot, from, to,
                               cost_t{3600U}, direction::kForward, 50.0);
  ASSERT_TRUE(preferred.has_value());
  ASSERT_FALSE(preferred->segments_.empty());
  ASSERT_FALSE(preferred->segments_.back().polyline_.empty());
  EXPECT_EQ(project_to_osm_way(w, 900, to.pos_),
            preferred->segments_.back().polyline_.back());
}

TEST_F(endpoint_matching_test, unreachable_closest_match_uses_farther_match) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const profile_params = car::parameters{};
  auto const params = profile_parameters{profile_params};
  auto const from = location{{49.010000, 8.000000}, kNoLevel};
  auto const to = location{{49.010040, 8.002800}, kNoLevel};

  auto to_matches = match_result{};
  l.match<car>(profile_params, to, true, direction::kForward, 50.0, nullptr,
               false, to_matches);
  auto const matches = to_matches[match_idx_t{0U}];
  ASSERT_GE(matches.size(), 2U);
  EXPECT_EQ(std::optional<std::int64_t>{400}, w.get_osm_way(matches.way_[0]));
  EXPECT_EQ(std::optional<std::int64_t>{300}, w.get_osm_way(matches.way_[1]));

  auto const result = route(params, w, l, search_profile::kCar, from, to,
                            cost_t{3600U}, direction::kForward, 50.0);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->segments_.empty());
  EXPECT_EQ(std::optional<std::int64_t>{31},
            w.get_osm_node(result->segments_.back().from_));
}

}  // namespace
}  // namespace osr
