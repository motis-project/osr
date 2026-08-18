#include <algorithm>
#include <filesystem>
#include <iterator>
#include <memory>
#include <numeric>
#include <tuple>
#include <type_traits>
#include <vector>

#include "gtest/gtest.h"

#include "utl/verify.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/routing/tracking.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {
namespace {

struct sharing_routing_test : public ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-sharing-routing-test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::osm_to_pbf("test/sharing-routing.osm"), dir_, {});
    ways_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    lookup_ =
        std::make_unique<lookup>(*ways_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    lookup_.reset();
    ways_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static inline fs::path dir_{};
  static inline std::unique_ptr<ways> ways_{};
  static inline std::unique_ptr<lookup> lookup_{};
};

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

struct test_sharing_data {
  explicit test_sharing_data(
      ways const& w, osm_node_idx_t const start_osm_node = osm_node_idx_t{1U}) {
    auto const start_node = w.find_node_idx(start_osm_node).value();
    auto const additional_node = node_idx_t{w.n_nodes()};
    auto const size =
        static_cast<bitvec<node_idx_t>::size_type>(w.n_nodes() + 1U);
    start_allowed_.resize(size);
    end_allowed_.resize(size);
    through_allowed_.resize(size);
    start_allowed_.set(additional_node, true);
    end_allowed_.one_out();
    through_allowed_.one_out();
    additional_node_coordinates_.push_back(
        w.get_node_pos(start_node).as_latlng());
    additional_edges_[start_node].push_back(
        additional_edge{.to_ = additional_node, .distance_ = 0U});
    additional_edges_[additional_node].push_back(
        additional_edge{.to_ = start_node, .distance_ = 0U});
  }

  sharing_data view(ways const& w) const {
    verify_additional_edge_count(additional_edges_, w.n_nodes());
    return {.start_allowed_ = &start_allowed_,
            .end_allowed_ = &end_allowed_,
            .through_allowed_ = &through_allowed_,
            .additional_node_offset_ = w.n_nodes(),
            .additional_node_coordinates_ = additional_node_coordinates_,
            .additional_edges_ = additional_edges_};
  }

  bitvec<node_idx_t> start_allowed_{};
  bitvec<node_idx_t> end_allowed_{};
  bitvec<node_idx_t> through_allowed_{};
  std::vector<geo::latlng> additional_node_coordinates_{};
  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_{};
};

TEST_F(sharing_routing_test, endpoint_matches_work_with_all_algorithms) {
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

TEST_F(sharing_routing_test, reconstructed_duration_excludes_matching_penalty) {
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

TEST_F(sharing_routing_test,
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

TEST_F(sharing_routing_test, closer_match_wins_over_graph_shortcut) {
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

TEST_F(sharing_routing_test,
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

TEST_F(sharing_routing_test, unreachable_closest_match_uses_farther_match) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const profile_params = car::parameters{};
  auto const params = profile_parameters{profile_params};
  auto const from = location{{49.010000, 8.000000}, kNoLevel};
  auto const to = location{{49.010040, 8.002800}, kNoLevel};

  auto to_matches = match_result{};
  l.match<car>(profile_params, to, true, direction::kForward, 50.0, nullptr,
               to_matches);
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

TEST_F(sharing_routing_test, matching_penalty_is_in_cost_limit) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const data = test_sharing_data{w, osm_node_idx_t{61U}};
  auto const sharing = data.view(w);
  auto const from = location{{49.020045, 8.000200}, kNoLevel};
  auto const to = location{{49.020000, 8.002800}, kNoLevel};
  auto const params = bike_sharing::parameters{};
  auto const penalized_options =
      route_options{.matching_penalty_factor_ = 100.0};
  auto from_matches = match_result{};
  l.match<bike_sharing>(params, from, false, direction::kForward, 50.0, nullptr,
                        from_matches);
  auto to_matches = match_result{};
  l.match<bike_sharing>(params, to, true, direction::kForward, 50.0, nullptr,
                        to_matches);
  ASSERT_GE(from_matches[match_idx_t{0U}].size(), 2U);
  auto& closest = from_matches.nodes_[match_result::way_candidate_idx_t{0U}];
  closest.left_.node_ = node_idx_t::invalid();
  closest.right_.node_ = node_idx_t::invalid();

  auto const baseline =
      route(profile_parameters{params}, w, l, search_profile::kBikeSharing,
            from, to, from_matches[match_idx_t{0U}],
            to_matches[match_idx_t{0U}], cost_t{3600U}, direction::kForward,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra,
            std::nullopt, route_options{.matching_penalty_factor_ = 0.0});
  ASSERT_TRUE(baseline.has_value());

  auto const penalized = route(
      profile_parameters{params}, w, l, search_profile::kBikeSharing, from, to,
      from_matches[match_idx_t{0U}], to_matches[match_idx_t{0U}], cost_t{3600U},
      direction::kForward, nullptr, &sharing, nullptr,
      routing_algorithm::kDijkstra, std::nullopt, penalized_options);
  ASSERT_TRUE(penalized.has_value());
  EXPECT_GT(penalized->cost_, baseline->cost_);
  EXPECT_EQ(
      penalized->cost_,
      std::accumulate(begin(penalized->segments_), end(penalized->segments_),
                      cost_t{0U}, [](cost_t const sum, path::segment const& s) {
                        return sum + s.cost_;
                      }));

  auto const limited = route(
      profile_parameters{params}, w, l, search_profile::kBikeSharing, from, to,
      from_matches[match_idx_t{0U}], to_matches[match_idx_t{0U}],
      clamp_cost(static_cast<std::uint64_t>(baseline->cost_) + 1U),
      direction::kForward, nullptr, &sharing, nullptr,
      routing_algorithm::kDijkstra, std::nullopt, penalized_options);
  EXPECT_FALSE(limited.has_value());
}

TEST_F(sharing_routing_test, endpoint_connection_cost_is_in_search_bound) {
  auto const& w = *ways_;
  auto const& l = *lookup_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const from = location{{49.000000, 8.000000}, kNoLevel};
  auto const to = location{{49.000000, 8.002800}, kNoLevel};
  auto const params = bike_sharing::parameters{};
  auto from_matches = match_result{};
  l.match<bike_sharing>(params, from, false, direction::kForward, 50.0, nullptr,
                        from_matches);
  auto to_matches = match_result{};
  l.match<bike_sharing>(params, to, true, direction::kForward, 50.0, nullptr,
                        to_matches);
  for (auto& nodes : from_matches.nodes_) {
    nodes.left_.dist_to_node_ = 5000.0F;
    nodes.right_.dist_to_node_ = 5000.0F;
  }

  auto result = std::optional<path>{};
  EXPECT_NO_THROW(result = route(profile_parameters{params}, w, l,
                                 search_profile::kBikeSharing, from, to,
                                 from_matches[match_idx_t{0U}],
                                 to_matches[match_idx_t{0U}], cost_t{30U},
                                 direction::kForward, nullptr, &sharing,
                                 nullptr, routing_algorithm::kDijkstra));
  EXPECT_FALSE(result.has_value());
}

TEST_F(sharing_routing_test, bike_does_not_switch_at_bike_inaccessible_node) {
  auto const& w = *ways_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const params = bike_sharing::parameters{};
  auto const node = w.find_node_idx(osm_node_idx_t{82U});
  ASSERT_TRUE(node.has_value());
  ASSERT_FALSE(w.r_->node_properties_[*node].is_bike_accessible());

  // make sure that we don't switch to bike at a node that is not bike
  // accessible in bwd search, because in fwd search the equivalent foot -> bike
  // switch also isn't allowed - needs to be consistent (although whether the
  // switch should be allowed in either direction is debatable)

  auto switched_to_bike = false;
  bike_sharing::adjacent<direction::kBackward, false>(
      params, *w.r_, timezone_cache_t{},
      bike_sharing::node{.n_ = *node,
                         .type_ = bike_sharing::node_type::kTrailingFoot,
                         .lvl_ = kNoLevel},
      duration_t{0U}, std::nullopt, nullptr, &sharing, nullptr,
      [&](bike_sharing::node const neighbor, auto...) {
        switched_to_bike |= neighbor.n_ == *node && neighbor.is_bike_node();
      });
  EXPECT_FALSE(switched_to_bike);
}

}  // namespace
}  // namespace osr
