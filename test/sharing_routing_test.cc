#include <filesystem>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/astar.h"
#include "osr/routing/bidirectional.h"
#include "osr/routing/dijkstra.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
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

template <typename Fn>
std::optional<std::size_t> find_match(match_view_t const& matches, Fn&& fn) {
  for (auto i = std::size_t{0U}; i != matches.size(); ++i) {
    if (fn(i)) {
      return i;
    }
  }
  return std::nullopt;
}

using tracked_car_sharing = car_sharing<track_node_tracking>;

struct car_return_transition {
  node_idx_t return_node_;
  tracked_car_sharing::node rental_;
  tracked_car_sharing::node trailing_foot_;
};

car_return_transition get_car_return_transition(ways const& w) {
  // Returning the rental car at node 4 changes to trailing-foot mode.  That
  // mode switch is the edge which marks node 4 as the tracked return node.
  auto const return_node = w.find_node_idx(osm_node_idx_t{4U}).value();
  auto rental = std::optional<tracked_car_sharing::node>{};
  tracked_car_sharing::resolve_all(
      *w.r_, return_node, kNoLevel, [&](tracked_car_sharing::node const n) {
        if (!rental.has_value() && n.is_rental_node()) {
          rental = n;
        }
      });
  return {
      .return_node_ = return_node,
      .rental_ = rental.value(),
      .trailing_foot_ = {.n_ = return_node,
                         .type_ = tracked_car_sharing::node_type::kTrailingFoot,
                         .lvl_ = kNoLevel}};
}

void expect_tracked_return_node(tracked_car_sharing::entry const& entry,
                                tracked_car_sharing::node const trailing_foot,
                                node_idx_t const return_node) {
  auto result = path{};
  entry.write(trailing_foot, result);
  EXPECT_EQ(return_node, result.track_node_);
}

template <typename Profile>
void verify_sharing_route(search_profile const profile,
                          mode const rental_mode) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const from = location{{49.000000, 8.000000}, kNoLevel};
  auto const to = location{{49.000005, 8.009800}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto const forward =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(forward.has_value());

  auto backward_source_result = match_result{};
  l.match_endpoint<Profile>(params, to, false, direction::kBackward, 50.0,
                            nullptr, false, backward_source_result);
  auto const backward_source_matches = backward_source_result[match_idx_t{0U}];
  ASSERT_GE(backward_source_matches.size(), 2U);
  auto const closest_foot = find_match(
      backward_source_matches,
      [&](auto const i) { return !backward_source_matches.vehicle_match(i); });
  ASSERT_TRUE(closest_foot.has_value());
  EXPECT_FALSE(find_match(backward_source_matches, [&](auto const i) {
                 return backward_source_matches.vehicle_match(i);
               }).has_value());

  auto const backward =
      route(params, w, l, profile, to, from, 3600U, direction::kBackward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(backward.has_value());
  EXPECT_EQ(forward->cost_, backward->cost_);
  EXPECT_EQ(forward->duration_, backward->duration_);

  auto const backward_many =
      route(params, w, l, profile, to, std::vector{from}, 3600U,
            direction::kBackward, 50.0, nullptr, &sharing, nullptr);
  ASSERT_EQ(1U, backward_many.size());
  ASSERT_TRUE(backward_many.front().has_value());
  EXPECT_EQ(forward->cost_, backward_many.front()->cost_);
  EXPECT_EQ(forward->duration_, backward_many.front()->duration_);
  EXPECT_TRUE(backward_many.front()->segments_.empty());

  auto const expected_return = w.find_node_idx(osm_node_idx_t{5U});
  ASSERT_TRUE(expected_return.has_value());
  auto const expected_return_node = *expected_return;
  auto const switch_segment =
      utl::find_if(forward->segments_, [&](path::segment const& s) {
        return s.from_ == expected_return_node &&
               s.to_ == expected_return_node && s.dist_ == 0U &&
               s.cost_ == Profile::kEndSwitchPenalty;
      });
  ASSERT_NE(end(forward->segments_), switch_segment);
  EXPECT_EQ(rental_mode, switch_segment->mode_);

  auto const connector = std::next(switch_segment);
  ASSERT_NE(end(forward->segments_), connector);
  EXPECT_EQ(mode::kFoot, connector->mode_);
  EXPECT_EQ(way_idx_t::invalid(), connector->way_);
  EXPECT_GT(connector->dist_, 0U);
  ASSERT_FALSE(connector->polyline_.empty());
  EXPECT_NEAR(49.0, connector->polyline_.back().lat_, 1e-6);
  EXPECT_EQ(end(forward->segments_), std::next(connector));

  if constexpr (std::is_same_v<Profile, car_sharing<track_node_tracking>>) {
    EXPECT_EQ(expected_return_node, forward->track_node_);
  }
}

template <typename Profile>
void verify_exact_node_return(search_profile const profile,
                              mode const rental_mode) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const from = location{{49.000000, 8.000000}, kNoLevel};
  auto const to = location{{49.000000, 8.003000}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto const result =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->segments_.empty());
  auto const& last = result->segments_.back();
  auto const expected_return_node = w.find_node_idx(osm_node_idx_t{4U}).value();
  EXPECT_EQ(rental_mode, last.mode_);
  EXPECT_EQ(expected_return_node, last.from_);
  EXPECT_EQ(expected_return_node, last.to_);
  EXPECT_EQ(0U, last.dist_);
  EXPECT_EQ(Profile::kEndSwitchPenalty, last.cost_);
}

template <typename Profile>
void verify_exact_coordinate_return(search_profile const profile,
                                    mode const rental_mode) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const from = location{{49.000000, 8.000000}, kNoLevel};
  auto const to = location{{49.000045, 8.002800}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto endpoint_result = match_result{};
  l.match_endpoint<Profile>(params, to, true, direction::kForward, 50.0,
                            nullptr, true, endpoint_result, std::nullopt,
                            std::nullopt);
  auto const endpoint_matches = endpoint_result[match_idx_t{0U}];
  auto const regular_road = find_match(endpoint_matches, [&](auto const i) {
    return w.way_osm_idx_[endpoint_matches.way_[i]] == osm_way_idx_t{100U} &&
           !endpoint_matches.vehicle_match(i);
  });
  auto const exact_road = find_match(endpoint_matches, [&](auto const i) {
    return w.way_osm_idx_[endpoint_matches.way_[i]] == osm_way_idx_t{100U} &&
           endpoint_matches.vehicle_match(i);
  });
  ASSERT_TRUE(regular_road.has_value());
  ASSERT_TRUE(exact_road.has_value());
  EXPECT_FALSE(endpoint_matches.exact_return(*regular_road));
  EXPECT_TRUE(endpoint_matches.exact_return(*exact_road));

  auto const regular =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(regular.has_value());
  ASSERT_EQ(mode::kFoot, regular->segments_.back().mode_);

  auto const exact = route(
      params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
      nullptr, &sharing, nullptr, routing_algorithm::kDijkstra, std::nullopt,
      route_endpoint_options{.exact_return_at_to_ = {true}});
  ASSERT_TRUE(exact.has_value());
  EXPECT_LT(exact->cost_, regular->cost_);
  ASSERT_FALSE(exact->segments_.empty());
  auto const& connector = exact->segments_.back();
  EXPECT_EQ(rental_mode, connector.mode_);
  EXPECT_EQ(way_idx_t::invalid(), connector.way_);
  EXPECT_GE(connector.cost_, Profile::kEndSwitchPenalty);
  ASSERT_FALSE(connector.polyline_.empty());
  EXPECT_EQ(to.pos_, connector.polyline_.back());

  auto const backward = route(
      params, w, l, profile, to, from, 3600U, direction::kBackward, 50.0,
      nullptr, &sharing, nullptr, routing_algorithm::kDijkstra, std::nullopt,
      route_endpoint_options{.exact_return_at_from_ = true});
  ASSERT_TRUE(backward.has_value());
  EXPECT_EQ(exact->cost_, backward->cost_);
  EXPECT_EQ(exact->duration_, backward->duration_);

  auto const backward_many = route(
      params, w, l, profile, to, std::vector{from}, 3600U, direction::kBackward,
      50.0, nullptr, &sharing, nullptr, [](path const&) { return true; },
      std::nullopt, route_endpoint_options{.exact_return_at_from_ = true});
  ASSERT_EQ(1U, backward_many.size());
  ASSERT_TRUE(backward_many.front().has_value());
  EXPECT_EQ(exact->cost_, backward_many.front()->cost_);
  EXPECT_EQ(exact->duration_, backward_many.front()->duration_);
  ASSERT_FALSE(backward_many.front()->segments_.empty());
  EXPECT_EQ(to.pos_, backward_many.front()->segments_.back().polyline_.back());

  if constexpr (std::is_same_v<Profile, car_sharing<track_node_tracking>>) {
    EXPECT_EQ(node_idx_t::invalid(), exact->track_node_);
    EXPECT_EQ(node_idx_t::invalid(), backward->track_node_);
  }
}

template <typename Profile>
void verify_destination_match_fallback(search_profile const profile) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w, osm_node_idx_t{31U}};
  auto const sharing = data.view(w);
  auto const from = location{{49.010000, 8.000000}, kNoLevel};
  auto const to = location{{49.010045, 8.002800}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto match_result = osr::match_result{};
  l.match_endpoint<Profile>(params, to, true, direction::kForward, 50.0,
                            nullptr, false, match_result);
  auto const matches = match_result[match_idx_t{0U}];
  auto const closest_foot = find_match(
      matches, [&](auto const i) { return !matches.vehicle_match(i); });
  auto const fallback_foot = find_match(matches, [&](auto const i) {
    return !matches.vehicle_match(i) &&
           w.way_osm_idx_[matches.way_[i]] == osm_way_idx_t{300U};
  });
  ASSERT_TRUE(closest_foot.has_value());
  ASSERT_TRUE(fallback_foot.has_value());
  EXPECT_EQ(osm_way_idx_t{400U}, w.way_osm_idx_[matches.way_[*closest_foot]]);
  EXPECT_EQ(w.r_->way_component_[matches.way_[*closest_foot]],
            w.r_->way_component_[matches.way_[*fallback_foot]]);

  auto const result =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->segments_.empty());
}

template <typename Profile>
void verify_source_match_fallback(search_profile const profile) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w, osm_node_idx_t{61U}};
  auto const sharing = data.view(w);
  auto const from = location{{49.020045, 8.000200}, kNoLevel};
  auto const to = location{{49.020000, 8.002800}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto match_result = osr::match_result{};
  l.match_endpoint<Profile>(params, from, false, direction::kForward, 50.0,
                            nullptr, false, match_result);
  auto const matches = match_result[match_idx_t{0U}];
  auto const closest_foot = find_match(
      matches, [&](auto const i) { return !matches.vehicle_match(i); });
  auto const fallback_foot = find_match(matches, [&](auto const i) {
    return !matches.vehicle_match(i) &&
           w.way_osm_idx_[matches.way_[i]] == osm_way_idx_t{500U};
  });
  ASSERT_TRUE(closest_foot.has_value());
  ASSERT_TRUE(fallback_foot.has_value());
  EXPECT_EQ(osm_way_idx_t{600U}, w.way_osm_idx_[matches.way_[*closest_foot]]);
  EXPECT_EQ(w.r_->way_component_[matches.way_[*closest_foot]],
            w.r_->way_component_[matches.way_[*fallback_foot]]);

  auto const result =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->segments_.empty());
}

TEST_F(sharing_routing_test, bike_returns_at_destination_road_node) {
  verify_sharing_route<bike_sharing>(search_profile::kBikeSharing, mode::kBike);
}

TEST_F(sharing_routing_test, car_returns_at_destination_road_node) {
  verify_sharing_route<car_sharing<track_node_tracking>>(
      search_profile::kCarSharing, mode::kCar);
}

TEST_F(sharing_routing_test, dijkstra_tracks_car_return_node) {
  auto const& w = *ways_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const transition = get_car_return_transition(w);

  auto search = dijkstra<tracked_car_sharing>{};
  search.reset(3600U);
  search.add_start(w,
                   tracked_car_sharing::label{transition.rental_, cost_t{0U}});
  search.run(tracked_car_sharing::parameters{}, w, *w.r_, 3600U, std::nullopt,
             nullptr, &sharing, nullptr, direction::kForward);

  expect_tracked_return_node(
      search.cost_.at(transition.trailing_foot_.get_key()),
      transition.trailing_foot_, transition.return_node_);
}

TEST_F(sharing_routing_test, astar_tracks_car_return_node) {
  auto const& w = *ways_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const transition = get_car_return_transition(w);
  auto const location = osr::location{w.get_node_pos(transition.return_node_)};
  auto const params = tracked_car_sharing::parameters{};

  auto search = astar<tracked_car_sharing>{};
  search.reset(3600U, location, location);
  search.add_destination(params, w, &sharing, transition.trailing_foot_);
  search.add_start(params, w, &sharing,
                   tracked_car_sharing::label{transition.rental_, cost_t{0U}});
  search.run(params, w, *w.r_, 3600U, nullptr, &sharing, nullptr,
             direction::kForward);

  expect_tracked_return_node(
      search.cost_.at(transition.trailing_foot_.get_key()),
      transition.trailing_foot_, transition.return_node_);
}

TEST_F(sharing_routing_test, bidirectional_tracks_car_return_node) {
  auto const& w = *ways_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const transition = get_car_return_transition(w);
  auto const location = osr::location{w.get_node_pos(transition.return_node_)};
  auto const params = tracked_car_sharing::parameters{};

  auto search = bidirectional<tracked_car_sharing>{};
  search.reset(params, 3600U, location, location);
  search.add_start(params, w,
                   tracked_car_sharing::label{transition.rental_, cost_t{0U}},
                   &sharing);
  search.run_single<direction::kForward, false, direction::kForward>(
      params, w, *w.r_, 3600U, nullptr, &sharing, nullptr, search.pq1_,
      search.cost1_);

  expect_tracked_return_node(
      search.cost1_.at(transition.trailing_foot_.get_key()),
      transition.trailing_foot_, transition.return_node_);
}

TEST_F(sharing_routing_test, bike_return_at_exact_node_has_no_walk_segment) {
  verify_exact_node_return<bike_sharing>(search_profile::kBikeSharing,
                                         mode::kBike);
}

TEST_F(sharing_routing_test, car_return_at_exact_node_has_no_walk_segment) {
  verify_exact_node_return<car_sharing<track_node_tracking>>(
      search_profile::kCarSharing, mode::kCar);
}

TEST_F(sharing_routing_test, bike_can_return_at_exact_coordinate) {
  verify_exact_coordinate_return<bike_sharing>(search_profile::kBikeSharing,
                                               mode::kBike);
}

TEST_F(sharing_routing_test, car_can_return_at_exact_coordinate) {
  verify_exact_coordinate_return<car_sharing<track_node_tracking>>(
      search_profile::kCarSharing, mode::kCar);
}

TEST_F(sharing_routing_test, bike_falls_back_to_second_destination_match) {
  verify_destination_match_fallback<bike_sharing>(search_profile::kBikeSharing);
}

TEST_F(sharing_routing_test, car_falls_back_to_second_destination_match) {
  verify_destination_match_fallback<car_sharing<track_node_tracking>>(
      search_profile::kCarSharing);
}

TEST_F(sharing_routing_test, bike_falls_back_to_second_source_match) {
  verify_source_match_fallback<bike_sharing>(search_profile::kBikeSharing);
}

TEST_F(sharing_routing_test, car_falls_back_to_second_source_match) {
  verify_source_match_fallback<car_sharing<track_node_tracking>>(
      search_profile::kCarSharing);
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
