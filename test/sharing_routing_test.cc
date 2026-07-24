#include <filesystem>
#include <iterator>
#include <memory>
#include <type_traits>
#include <vector>

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
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

template <typename Profile>
void verify_sharing_route(search_profile const profile,
                          mode const rental_mode) {
  auto const& w = *sharing_routing_test::ways_;
  auto const& l = *sharing_routing_test::lookup_;
  auto const data = test_sharing_data{w};
  auto const sharing = data.view(w);
  auto const from = location{{49.000000, 8.000000}, kNoLevel};
  auto const to = location{{49.000045, 8.002800}, kNoLevel};
  auto const params = typename Profile::parameters{};

  auto const forward =
      route(params, w, l, profile, from, to, 3600U, direction::kForward, 50.0,
            nullptr, &sharing, nullptr, routing_algorithm::kDijkstra);
  ASSERT_TRUE(forward.has_value());

  auto const backward_source_matches = l.match_endpoint<Profile>(
      params, to, false, direction::kBackward, 50.0, nullptr, false);
  ASSERT_GE(backward_source_matches.size(), 2U);
  auto const closest_foot =
      utl::find_if(backward_source_matches,
                   [](way_candidate const& wc) { return !wc.vehicle_match_; });
  auto const closest_vehicle =
      utl::find_if(backward_source_matches,
                   [](way_candidate const& wc) { return wc.vehicle_match_; });
  ASSERT_NE(end(backward_source_matches), closest_foot);
  ASSERT_NE(end(backward_source_matches), closest_vehicle);
  EXPECT_EQ(osm_way_idx_t{200U}, w.way_osm_idx_[closest_foot->way_]);
  EXPECT_EQ(osm_way_idx_t{100U}, w.way_osm_idx_[closest_vehicle->way_]);

  auto const station_matches = l.match_endpoint<Profile>(
      params, to, false, direction::kBackward, 50.0, nullptr, false,
      std::nullopt, std::nullopt, false);
  EXPECT_TRUE(utl::none_of(station_matches, [](way_candidate const& candidate) {
    return candidate.vehicle_match_;
  }));

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

  auto const expected_return_node = w.find_node_idx(osm_node_idx_t{4U}).value();
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

  auto const endpoint_matches = l.match_endpoint<Profile>(
      params, to, true, direction::kForward, 50.0, nullptr, true, std::nullopt,
      std::nullopt, false);
  auto const regular_road =
      utl::find_if(endpoint_matches, [&](way_candidate const& m) {
        return w.way_osm_idx_[m.way_] == osm_way_idx_t{100U} &&
               !m.vehicle_match_;
      });
  auto const exact_road =
      utl::find_if(endpoint_matches, [&](way_candidate const& m) {
        return w.way_osm_idx_[m.way_] == osm_way_idx_t{100U} &&
               m.vehicle_match_;
      });
  ASSERT_NE(end(endpoint_matches), regular_road);
  ASSERT_NE(end(endpoint_matches), exact_road);
  EXPECT_FALSE(regular_road->exact_return_);
  EXPECT_TRUE(exact_road->exact_return_);

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

  auto const matches = l.match_endpoint<Profile>(
      params, to, true, direction::kForward, 50.0, nullptr, false);
  auto const closest_foot = utl::find_if(
      matches,
      [](way_candidate const& match) { return !match.vehicle_match_; });
  auto const closest_vehicle = utl::find_if(
      matches, [](way_candidate const& match) { return match.vehicle_match_; });
  auto const fallback_foot =
      utl::find_if(matches, [&](way_candidate const& match) {
        return !match.vehicle_match_ &&
               w.way_osm_idx_[match.way_] == osm_way_idx_t{300U};
      });
  ASSERT_NE(end(matches), closest_foot);
  ASSERT_NE(end(matches), closest_vehicle);
  ASSERT_NE(end(matches), fallback_foot);
  EXPECT_EQ(osm_way_idx_t{400U}, w.way_osm_idx_[closest_foot->way_]);
  EXPECT_EQ(osm_way_idx_t{400U}, w.way_osm_idx_[closest_vehicle->way_]);
  EXPECT_EQ(w.r_->way_component_[closest_foot->way_],
            w.r_->way_component_[fallback_foot->way_]);

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

  auto const matches = l.match_endpoint<Profile>(
      params, from, false, direction::kForward, 50.0, nullptr, false);
  auto const closest_foot = utl::find_if(
      matches,
      [](way_candidate const& match) { return !match.vehicle_match_; });
  auto const closest_vehicle = utl::find_if(
      matches, [](way_candidate const& match) { return match.vehicle_match_; });
  auto const fallback_foot =
      utl::find_if(matches, [&](way_candidate const& match) {
        return !match.vehicle_match_ &&
               w.way_osm_idx_[match.way_] == osm_way_idx_t{500U};
      });
  ASSERT_NE(end(matches), closest_foot);
  ASSERT_NE(end(matches), closest_vehicle);
  ASSERT_NE(end(matches), fallback_foot);
  EXPECT_EQ(osm_way_idx_t{600U}, w.way_osm_idx_[closest_foot->way_]);
  EXPECT_EQ(osm_way_idx_t{600U}, w.way_osm_idx_[closest_vehicle->way_]);
  EXPECT_EQ(w.r_->way_component_[closest_foot->way_],
            w.r_->way_component_[fallback_foot->way_]);

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

}  // namespace
}  // namespace osr
