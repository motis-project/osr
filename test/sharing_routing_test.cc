#include <numeric>
#include <vector>

#include "gtest/gtest.h"

#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"

#include "sharing_routing_fixture.h"

namespace osr {
namespace {

using sharing_routing_test = test::sharing_routing_fixture;

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
                        false, from_matches);
  auto to_matches = match_result{};
  l.match<bike_sharing>(params, to, true, direction::kForward, 50.0, nullptr,
                        false, to_matches);
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
                        false, from_matches);
  auto to_matches = match_result{};
  l.match<bike_sharing>(params, to, true, direction::kForward, 50.0, nullptr,
                        false, to_matches);
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

struct levelled_sharing_test : testing::Test {
  void SetUp() override {
    std::filesystem::create_directories(dir_);
    extract(false, test::write_osm_pbf("osr-levelled-sharing", R"(
<osm version="0.6">
  <node id="1" lat="49" lon="8"/>
  <node id="2" lat="49" lon="8.001"/>
  <node id="3" lat="49" lon="8.0013"/>
  <node id="4" lat="49" lon="8.0043"/>
  <node id="5" lat="49" lon="8.0046"/>
  <node id="6" lat="49" lon="8.0056"/>
  <node id="7" lat="49.002" lon="8"/>
  <node id="8" lat="49.002" lon="8.0013"/>
  <node id="9" lat="49.002" lon="8.0046"/>
  <way id="1"><nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="footway"/><tag k="level" v="0"/></way>
  <way id="2"><nd ref="3"/><nd ref="4"/>
    <tag k="highway" v="residential"/></way>
  <way id="3"><nd ref="5"/><nd ref="6"/>
    <tag k="highway" v="footway"/><tag k="level" v="1"/></way>
  <!-- Longer parallel ways retain the endpoints as routing junctions. -->
  <way id="4"><nd ref="1"/><nd ref="7"/><nd ref="2"/>
    <tag k="highway" v="footway"/><tag k="level" v="0"/></way>
  <way id="5"><nd ref="3"/><nd ref="8"/><nd ref="4"/>
    <tag k="highway" v="residential"/></way>
  <way id="6"><nd ref="5"/><nd ref="9"/><nd ref="6"/>
    <tag k="highway" v="footway"/><tag k="level" v="1"/></way>
</osm>)"),
            dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
    auto const size = w_->n_nodes() + 2U;
    start_.resize(size);
    end_.resize(size);
    through_.resize(size);
    through_.one_out();
    start_.set(node_idx_t{w_->n_nodes()}, true);
    end_.set(node_idx_t{w_->n_nodes() + 1U}, true);
    for (auto const id : {2U, 3U, 4U, 5U}) {
      auto const graph = w_->find_node_idx(osm_node_idx_t{id}).value();
      auto const station = node_idx_t{w_->n_nodes() + (id >= 4U ? 1U : 0U)};
      edges_[graph].push_back(
          additional_edge{.to_ = station, .distance_ = 10U});
      edges_[station].push_back(
          additional_edge{.to_ = graph, .distance_ = 10U});
    }
    coordinates_ = {{49, 8.00115}, {49, 8.00445}};
  }

  void TearDown() override {
    l_.reset();
    w_.reset();
    std::filesystem::remove_all(dir_);
  }

  void check(search_profile const profile) const {
    auto const sharing =
        sharing_data{.start_allowed_ = &start_,
                     .end_allowed_ = &end_,
                     .through_allowed_ = &through_,
                     .additional_node_offset_ = w_->n_nodes(),
                     .additional_node_coordinates_ = coordinates_,
                     .additional_edges_ = edges_};
    auto const params = get_parameters(profile);
    for (auto const explicit_levels : {false, true}) {
      auto const from =
          location{{49, 8.001}, explicit_levels ? level_t{0.F} : kNoLevel};
      auto const to =
          location{{49, 8.0046}, explicit_levels ? level_t{1.F} : kNoLevel};
      auto const fwd = route(params, *w_, *l_, profile, from, to, 3600U,
                             direction::kForward, 2.0, nullptr, &sharing);
      auto const bwd = route(params, *w_, *l_, profile, to, from, 3600U,
                             direction::kBackward, 2.0, nullptr, &sharing);
      ASSERT_TRUE(fwd.has_value());
      ASSERT_TRUE(bwd.has_value());
      EXPECT_EQ(fwd->cost_, bwd->cost_);
      EXPECT_EQ(fwd->duration_, bwd->duration_);
      // Both endpoints are directly at their station connector. Walking along
      // a footway and back would only serve to change the routing level state.
      for (auto const* p : {&*fwd, &*bwd}) {
        auto road_segments = 0U;
        for (auto const& segment : p->segments_) {
          if (segment.way_ != way_idx_t::invalid()) {
            EXPECT_EQ(w_->way_osm_idx_[segment.way_], osm_way_idx_t{2U});
            ++road_segments;
          }
        }
        EXPECT_EQ(road_segments, 1U);
      }
    }
  }

  std::filesystem::path dir_{std::filesystem::temp_directory_path() /
                             "osr-levelled-sharing"};
  std::unique_ptr<ways> w_;
  std::unique_ptr<lookup> l_;
  bitvec<node_idx_t> start_, end_, through_;
  std::vector<geo::latlng> coordinates_;
  hash_map<node_idx_t, std::vector<additional_edge>> edges_;
};

TEST_F(levelled_sharing_test, bike_pickup_and_return_preserve_walking_levels) {
  check(search_profile::kBikeSharing);
}

TEST_F(levelled_sharing_test, car_pickup_and_return_preserve_walking_levels) {
  check(search_profile::kCarSharing);
}

}  // namespace
}  // namespace osr
