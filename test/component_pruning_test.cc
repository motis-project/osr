#include "gtest/gtest.h"

#include <cstdint>

#include <chrono>
#include <filesystem>
#include <optional>
#include <span>
#include <vector>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/reachability.h"
#include "osr/routing/route.h"
#include "osr/routing/sharing_data.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {

// Mainland: ways 100-102. Island: ways 200-201. Way 300 is a single-way car
// component: only footways 301 and 302 connect to it. Ways 400-401 are a
// fragment next to way 102.
constexpr auto const kPruningOsm = R"(<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000"/>
  <node id="3" lat="49.000000" lon="8.004000"/>
  <node id="4" lat="49.002000" lon="8.004000"/>
  <node id="11" lat="49.010000" lon="8.000000"/>
  <node id="12" lat="49.010000" lon="8.002000"/>
  <node id="13" lat="49.010000" lon="8.004000"/>
  <node id="21" lat="49.020000" lon="8.000000"/>
  <node id="22" lat="49.020000" lon="8.002000"/>
  <node id="25" lat="49.021000" lon="8.000000"/>
  <node id="26" lat="49.021000" lon="8.002000"/>
  <node id="31" lat="49.001200" lon="8.004300"/>
  <node id="32" lat="49.002000" lon="8.004300"/>
  <node id="33" lat="49.002300" lon="8.004300"/>
  <way id="100">
    <nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="101">
    <nd ref="2"/><nd ref="3"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="102">
    <nd ref="3"/><nd ref="4"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="200">
    <nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="201">
    <nd ref="12"/><nd ref="13"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="300">
    <nd ref="21"/><nd ref="22"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="301">
    <nd ref="21"/><nd ref="25"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="302">
    <nd ref="22"/><nd ref="26"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="400">
    <nd ref="31"/><nd ref="32"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="401">
    <nd ref="32"/><nd ref="33"/>
    <tag k="highway" v="residential"/>
  </way>
</osm>
)";

constexpr auto const kMainlandA = location{{49.000020, 8.000500}, kNoLevel};
constexpr auto const kMainlandB = location{{49.001500, 8.004020}, kNoLevel};
constexpr auto const kIsland = location{{49.010020, 8.003000}, kNoLevel};
constexpr auto const kSingleWayA = location{{49.020020, 8.000400}, kNoLevel};
constexpr auto const kSingleWayB = location{{49.020020, 8.001600}, kNoLevel};

constexpr auto const kPruningMaxDuration = std::chrono::seconds{3600};
constexpr auto const kPruningMaxMatchDistance = 50.0;

struct component_pruning_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "component_pruning_test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("component_pruning_test", kPruningOsm),
            dir_, {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static std::optional<path> route_1_1(search_profile const p,
                                       location const& from,
                                       location const& to,
                                       routing_algorithm const algo) {
    return route(get_parameters(p), *w_, *l_, p, from, to, kPruningMaxDuration,
                 direction::kForward, kPruningMaxMatchDistance, nullptr,
                 nullptr, nullptr, algo);
  }

  static std::vector<std::optional<path>> route_1_n(
      search_profile const p,
      location const& from,
      std::vector<location> const& to) {
    return route(get_parameters(p), *w_, *l_, p, from, to, kPruningMaxDuration,
                 direction::kForward, kPruningMaxMatchDistance, nullptr,
                 nullptr, nullptr);
  }

  static way_idx_t way(std::uint64_t const osm) {
    return *w_->find_way(osm_way_idx_t{osm});
  }

  static match_view_t match(location const& query,
                            bool const reverse,
                            match_result& out) {
    l_->match<foot<false>>(foot<false>::parameters{}, query, reverse,
                           direction::kForward, kPruningMaxMatchDistance,
                           nullptr, out);
    return out[match_idx_t{0U}];
  }

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

TEST_F(component_pruning_test, single_way_component_has_no_id) {
  auto const way = w_->find_way(osm_way_idx_t{300U});
  ASSERT_TRUE(way.has_value());
  EXPECT_FALSE(w_->r_->get_class_components(component_class::kCar)
                   .get(*way)
                   .has_value());
  EXPECT_TRUE(w_->r_->get_class_components(component_class::kFoot)
                  .get(*way)
                  .has_value());
}

TEST_F(component_pruning_test, is_connected) {
  auto const& car = w_->r_->get_class_components(component_class::kCar);
  EXPECT_TRUE(car.is_connected(way(100U), way(102U)));
  EXPECT_FALSE(car.is_connected(way(100U), way(200U)));
  EXPECT_TRUE(car.is_connected(way(300U), way(300U)));
  EXPECT_FALSE(car.is_connected(way(300U), way(100U)));
  EXPECT_FALSE(car.is_connected(way(100U), way(300U)));
  EXPECT_FALSE(car.is_connected(way(300U), way(301U)));

  auto const& foot = w_->r_->get_class_components(component_class::kFoot);
  EXPECT_TRUE(foot.is_connected(way(300U), way(301U)));
  EXPECT_TRUE(foot.is_connected(way(301U), way(302U)));
  EXPECT_FALSE(foot.is_connected(way(300U), way(100U)));
}

TEST_F(component_pruning_test, connected_candidates) {
  using P = foot<false>;
  auto mainland_a_m = match_result{};
  auto mainland_b_m = match_result{};
  auto island_m = match_result{};
  auto const mainland_a = match(kMainlandA, false, mainland_a_m);
  auto const mainland_b = match(kMainlandB, true, mainland_b_m);
  auto const island = match(kIsland, true, island_m);
  ASSERT_EQ(1U, mainland_a.size());
  ASSERT_EQ(2U, mainland_b.size());
  EXPECT_EQ(way(102U), mainland_b.way_[0]);
  EXPECT_EQ(way(400U), mainland_b.way_[1]);

  auto const connected = [](match_view_t const& m,
                            std::span<way_idx_t const> other_ways,
                            sharing_data const* sharing = nullptr) {
    return connected_candidates<P>(*w_, sharing, m, other_ways);
  };

  EXPECT_TRUE(connected(mainland_a, island.way_)[match_idx_t{0U}].empty());
  EXPECT_TRUE(connected(island, mainland_a.way_)[match_idx_t{0U}].empty());

  auto const a = connected(mainland_a, mainland_b.way_);
  ASSERT_EQ(1U, a[match_idx_t{0U}].size());
  EXPECT_EQ(way(100U), a[match_idx_t{0U}].way_[0]);

  auto const b = connected(mainland_b, mainland_a.way_);
  ASSERT_EQ(1U, b[match_idx_t{0U}].size());
  EXPECT_EQ(way(102U), b[match_idx_t{0U}].way_[0]);
  EXPECT_EQ(mainland_b.penalty_ref_, b[match_idx_t{0U}].penalty_ref_);

  auto const coordinates = std::vector<geo::latlng>{};
  auto const edges = hash_map<node_idx_t, std::vector<additional_edge>>{};
  auto const sharing = sharing_data{.additional_node_coordinates_ = coordinates,
                                    .additional_edges_ = edges};
  EXPECT_EQ(
      mainland_b.size(),
      connected(mainland_b, island.way_, &sharing)[match_idx_t{0U}].size());
}

TEST_F(component_pruning_test, routes_only_within_components) {
  auto const points =
      std::vector{kMainlandA, kMainlandB, kIsland, kSingleWayA, kSingleWayB};
  auto const component = std::vector{0U, 0U, 1U, 2U, 2U};
  for (auto const p :
       {search_profile::kFoot, search_profile::kBike, search_profile::kCar}) {
    for (auto i = 0U; i != points.size(); ++i) {
      auto const one_to_many = route_1_n(p, points[i], points);
      ASSERT_EQ(points.size(), one_to_many.size());
      for (auto j = 0U; j != points.size(); ++j) {
        auto const reachable = component[i] == component[j];
        EXPECT_EQ(reachable, one_to_many[j].has_value())
            << to_str(p) << " " << i << " -> " << j;
        for (auto const algo :
             {routing_algorithm::kAStarBi, routing_algorithm::kDijkstra}) {
          EXPECT_EQ(reachable,
                    route_1_1(p, points[i], points[j], algo).has_value())
              << to_str(p) << " " << i << " -> " << j;
        }
      }
    }
  }
}

}  // namespace osr
