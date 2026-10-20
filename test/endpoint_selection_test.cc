#include "gtest/gtest.h"

#include <chrono>
#include <filesystem>
#include <memory>
#include <vector>

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/endpoint_selection.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;

namespace osr {

// Foot networks:
// - street 100..105 (6 ways) along lat 49.0
// - footway 200..202 (3 ways) 33 m north of it: it leaves at 8.0035, goes
//   1 km north and comes back at 8.0065
// - street 300..304 (5 ways) 66 m north of the street, not connected
constexpr auto const kOsm = R"(
<osm version="0.6" generator="osr-test">
  <node id="1" lat="49.000000" lon="8.000000"/>
  <node id="2" lat="49.000000" lon="8.002000"/>
  <node id="3" lat="49.000000" lon="8.004000"/>
  <node id="4" lat="49.000000" lon="8.006000"/>
  <node id="5" lat="49.000000" lon="8.008000"/>
  <node id="6" lat="49.000000" lon="8.010000"/>
  <node id="7" lat="49.000000" lon="8.012000"/>
  <node id="10" lat="49.000300" lon="8.004500"/>
  <node id="11" lat="49.000300" lon="8.003500"/>
  <node id="12" lat="49.010000" lon="8.003500"/>
  <node id="13" lat="49.010000" lon="8.006500"/>
  <node id="14" lat="49.000300" lon="8.006500"/>
  <node id="15" lat="49.000300" lon="8.005500"/>
  <node id="20" lat="49.000600" lon="8.007000"/>
  <node id="21" lat="49.000600" lon="8.007600"/>
  <node id="22" lat="49.000600" lon="8.008200"/>
  <node id="23" lat="49.000600" lon="8.008800"/>
  <node id="24" lat="49.000600" lon="8.009400"/>
  <node id="25" lat="49.000600" lon="8.010000"/>
  <way id="100"><nd ref="1"/><nd ref="2"/><tag k="highway" v="residential"/></way>
  <way id="101"><nd ref="2"/><nd ref="3"/><tag k="highway" v="residential"/></way>
  <way id="102"><nd ref="3"/><nd ref="4"/><tag k="highway" v="residential"/></way>
  <way id="103"><nd ref="4"/><nd ref="5"/><tag k="highway" v="residential"/></way>
  <way id="104"><nd ref="5"/><nd ref="6"/><tag k="highway" v="residential"/></way>
  <way id="105"><nd ref="6"/><nd ref="7"/><tag k="highway" v="residential"/></way>
  <way id="200">
    <nd ref="10"/><nd ref="11"/><nd ref="12"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="201"><nd ref="12"/><nd ref="13"/><tag k="highway" v="footway"/></way>
  <way id="202">
    <nd ref="13"/><nd ref="14"/><nd ref="15"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="300"><nd ref="20"/><nd ref="21"/><tag k="highway" v="residential"/></way>
  <way id="301"><nd ref="21"/><nd ref="22"/><tag k="highway" v="residential"/></way>
  <way id="302"><nd ref="22"/><nd ref="23"/><tag k="highway" v="residential"/></way>
  <way id="303"><nd ref="23"/><nd ref="24"/><tag k="highway" v="residential"/></way>
  <way id="304"><nd ref="24"/><nd ref="25"/><tag k="highway" v="residential"/></way>
</osm>
)";

constexpr auto const kOnFootwayWest = location{{49.000250, 8.004000}, kNoLevel};
constexpr auto const kOnFootwayEast = location{{49.000250, 8.006000}, kNoLevel};
constexpr auto const kStreetEast = location{{49.000000, 8.009500}, kNoLevel};
constexpr auto const kStreetWest = location{{49.000000, 8.001000}, kNoLevel};
constexpr auto const kNorthStreet = location{{49.000500, 8.008500}, kNoLevel};

constexpr auto const kTinyBelow4 = route_options{.tiny_component_ways_ = 4U};

struct endpoint_selection_test : ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr-endpoint-selection";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false, test::write_osm_pbf("osr-endpoint-selection", kOsm), dir_,
            {});
    w_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
    l_ = std::make_unique<lookup>(*w_, dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    l_.reset();
    w_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static std::optional<path> route(location const& from,
                                   location const& to,
                                   route_options const& options = kTinyBelow4,
                                   double const max_match_distance = 50.0) {
    return osr::route(get_parameters(search_profile::kFoot), *w_, *l_,
                      search_profile::kFoot, from, to,
                      std::chrono::seconds{3600}, direction::kForward,
                      max_match_distance, nullptr, nullptr, nullptr,
                      routing_algorithm::kDijkstra, std::nullopt, options);
  }

  static std::vector<std::optional<path>> route(
      location const& from, std::vector<location> const& to) {
    return osr::route(
        get_parameters(search_profile::kFoot), *w_, *l_, search_profile::kFoot,
        from, to, std::chrono::seconds{3600}, direction::kForward, 50.0,
        nullptr, nullptr, nullptr, [](path const&) { return true; },
        std::nullopt, kTinyBelow4);
  }

  static match_view_t match(location const& query,
                            bool const reverse,
                            match_result& out) {
    l_->match<foot_t>(foot_t::parameters{}, query, reverse, direction::kForward,
                      50.0, nullptr, out);
    return out[match_idx_t{static_cast<match_idx_t::value_t>(out.size() - 1U)}];
  }

  static way_idx_t way(std::uint64_t const osm) {
    return *w_->find_way(osm_way_idx_t{osm});
  }

  using foot_t = foot<false, elevator_tracking>;

  static inline fs::path dir_;
  static inline std::unique_ptr<ways> w_;
  static inline std::unique_ptr<lookup> l_;
};

TEST_F(endpoint_selection_test, component_sizes) {
  auto const& c = w_->r_->get_class_components(component_class::kFoot);
  auto const size = [&](std::uint64_t const osm) {
    return c.size(*c.get(way(osm)));
  };
  EXPECT_EQ(6U, size(100));
  EXPECT_EQ(3U, size(201));
  EXPECT_EQ(5U, size(304));
}

// Both ends are closest to the tiny footway: the route stays on it (2.4 km)
// although the street would be shorter.
TEST_F(endpoint_selection_test, shared_tiny_component_is_kept) {
  auto const p = route(kOnFootwayWest, kOnFootwayEast);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(2052U, p->cost_);
  EXPECT_EQ(2048U, p->duration_.count());
  EXPECT_NEAR(2449.1, p->dist_, 0.1);
}

// The other end is not on the footway: the start moves to the street 28 m
// away. Its matching penalty is measured from the street, so the cost is the
// street route (334) plus 28.
TEST_F(endpoint_selection_test, tiny_component_switches_to_big) {
  auto const p = route(kOnFootwayWest, kStreetEast);
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(362U, p->cost_);
  EXPECT_EQ(357U, p->duration_.count());
  EXPECT_NEAR(401.3, p->dist_, 0.1);
}

// With the default threshold, every component of this map is tiny, but the
// largest one counts as big.
TEST_F(endpoint_selection_test, largest_component_is_never_tiny) {
  EXPECT_TRUE(route(kOnFootwayWest, kStreetEast, route_options{}).has_value());
}

// Nothing within 25 m: the lookup finds the footway at 55 m and the street at
// 88 m, more than 25 m farther away. With 100 m, the start may switch.
TEST_F(endpoint_selection_test, switch_distance_is_capped) {
  auto const north_of_footway = location{{49.000795, 8.0043}, kNoLevel};
  EXPECT_FALSE(
      route(north_of_footway, kStreetEast, kTinyBelow4, 25.0).has_value());
  EXPECT_TRUE(
      route(north_of_footway, kStreetEast, kTinyBelow4, 100.0).has_value());
}

// The cap applies to foot and bike components, not to car components, and
// counts from the closest component.
TEST(choose_component, car_switches_without_cap) {
  auto e = match_components{.components_ = {}, .max_switch_ = 250.F};
  auto o = match_components{};
  for (auto const c : kAllComponentClasses) {
    e.components_.push_back(
        {.class_ = c, .key_ = 7U, .dist_ = 10.F, .tiny_ = true});
    e.components_.push_back(
        {.class_ = c, .key_ = 1U, .dist_ = 400.F, .tiny_ = false});
    o.components_.push_back(
        {.class_ = c, .key_ = 1U, .dist_ = 5.F, .tiny_ = false});
  }
  EXPECT_EQ(7U, choose_component(e, o, component_class::kFoot));
  EXPECT_EQ(1U, choose_component(e, o, component_class::kCar));

  for (auto& x : e.components_) {
    if (x.tiny_) {
      x.dist_ = 300.F;
    }
  }
  EXPECT_EQ(1U, choose_component(e, o, component_class::kFoot));
}

// The closest component is big: the street 55 m away is not considered.
TEST_F(endpoint_selection_test, closest_big_component_is_not_left) {
  EXPECT_FALSE(
      route(kNorthStreet, kStreetWest, kTinyBelow4, 100.0).has_value());
}

// The start keeps the footway for the destination on it and the street for
// the other one: both routes are the same as the 1:1 routes.
TEST_F(endpoint_selection_test, one_to_many_source_keeps_both_components) {
  auto const results = route(kOnFootwayWest, {kOnFootwayEast, kStreetEast});
  ASSERT_EQ(2U, results.size());
  ASSERT_TRUE(results[0].has_value());
  ASSERT_TRUE(results[1].has_value());
  EXPECT_EQ(2052U, results[0]->cost_);
  EXPECT_EQ(2048U, results[0]->duration_.count());
  EXPECT_EQ(362U, results[1]->cost_);
  EXPECT_EQ(357U, results[1]->duration_.count());
}

// Each component the start keeps measures the matching penalty from its own
// closest candidate.
TEST_F(endpoint_selection_test, source_measures_each_component_separately) {
  auto from = match_result{};
  auto to = match_result{};
  auto const f = match(kOnFootwayWest, false, from);
  match(kOnFootwayEast, true, to);
  match(kStreetEast, true, to);
  ASSERT_EQ(3U, f.size());

  auto const sel = select_endpoints<foot_t>(*w_, f, to, direction::kForward,
                                            kTinyBelow4.tiny_component_ways_);
  ASSERT_TRUE(sel.has_value());
  auto const s = sel->from_[match_idx_t{0U}];
  ASSERT_EQ(3U, s.size());
  EXPECT_EQ(way(200U), s.way_[0]);
  EXPECT_EQ(way(102U), s.way_[1]);
  EXPECT_EQ(way(101U), s.way_[2]);
  EXPECT_NEAR(5.56F, s.penalty_ref_[0], 0.01F);
  EXPECT_NEAR(27.80F, s.penalty_ref_[1], 0.01F);
  EXPECT_NEAR(27.80F, s.penalty_ref_[2], 0.01F);

  ASSERT_EQ(1U, sel->to_[match_idx_t{0U}].size());
  EXPECT_EQ(way(202U), sel->to_[match_idx_t{0U}].way_[0]);
  ASSERT_EQ(1U, sel->to_[match_idx_t{1U}].size());
  EXPECT_EQ(way(104U), sel->to_[match_idx_t{1U}].way_[0]);
}

// Both ends only have candidates on the street: nothing is copied.
TEST_F(endpoint_selection_test, nothing_to_choose) {
  auto from = match_result{};
  auto to = match_result{};
  auto const f = match(kStreetWest, false, from);
  match(kStreetEast, true, to);
  EXPECT_FALSE(select_endpoints<foot_t>(*w_, f, to, direction::kForward,
                                        kTinyBelow4.tiny_component_ways_)
                   .has_value());
}

}  // namespace osr
