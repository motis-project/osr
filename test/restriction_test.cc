#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <optional>
#include <utility>

#include "cista/mmap.h"

#include "fmt/core.h"
#include "fmt/ranges.h"

#include "osr/extract/extract.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/profile.h"
#include "osr/routing/route.h"
#include "osr/types.h"
#include "osr/ways.h"

#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

TEST(extract, wa) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  osr::extract(false, osr::test::osm_to_pbf("test/map.osm").generic_string(), p,
               "test/restriction_test_elevation/");

  auto w = osr::ways{p, cista::mmap::protection::READ};
  auto l = osr::lookup{w, p, cista::mmap::protection::READ};
  auto const elevations = elevation_storage::try_open(p);
  ASSERT_TRUE(elevations);

  auto const n = w.find_node_idx(osm_node_idx_t{528944});
  auto const rhoenring = w.find_way(osm_way_idx_t{120682496});
  auto const arheilger = w.find_way(osm_way_idx_t{1068971150});
  // Crossing Eckhardt / Barkhaus
  auto const n_dst = w.find_node_idx(osm_node_idx_t{586157});

  ASSERT_TRUE(n.has_value());
  ASSERT_TRUE(n_dst.has_value());
  auto const from = location{w.get_node_pos(*n), kNoLevel};
  auto const to = location{w.get_node_pos(*n_dst), kNoLevel};
  constexpr auto const kMaxDuration = std::chrono::seconds{3600};
  constexpr auto const kMaxMatchDistance = 100;
  constexpr auto const kParamsNoCosts =
      bike<bike_costing::kSafe, kElevationNoCost>::parameters{};
  constexpr auto const kParamsHighCosts =
      bike<bike_costing::kSafe, kElevationHighCost>::parameters{};
  auto const route_no_costs =
      route(kParamsNoCosts, w, l, search_profile::kBike, from, to, kMaxDuration,
            direction::kForward, kMaxMatchDistance, nullptr, nullptr,
            elevations.get());
  auto const route_high_costs =
      route(kParamsHighCosts, w, l, search_profile::kBikeElevationHigh, from,
            to, kMaxDuration, direction::kForward, kMaxMatchDistance, nullptr,
            nullptr, elevations.get());

  auto const is_restricted = w.r_->is_restricted<osr::direction::kForward>(
      n.value(), w.r_->get_way_pos(n.value(), rhoenring.value()),
      w.r_->get_way_pos(n.value(), arheilger.value()));
  EXPECT_TRUE(is_restricted);

  constexpr auto const kShortestDistance = 163.0;
  ASSERT_TRUE(route_no_costs.has_value());
  EXPECT_TRUE(std::abs(route_no_costs->dist_ - kShortestDistance) < 2.0);
  // Upper bounds for elevations on each segment
  EXPECT_EQ(elevation_monotonic_t{4U + 1U}, route_no_costs->elevation_.up_);
  EXPECT_EQ(elevation_monotonic_t{0U + 6U}, route_no_costs->elevation_.down_);

  ASSERT_TRUE(route_high_costs.has_value());
  EXPECT_TRUE(route_high_costs->dist_ - kShortestDistance > 2.0);
  // Upper bounds for elevations on each segment
  EXPECT_EQ(elevation_monotonic_t{1U + 0U}, route_high_costs->elevation_.up_);
  EXPECT_EQ(elevation_monotonic_t{4U + 0U}, route_high_costs->elevation_.down_);
}

// A turn restriction between ways at positions >= 16 of a node. Which ways
// end up at those positions depends on the way order, so the star is
// extracted twice: once to see which ways land at positions 18 and 19, then
// with the restriction between them.
TEST(extract, restriction_at_high_way_position) {
  auto const p = fs::temp_directory_path() / "osr_restriction_high_way_pos";
  auto ec = std::error_code{};

  constexpr auto const kWays = 20U;
  auto const make_xml =
      [&](std::optional<std::pair<std::int64_t, std::int64_t>> const
              restriction) {
        auto xml = std::string{R"(<osm version="0.6" generator="test">
  <node id="1" lat="48.0" lon="9.0"/>
)"};
        for (auto i = 0U; i != kWays; ++i) {
          xml += fmt::format(R"(  <node id="{}" lat="{}" lon="{}"/>
)",
                             i + 2U, 48.0 + 0.001 * std::cos(i * 6.28 / kWays),
                             9.0 + 0.001 * std::sin(i * 6.28 / kWays));
        }
        for (auto i = 0U; i != kWays; ++i) {
          xml += fmt::format(R"(  <way id="{}">
    <nd ref="1"/><nd ref="{}"/>
    <tag k="highway" v="residential"/>
  </way>
)",
                             i + 1U, i + 2U);
        }
        if (restriction.has_value()) {
          xml += fmt::format(R"(  <relation id="1">
    <member type="way" ref="{}" role="from"/>
    <member type="node" ref="1" role="via"/>
    <member type="way" ref="{}" role="to"/>
    <tag k="type" v="restriction"/>
    <tag k="restriction" v="no_left_turn"/>
  </relation>
)",
                             restriction->first, restriction->second);
        }
        xml += "</osm>";
        return xml;
      };
  auto const extract = [&](std::string_view const name,
                           std::string const& xml) {
    fs::remove_all(p, ec);
    osr::extract(false, osr::test::write_osm_pbf(name, xml), p, {});
  };

  extract("osr_restriction_high_way_pos_probe", make_xml(std::nullopt));
  auto const [from_way, to_way] = [&] {
    auto w = osr::ways{p, cista::mmap::protection::READ};
    auto const center = w.get_node_idx(osr::to_osm_node_idx(1));
    auto const osm_way = [&](way_pos_t const pos) {
      return static_cast<std::int64_t>(
          cista::to_idx(w.way_osm_idx_[w.r_->node_ways_[center][pos]]));
    };
    return std::pair{osm_way(18U), osm_way(19U)};
  }();

  extract("osr_restriction_high_way_pos",
          make_xml(std::pair{from_way, to_way}));
  auto w = osr::ways{p, cista::mmap::protection::READ};
  auto const center = w.get_node_idx(osr::to_osm_node_idx(1));
  auto const pos = [&](std::int64_t const osm_way) {
    return w.r_->get_way_pos(center, *w.find_way(osr::to_osm_way_idx(osm_way)));
  };
  ASSERT_GE(pos(from_way), 16U);
  ASSERT_GE(pos(to_way), 16U);
  EXPECT_TRUE(w.r_->is_restricted<osr::direction::kForward>(
      center, pos(from_way), pos(to_way)));
  EXPECT_FALSE(w.r_->is_restricted<osr::direction::kForward>(
      center, pos(from_way) - 16U, pos(to_way) - 16U));  // no wrap-around
}
