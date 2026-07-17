#include "gtest/gtest.h"

#include <chrono>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

#include "fmt/format.h"

#include "osr/extract/extract.h"

#include "xml_to_pbf.h"
#include "osr/lookup.h"
#include "osr/routing/path.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/hgv.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

namespace {

constexpr auto const kFrom = osr::location{48.0, 9.0, osr::kNoLevel};
constexpr auto const kTo = osr::location{48.0, 9.01, osr::kNoLevel};

std::string two_route_osm(std::string_view const direct_tags,
                          std::string_view const relations = {}) {
  return fmt::format(
      R"osm(<osm version="0.6" generator="test">
  <node id="1" version="1" lat="48.0000" lon="9.0000"/>
  <node id="2" version="1" lat="48.0000" lon="9.0010"/>
  <node id="3" version="1" lat="48.0000" lon="9.0090"/>
  <node id="4" version="1" lat="48.0000" lon="9.0100"/>
  <node id="5" version="1" lat="48.0003" lon="9.0050"/>
  <way id="1" version="1">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
  </way>
  <way id="2" version="1">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="highway" v="primary"/>
  </way>
  <way id="10" version="1">
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="maxspeed" v="80"/>
    {}
  </way>
  <way id="20" version="1">
    <nd ref="2"/>
    <nd ref="5"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="maxspeed" v="80"/>
  </way>
  {}
</osm>)osm",
      direct_tags, relations);
}

constexpr auto const kLowEmissionZoneOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="1" version="1" lat="48.0000" lon="9.0000"/>
  <node id="2" version="1" lat="48.0000" lon="9.0010"/>
  <node id="3" version="1" lat="48.0000" lon="9.0090"/>
  <node id="4" version="1" lat="48.0000" lon="9.0100"/>
  <node id="5" version="1" lat="47.9980" lon="9.0050"/>
  <node id="101" version="1" lat="47.9995" lon="9.0030"/>
  <node id="102" version="1" lat="47.9995" lon="9.0070"/>
  <node id="103" version="1" lat="48.0005" lon="9.0070"/>
  <node id="104" version="1" lat="48.0005" lon="9.0030"/>
  <way id="1" version="1">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
  </way>
  <way id="2" version="1">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="highway" v="primary"/>
  </way>
  <way id="10" version="1">
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="maxspeed" v="80"/>
  </way>
  <way id="20" version="1">
    <nd ref="2"/>
    <nd ref="5"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="maxspeed" v="80"/>
  </way>
  <way id="100" version="1">
    <nd ref="101"/>
    <nd ref="102"/>
    <nd ref="103"/>
    <nd ref="104"/>
    <nd ref="101"/>
  </way>
  <relation id="1000" version="1">
    <!-- the LEZ covers part of way 10 -->
    <member type="way" ref="100" role="outer"/>
    <tag k="type" v="boundary"/>
    <tag k="boundary" v="low_emission_zone"/>
  </relation>
</osm>)osm";

class extracted_graph {
public:
  extracted_graph(std::string_view const name, std::string_view const osm) {
    auto const temp = fs::temp_directory_path();
    auto const osm_path = osr::test::write_osm_pbf(name, osm);
    dir_ = temp / fmt::format("{}_dir", name);

    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);

    osr::extract(false, osm_path.generic_string(), dir_, {});
    ways_ = std::make_unique<osr::ways>(dir_, cista::mmap::protection::READ);
    lookup_ = std::make_unique<osr::lookup>(*ways_, dir_,
                                            cista::mmap::protection::READ);
  }

  osr::path route(osr::profile_parameters const& params,
                  osr::search_profile const profile,
                  osr::location const& from,
                  osr::location const& to,
                  std::optional<osr::routing_time_t> const start_time =
                      std::nullopt) const {
    auto result =
        osr::route(params, *ways_, *lookup_, profile, from, to, 10'000U,
                   osr::direction::kForward, 50.0, nullptr, nullptr, nullptr,
                   osr::routing_algorithm::kDijkstra, start_time);
    EXPECT_TRUE(result.has_value());
    return result.value_or(osr::path{});
  }

  bool uses_way(osr::path const& path, std::uint64_t const osm_way) const {
    return std::ranges::any_of(path.segments_, [&](auto const& segment) {
      return segment.way_ != osr::way_idx_t::invalid() &&
             cista::to_idx(ways_->way_osm_idx_[segment.way_]) == osm_way;
    });
  }

  osr::ways& ways() { return *ways_; }

private:
  fs::path dir_;
  std::unique_ptr<osr::ways> ways_;
  std::unique_ptr<osr::lookup> lookup_;
};

TEST(hgv_routing, low_emission_zone_access_changes_route) {
  auto graph = extracted_graph{"osr_hgv_lez_routing", kLowEmissionZoneOsm};

  auto const with_access =
      graph.route(osr::hgv::parameters{.low_emission_zone_access_ = true},
                  osr::search_profile::kHgv, kFrom, kTo);
  EXPECT_TRUE(graph.uses_way(with_access, 10U));
  EXPECT_FALSE(graph.uses_way(with_access, 20U));

  auto const without_access =
      graph.route(osr::hgv::parameters{.low_emission_zone_access_ = false},
                  osr::search_profile::kHgv, kFrom, kTo);
  EXPECT_FALSE(graph.uses_way(without_access, 10U));
  EXPECT_TRUE(graph.uses_way(without_access, 20U));
}

TEST(hgv_routing, detour_relation_is_extracted_and_preferred) {
  auto const osm = two_route_osm({}, R"osm(<relation id="100" version="1">
    <member type="way" ref="20" role=""/>
    <tag k="type" v="route"/>
    <tag k="route" v="detour"/>
  </relation>)osm");
  auto graph = extracted_graph{"osr_hgv_detour_routing", osm};

  auto const direct = graph.ways().find_way(osr::osm_way_idx_t{10U});
  auto const detour = graph.ways().find_way(osr::osm_way_idx_t{20U});
  ASSERT_TRUE(direct.has_value());
  ASSERT_TRUE(detour.has_value());
  EXPECT_FALSE(graph.ways().r_->way_properties_[*direct].is_detour());
  EXPECT_TRUE(graph.ways().r_->way_properties_[*detour].is_detour());

  // in this test, the detour is slighty longer than the direct route
  // the current implementation prefers it, because it's marked as a detour
  // (subject to change)
  auto const route = graph.route(osr::hgv::parameters{},
                                 osr::search_profile::kHgv, kFrom, kTo);
  EXPECT_FALSE(graph.uses_way(route, 10U));
  EXPECT_TRUE(graph.uses_way(route, 20U));
}

TEST(hgv_routing, directional_hgv_access_is_extracted_and_routed) {
  auto const osm = two_route_osm(
      R"osm(<tag k="hgv:forward" v="no"/>
    <tag k="hgv:backward" v="yes"/>)osm");
  auto graph = extracted_graph{"osr_hgv_directional_access", osm};

  auto const direct = graph.ways().find_way(osr::osm_way_idx_t{10U});
  ASSERT_TRUE(direct.has_value());
  auto const* info = graph.ways().r_->get_hgv_info(*direct);
  ASSERT_NE(nullptr, info);
  EXPECT_EQ(osr::access_value::kNo, info->hgv_access_fwd());
  EXPECT_EQ(osr::access_value::kYes, info->hgv_access_bwd());

  auto const forward = graph.route(osr::hgv::parameters{},
                                   osr::search_profile::kHgv, kFrom, kTo);
  EXPECT_FALSE(graph.uses_way(forward, 10U));
  EXPECT_TRUE(graph.uses_way(forward, 20U));

  auto const backward = graph.route(osr::hgv::parameters{},
                                    osr::search_profile::kHgv, kTo, kFrom);
  EXPECT_TRUE(graph.uses_way(backward, 10U));
  EXPECT_FALSE(graph.uses_way(backward, 20U));
}

TEST(hgv_routing, reverse_oneway_is_respected_by_car_and_hgv) {
  auto const osm = two_route_osm(R"osm(<tag k="oneway" v="-1"/>)osm");
  auto graph = extracted_graph{"osr_reverse_oneway", osm};

  auto const direct = graph.ways().find_way(osr::osm_way_idx_t{10U});
  ASSERT_TRUE(direct.has_value());
  auto const properties = graph.ways().r_->way_properties_[*direct];
  EXPECT_TRUE(properties.is_oneway_car());
  EXPECT_TRUE(properties.is_oneway_reverse());

  auto const check_profile = [&](osr::profile_parameters const& params,
                                 osr::search_profile const profile) {
    auto const forward = graph.route(params, profile, kFrom, kTo);
    EXPECT_FALSE(graph.uses_way(forward, 10U));
    EXPECT_TRUE(graph.uses_way(forward, 20U));

    auto const backward = graph.route(params, profile, kTo, kFrom);
    EXPECT_TRUE(graph.uses_way(backward, 10U));
    EXPECT_FALSE(graph.uses_way(backward, 20U));
  };

  check_profile(osr::car::parameters{}, osr::search_profile::kCar);
  check_profile(osr::hgv::parameters{}, osr::search_profile::kHgv);
}

TEST(hgv_routing, conditional_reverse_oneway_changes_route) {
  auto const osm =
      two_route_osm(R"osm(<tag k="oneway:conditional" v="-1 @ (24/7)"/>)osm");
  auto graph = extracted_graph{"osr_conditional_reverse_oneway", osm};
  auto const start_time = osr::routing_time_t{std::chrono::seconds{0}};

  auto const forward =
      graph.route(osr::hgv::parameters{}, osr::search_profile::kHgv, kFrom, kTo,
                  start_time);
  EXPECT_FALSE(graph.uses_way(forward, 10U));
  EXPECT_TRUE(graph.uses_way(forward, 20U));

  auto const backward =
      graph.route(osr::hgv::parameters{}, osr::search_profile::kHgv, kTo, kFrom,
                  start_time);
  EXPECT_TRUE(graph.uses_way(backward, 10U));
  EXPECT_FALSE(graph.uses_way(backward, 20U));
}

TEST(hgv_routing, conditional_no_overrides_reverse_oneway) {
  auto const osm = two_route_osm(
      R"osm(<tag k="oneway" v="-1"/>
    <tag k="oneway:conditional" v="no @ (24/7)"/>)osm");
  auto graph = extracted_graph{"osr_conditional_no_oneway", osm};
  auto const start_time = osr::routing_time_t{std::chrono::seconds{0}};

  auto const forward =
      graph.route(osr::hgv::parameters{}, osr::search_profile::kHgv, kFrom, kTo,
                  start_time);
  EXPECT_TRUE(graph.uses_way(forward, 10U));
  EXPECT_FALSE(graph.uses_way(forward, 20U));
}

}  // namespace
