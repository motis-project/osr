#include "gtest/gtest.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string_view>

#include "cista/mmap.h"

#include "osr/extract/extract.h"

#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "xml_to_pbf.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

constexpr auto const kNegativeIdsOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="-1" lat="48.0000" lon="9.0000"/>
  <node id="-2" lat="48.0000" lon="9.0010"/>
  <node id="-3" lat="48.0000" lon="9.0020"/>
  <node id="-4" lat="48.0010" lon="9.0020"/>
  <way id="-10">
    <nd ref="-2"/>
    <nd ref="-3"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Way -10"/>
  </way>
  <way id="-11">
    <nd ref="-1"/>
    <nd ref="-2"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Way -11"/>
  </way>
  <way id="20">
    <nd ref="-3"/>
    <nd ref="-4"/>
    <tag k="highway" v="primary"/>
  </way>
</osm>)osm";

constexpr auto const kHugePositiveIdsOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="35184372088821" lat="48.0000" lon="9.0000"/>
  <node id="35184372088822" lat="48.0000" lon="9.0010"/>
  <node id="35184372088823" lat="48.0000" lon="9.0020"/>
  <node id="35184372088824" lat="48.0010" lon="9.0020"/>
  <way id="35184372088831">
    <nd ref="35184372088821"/>
    <nd ref="35184372088822"/>
    <nd ref="35184372088823"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Huge Positive Way"/>
  </way>
  <way id="35184372088832">
    <nd ref="35184372088823"/>
    <nd ref="35184372088824"/>
    <tag k="highway" v="primary"/>
  </way>
</osm>)osm";

constexpr auto const kLowEmissionZoneOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="1" lat="0.0000" lon="0.0000"/>
  <node id="2" lat="0.0000" lon="0.0100"/>
  <node id="3" lat="0.0100" lon="0.0100"/>
  <node id="4" lat="0.0100" lon="0.0000"/>
  <node id="5" lat="0.0050" lon="0.0020"/>
  <node id="6" lat="0.0050" lon="0.0080"/>
  <node id="7" lat="0.0200" lon="0.0200"/>
  <node id="8" lat="0.0200" lon="0.0300"/>
  <node id="9" lat="0.0050" lon="-0.0020"/>
  <node id="10" lat="0.0050" lon="0.0120"/>
  <way id="10">
    <nd ref="5"/>
    <nd ref="6"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Inside"/>
  </way>
  <way id="20">
    <nd ref="7"/>
    <nd ref="8"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Outside"/>
  </way>
  <way id="30">
    <nd ref="9"/>
    <nd ref="10"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Crossing"/>
  </way>
  <way id="100">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <nd ref="4"/>
    <nd ref="1"/>
  </way>
  <relation id="1000">
    <member type="way" ref="100" role="outer"/>
    <tag k="type" v="boundary"/>
    <tag k="boundary" v="low_emission_zone"/>
    <tag k="name" v="Test LEZ"/>
  </relation>
</osm>)osm";

constexpr auto const kCrossingNodeAccessOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="1" lat="48.0000" lon="8.9990"/>
  <node id="2" lat="48.0000" lon="9.0000">
    <tag k="highway" v="crossing"/>
    <tag k="bicycle" v="no"/>
    <tag k="foot" v="no"/>
    <tag k="motor_vehicle" v="no"/>
    <tag k="bus" v="no"/>
  </node>
  <node id="3" lat="48.0000" lon="9.0010"/>
  <node id="4" lat="47.9990" lon="9.0000"/>
  <node id="5" lat="48.0010" lon="9.0000"/>

  <node id="11" lat="48.0100" lon="8.9990"/>
  <node id="12" lat="48.0100" lon="9.0000">
    <tag k="highway" v="crossing"/>
    <tag k="bicycle" v="no"/>
    <tag k="barrier" v="yes"/>
  </node>
  <node id="13" lat="48.0100" lon="9.0010"/>
  <node id="14" lat="48.0090" lon="9.0000"/>
  <node id="15" lat="48.0110" lon="9.0000"/>

  <node id="21" lat="48.0200" lon="8.9990"/>
  <node id="22" lat="48.0200" lon="9.0000">
    <tag k="highway" v="traffic_signals"/>
    <tag k="bicycle" v="no"/>
  </node>
  <node id="23" lat="48.0200" lon="9.0010"/>
  <node id="24" lat="48.0190" lon="9.0000"/>
  <node id="25" lat="48.0210" lon="9.0000"/>

  <way id="101">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="102">
    <nd ref="4"/>
    <nd ref="2"/>
    <nd ref="5"/>
    <tag k="highway" v="footway"/>
    <tag k="bicycle" v="no"/>
  </way>
  <way id="111">
    <nd ref="11"/>
    <nd ref="12"/>
    <nd ref="13"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="112">
    <nd ref="14"/>
    <nd ref="12"/>
    <nd ref="15"/>
    <tag k="highway" v="footway"/>
  </way>
  <way id="121">
    <nd ref="21"/>
    <nd ref="22"/>
    <nd ref="23"/>
    <tag k="highway" v="residential"/>
  </way>
  <way id="122">
    <nd ref="24"/>
    <nd ref="22"/>
    <nd ref="25"/>
    <tag k="highway" v="service"/>
  </way>
</osm>)osm";

struct crossing_node_access_test : public ::testing::Test {
  static void SetUpTestSuite() {
    dir_ = fs::temp_directory_path() / "osr_crossing_node_access_test";
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
    fs::create_directories(dir_, ec);
    extract(false,
            osr::test::write_osm_pbf("osr_crossing_node_access",
                                     kCrossingNodeAccessOsm)
                .generic_string(),
            dir_, {});
    ways_ = std::make_unique<ways>(dir_, cista::mmap::protection::READ);
  }

  static void TearDownTestSuite() {
    ways_.reset();
    auto ec = std::error_code{};
    fs::remove_all(dir_, ec);
  }

  static inline fs::path dir_{};
  static inline std::unique_ptr<ways> ways_{};
};

}  // namespace

TEST(extract, string_cache) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, osr::test::osm_to_pbf("test/map.osm").generic_string(), p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  // 140186757 and 519430215 have the same name=Pankratiusstraße
  const auto pankratius1 = w.find_way(osm_way_idx_t{140186757});
  const auto pankratius2 = w.find_way(osm_way_idx_t{519430215});
  ASSERT_TRUE(pankratius1.has_value());
  ASSERT_TRUE(pankratius2.has_value());

  const auto name_idx1 = w.way_names_[pankratius1.value()];
  const auto name_idx2 = w.way_names_[pankratius2.value()];
  ASSERT_EQ(name_idx1, name_idx2);
}

TEST(extract, bus_only_on_highway) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/luisenplatz-darmstadt.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto const luisenplatz_outer = w.find_way(osm_way_idx_t{53341306});
  ASSERT_TRUE(luisenplatz_outer.has_value());

  auto const& wp = w.r_->way_properties_[luisenplatz_outer.value()];
  ASSERT_FALSE(wp.is_bus_accessible());
  ASSERT_TRUE(wp.is_foot_accessible());
}

TEST(extract, standalone_ramp) {
  auto p = fs::temp_directory_path() / "osr_standalone_ramp_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false,
          osr::test::osm_to_pbf("test/standalone-ramp.osm").generic_string(), p,
          {});

  auto w = ways{p, cista::mmap::protection::READ};
  auto const ramp = w.find_way(osm_way_idx_t{1});
  auto const flat_footway = w.find_way(osm_way_idx_t{2});
  auto const inclined_path = w.find_way(osm_way_idx_t{3});
  ASSERT_TRUE(ramp.has_value());
  ASSERT_TRUE(flat_footway.has_value());
  ASSERT_TRUE(inclined_path.has_value());

  EXPECT_TRUE(w.r_->way_properties_[*ramp].is_ramp());
  EXPECT_FALSE(w.r_->way_properties_[*flat_footway].is_ramp());
  EXPECT_FALSE(w.r_->way_properties_[*inclined_path].is_ramp());
}

TEST(extract, supports_negative_custom_ids) {
  auto const osm_path =
      osr::test::write_osm_pbf("osr_negative_ids", kNegativeIdsOsm);
  auto const dir = fs::temp_directory_path() / "osr_negative_ids_dir";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  extract(false, osm_path.generic_string(), dir, {});

  auto w = ways{dir, cista::mmap::protection::READ};

  EXPECT_TRUE(std::is_sorted(
      begin(w.way_osm_idx_), end(w.way_osm_idx_),
      [](auto const lhs, auto const rhs) { return osm_id_less(lhs, rhs); }));

  auto const earlier_negative_way = w.find_way(to_osm_way_idx(-11));
  auto const negative_way = w.find_way(to_osm_way_idx(-10));
  auto const positive_way = w.find_way(osm_way_idx_t{20});
  auto const shared_node = w.find_node_idx(to_osm_node_idx(-3));

  ASSERT_TRUE(earlier_negative_way.has_value());
  ASSERT_TRUE(negative_way.has_value());
  ASSERT_TRUE(positive_way.has_value());
  ASSERT_TRUE(shared_node.has_value());
  EXPECT_EQ("Way -11", w.strings_[w.way_names_[*earlier_negative_way]].view());
  EXPECT_EQ("Way -10", w.strings_[w.way_names_[*negative_way]].view());
}

TEST(extract, supports_huge_positive_custom_ids) {
  auto const osm_path =
      osr::test::write_osm_pbf("osr_huge_positive_ids", kHugePositiveIdsOsm);
  auto const dir = fs::temp_directory_path() / "osr_huge_positive_ids_dir";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  extract(false, osm_path.generic_string(), dir, {});

  auto w = ways{dir, cista::mmap::protection::READ};
  auto const sparse_way = w.find_way(osm_way_idx_t{35184372088831ULL});
  auto const second_way = w.find_way(osm_way_idx_t{35184372088832ULL});
  auto const shared_node = w.find_node_idx(osm_node_idx_t{35184372088823ULL});

  ASSERT_TRUE(sparse_way.has_value());
  ASSERT_TRUE(second_way.has_value());
  ASSERT_TRUE(shared_node.has_value());
  EXPECT_EQ("Huge Positive Way", w.strings_[w.way_names_[*sparse_way]].view());
}

TEST(extract, marks_low_emission_zone_ways) {
  auto const osm_path =
      osr::test::write_osm_pbf("osr_low_emission_zone", kLowEmissionZoneOsm);
  auto const dir = fs::temp_directory_path() / "osr_low_emission_zone_dir";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  extract(false, osm_path.generic_string(), dir, {});

  auto w = ways{dir, cista::mmap::protection::READ};
  auto const inside = w.find_way(osm_way_idx_t{10});
  auto const outside = w.find_way(osm_way_idx_t{20});
  auto const crossing = w.find_way(osm_way_idx_t{30});

  ASSERT_TRUE(inside.has_value());
  ASSERT_TRUE(outside.has_value());
  ASSERT_TRUE(crossing.has_value());

  EXPECT_TRUE(w.r_->way_properties_[*inside].is_in_low_emission_zone());
  EXPECT_FALSE(w.r_->way_properties_[*outside].is_in_low_emission_zone());
  EXPECT_TRUE(w.r_->way_properties_[*crossing].is_in_low_emission_zone());
}

TEST_F(crossing_node_access_test, unbarriered_crossing_ignores_bicycle_access) {
  auto const node = ways_->find_node_idx(osm_node_idx_t{2});
  ASSERT_TRUE(node.has_value());
  EXPECT_TRUE(ways_->r_->node_properties_[*node].is_bike_accessible());
}

TEST_F(crossing_node_access_test, other_profiles_are_unchanged) {
  auto const node = ways_->find_node_idx(osm_node_idx_t{2});
  ASSERT_TRUE(node.has_value());
  auto const& properties = ways_->r_->node_properties_[*node];
  EXPECT_FALSE(properties.is_walk_accessible());
  EXPECT_FALSE(properties.is_car_accessible());
  EXPECT_FALSE(properties.is_bus_accessible());
}

TEST_F(crossing_node_access_test, barrier_keeps_bicycle_access_restriction) {
  auto const node = ways_->find_node_idx(osm_node_idx_t{12});
  ASSERT_TRUE(node.has_value());
  EXPECT_FALSE(ways_->r_->node_properties_[*node].is_bike_accessible());
}

TEST_F(crossing_node_access_test,
       non_crossing_keeps_bicycle_access_restriction) {
  auto const node = ways_->find_node_idx(osm_node_idx_t{22});
  ASSERT_TRUE(node.has_value());
  EXPECT_FALSE(ways_->r_->node_properties_[*node].is_bike_accessible());
}

TEST_F(crossing_node_access_test, way_access_is_unchanged) {
  auto const crossing = ways_->find_way(osm_way_idx_t{102});
  ASSERT_TRUE(crossing.has_value());
  EXPECT_FALSE(ways_->r_->way_properties_[*crossing].is_bike_accessible());
}
