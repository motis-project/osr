#include "gtest/gtest.h"

#include <cstdint>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "cista/mmap.h"

#include "fmt/core.h"

#include "utl/overloaded.h"

#include "osr/extract/extract.h"

#include "osr/lookup.h"
#include "osr/platforms.h"
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

TEST(extract, ignores_ways_exceeding_max_ways_per_node) {
  auto const p = fs::temp_directory_path() / "osr_24_way_node_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/39-way-node.osm.pbf", p, {});

  auto w = ways{p, cista::mmap::protection::READ};
  for (auto n = node_idx_t{0U}; n != w.n_nodes(); ++n) {
    auto const ways = w.r_->node_ways_[n];
    auto const in_way_idx = w.r_->node_in_way_idx_[n];
    EXPECT_LE(ways.size(), kMaxWaysPerNode);
    ASSERT_EQ(ways.size(), in_way_idx.size());
    for (auto i = std::size_t{0U}; i != ways.size(); ++i) {
      auto const way = ways[i];
      ASSERT_LT(to_idx(way), w.r_->way_nodes_.size()) << "dangling way index";
      ASSERT_LT(in_way_idx[i], w.r_->way_nodes_[way].size())
          << "dangling node-in-way index";
      EXPECT_EQ(n, w.r_->way_nodes_[way][in_way_idx[i]])
          << "node_ways_/node_in_way_idx_ does not point back to this node";
    }
  }
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

  // Way indices follow the way order; osm_way_order_ lists them by OSM id.
  EXPECT_TRUE(std::is_sorted(begin(w.osm_way_order_), end(w.osm_way_order_),
                             [&](auto const lhs, auto const rhs) {
                               return osm_id_less(w.way_osm_idx_[lhs],
                                                  w.way_osm_idx_[rhs]);
                             }));

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

// platforms::reorder_ways() has to follow ways::order_ways(): the platform bit
// per way and the way references of the platforms must name the renumbered
// ways. The spokes get their OSM ids from east to west, so the way order
// (Morton code of the midpoint, here west to east) reverses them.
TEST(extract, platform_way_refs_follow_the_way_order) {
  constexpr auto const kSpokes = 8U;
  auto xml = std::string{R"(<osm version="0.6" generator="test">
)"};
  // Street nodes 10..19 along lat 48.0; spoke i hangs south of node 19 - i.
  for (auto k = 0U; k != 10U; ++k) {
    xml += fmt::format(R"(  <node id="{}" lat="48.0000" lon="{:.4f}"/>
)",
                       10U + k, 9.0 + 0.001 * k);
  }
  for (auto i = 1U; i <= kSpokes; ++i) {
    xml += fmt::format(R"(  <node id="{}" lat="47.9995" lon="{:.4f}"/>
)",
                       20U + i, 9.0 + 0.001 * (9U - i));
  }
  // A platform node in the middle of spoke 4.
  xml += R"(  <node id="30" lat="47.9998" lon="9.0050">
    <tag k="public_transport" v="platform"/>
    <tag k="name" v="Node platform"/>
  </node>
  <way id="100">
    <nd ref="10"/><nd ref="11"/><nd ref="12"/><nd ref="13"/><nd ref="14"/>
    <nd ref="15"/><nd ref="16"/><nd ref="17"/><nd ref="18"/><nd ref="19"/>
    <tag k="highway" v="residential"/>
  </way>
)";
  // Spokes 2, 5 and 7 are named platform ways, the others plain footways.
  for (auto i = 1U; i <= kSpokes; ++i) {
    auto const platform = i == 2U || i == 5U || i == 7U;
    auto const tags =
        platform ? fmt::format(R"(    <tag k="public_transport" v="platform"/>
    <tag k="name" v="Platform {}"/>
)",
                               i)
                 : std::string{R"(    <tag k="highway" v="footway"/>
)"};
    xml += fmt::format(R"(  <way id="{}">
    <nd ref="{}"/>{}<nd ref="{}"/>
{}  </way>
)",
                       i, 19U - i, i == 4U ? R"(<nd ref="30"/>)" : "", 20U + i,
                       tags);
  }
  // A platform relation over the unnamed spokes 3 and 6: they get a reference
  // in the relation's platform, but no platform bit of their own.
  xml += R"(  <relation id="200">
    <member type="way" ref="3" role=""/>
    <member type="way" ref="6" role=""/>
    <tag k="type" v="multipolygon"/>
    <tag k="public_transport" v="platform"/>
    <tag k="name" v="Relation platform"/>
  </relation>
</osm>)";

  auto const dir = fs::temp_directory_path() / "osr_platform_way_order_dir";
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  extract(
      true,
      osr::test::write_osm_pbf("osr_platform_way_order", xml).generic_string(),
      dir, {});

  auto w = ways{dir, cista::mmap::protection::READ};
  auto pl = platforms{dir, cista::mmap::protection::READ};

  // The test only shows something if the way order differs from the OSM id
  // order.
  ASSERT_FALSE(std::is_sorted(
      begin(w.way_osm_idx_), end(w.way_osm_idx_),
      [](auto const a, auto const b) { return osm_id_less(a, b); }));

  auto const osm_way = [&](way_idx_t const x) {
    return static_cast<std::int64_t>(cista::to_idx(w.way_osm_idx_[x]));
  };
  auto const osm_node = [&](node_idx_t const x) {
    return static_cast<std::int64_t>(cista::to_idx(w.node_to_osm_[x]));
  };

  // The platform bits name the three named platform ways.
  auto marked = std::vector<std::int64_t>{};
  for (auto i = 0U; i != pl.way_is_platform_.size(); ++i) {
    if (pl.way_is_platform_.test(way_idx_t{i})) {
      marked.push_back(osm_way(way_idx_t{i}));
    }
  }
  std::sort(begin(marked), end(marked));
  EXPECT_EQ((std::vector<std::int64_t>{2, 5, 7}), marked);

  // The references: one platform per named way, the relation's platform with
  // its two member ways, and the node platform.
  auto way_refs = std::vector<std::vector<std::int64_t>>{};
  auto node_refs = std::vector<std::int64_t>{};
  for (auto const refs : pl.platform_ref_) {
    auto ways_of_platform = std::vector<std::int64_t>{};
    for (auto const ref : refs) {
      std::visit(utl::overloaded{[&](way_idx_t const x) {
                                   ways_of_platform.push_back(osm_way(x));
                                 },
                                 [&](node_idx_t const x) {
                                   EXPECT_TRUE(pl.node_is_platform_.test(x));
                                   node_refs.push_back(osm_node(x));
                                 }},
                 to_ref(ref));
    }
    if (!ways_of_platform.empty()) {
      std::sort(begin(ways_of_platform), end(ways_of_platform));
      way_refs.push_back(ways_of_platform);
    }
  }
  std::sort(begin(way_refs), end(way_refs));
  EXPECT_EQ((std::vector<std::vector<std::int64_t>>{{2}, {3, 6}, {5}, {7}}),
            way_refs);
  EXPECT_EQ((std::vector<std::int64_t>{30}), node_refs);
}
