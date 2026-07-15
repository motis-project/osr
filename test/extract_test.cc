#include "gtest/gtest.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string_view>

#include "cista/mmap.h"

#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;

namespace {

constexpr auto const kNegativeIdsOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="-1" lat="48.0000" lon="9.0000"/>
  <node id="-2" lat="48.0000" lon="9.0010"/>
  <node id="-3" lat="48.0000" lon="9.0020"/>
  <node id="-4" lat="48.0010" lon="9.0020"/>
  <way id="-11">
    <nd ref="-1"/>
    <nd ref="-2"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Way -11"/>
  </way>
  <way id="-10">
    <nd ref="-2"/>
    <nd ref="-3"/>
    <tag k="highway" v="primary"/>
    <tag k="name" v="Way -10"/>
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

fs::path write_osm(std::string_view const name,
                   std::string_view const content) {
  auto const path = fs::temp_directory_path() / name;
  auto out = std::ofstream{path};
  out << content;
  out.close();
  return path;
}

}  // namespace

TEST(extract, string_cache) {
  auto p = fs::temp_directory_path() / "osr_test";
  auto ec = std::error_code{};
  fs::remove_all(p, ec);
  fs::create_directories(p, ec);

  extract(false, "test/map.osm", p, {});

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

  extract(false, "test/standalone-ramp.osm", p, {});

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
  auto const osm_path = write_osm("osr_negative_ids.osm", kNegativeIdsOsm);
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
      write_osm("osr_huge_positive_ids.osm", kHugePositiveIdsOsm);
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
