#include <filesystem>
#include <fstream>
#include <string_view>

#include "cista/mmap.h"

#include "gtest/gtest.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

namespace {

constexpr auto const kConditionalRestrictionsOsm =
    R"osm(<osm version="0.6" generator="test">
  <node id="1" lat="48.0000" lon="9.0000"/>
  <node id="2" lat="48.0000" lon="9.0010"/>
  <node id="3" lat="48.0000" lon="9.0020"/>
  <node id="4" lat="47.9990" lon="8.9990"/>
  <node id="5" lat="47.9990" lon="9.0030"/>
  <node id="6" lat="48.0010" lon="9.0030"/>
  <node id="7" lat="48.0010" lon="8.9990"/>
  <way id="10">
    <nd ref="1"/>
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="access:conditional" v="no @ (Mo-Fr 06:00-10:00)"/>
    <tag k="maxheight:hgv:conditional" v="3.8 @ (weight &gt; 7.5)"/>
    <tag k="hazmat:water:conditional" v="no @ (trailer)"/>
    <tag k="vehicle:conditional" v="no @ (Mo-Fr 06:00-10:00)"/>
    <tag k="maxstay:conditional" v="30 @ (Mo-Fr 06:00-10:00)"/>
    <tag k="opening_hours:conditional" v="closed @ (Mo-Fr 06:00-10:00)"/>
  </way>
  <way id="20">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
    <tag k="access:conditional" v="no @ (PH)"/>
  </way>
  <way id="100">
    <nd ref="4"/>
    <nd ref="5"/>
    <nd ref="6"/>
    <nd ref="7"/>
    <nd ref="4"/>
  </way>
  <relation id="1000">
    <member type="way" ref="100" role="outer"/>
    <tag k="type" v="boundary"/>
    <tag k="boundary" v="administrative"/>
    <tag k="admin_level" v="4"/>
    <tag k="timezone" v="Europe/Berlin"/>
  </relation>
</osm>)osm";

constexpr auto const kHgvAccessOsm = R"osm(<osm version="0.6" generator="test">
  <node id="1" lat="48.0000" lon="9.0000"/>
  <node id="2" lat="48.0000" lon="9.0010"/>
  <node id="3" lat="48.0000" lon="9.0020"/>
  <node id="4" lat="48.0000" lon="9.0030"/>
  <node id="5" lat="48.0000" lon="9.0040"/>
  <node id="6" lat="48.0000" lon="9.0050"/>
  <way id="10">
    <nd ref="1"/>
    <nd ref="2"/>
    <tag k="highway" v="primary"/>
    <tag k="hgv" v="yes"/>
  </way>
  <way id="20">
    <nd ref="2"/>
    <nd ref="3"/>
    <tag k="highway" v="primary"/>
    <tag k="hgv" v="designated"/>
  </way>
  <way id="30">
    <nd ref="3"/>
    <nd ref="4"/>
    <tag k="highway" v="primary"/>
    <tag k="hgv" v="delivery"/>
  </way>
  <way id="40">
    <nd ref="4"/>
    <nd ref="5"/>
    <tag k="highway" v="primary"/>
    <tag k="hgv" v="destination"/>
  </way>
  <way id="50">
    <nd ref="5"/>
    <nd ref="6"/>
    <tag k="highway" v="primary"/>
    <tag k="hgv" v="no"/>
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

fs::path prepare_extract_dir(std::string_view const name) {
  auto const dir = fs::temp_directory_path() / name;
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  return dir;
}

}  // namespace

TEST(conditional_extract, stores_parseable_conditional_restrictions) {
  auto const osm_path =
      write_osm("osr_conditionals.osm", kConditionalRestrictionsOsm);
  auto const dir = prepare_extract_dir("osr_conditionals_dir");

  osr::extract(false, osm_path.generic_string(), dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto const way = w.find_way(osr::osm_way_idx_t{10});
  ASSERT_TRUE(way.has_value());

  auto const props = w.r_->way_properties_[*way];
  ASSERT_TRUE(props.has_conditionals());
  auto const* conditionals = w.r_->get_conditional_restrictions(*way);
  ASSERT_NE(nullptr, conditionals);

  ASSERT_EQ(2U, conditionals->access_.size());
  ASSERT_EQ(1U, conditionals->numeric_.size());
  auto const& access = w.r_->conditional_access_[conditionals->access_.begin_];
  EXPECT_EQ(osr::access_value::kNo, access.value_);
  EXPECT_EQ(osr::conditional_restriction_field::kAccess, access.field_);
  auto const& hazmat =
      w.r_->conditional_access_[conditionals->access_.begin_ + 1U];
  EXPECT_EQ(osr::conditional_restriction_field::kHazmatWater, hazmat.field_);
  EXPECT_EQ(osr::access_value::kNo, hazmat.value_);

  auto const& maxheight =
      w.r_->conditional_numeric_[conditionals->numeric_.begin_];
  EXPECT_EQ(osr::conditional_restriction_field::kMaxHeight, maxheight.field_);
  EXPECT_EQ(osr::conditional_transport_mode::kHgv, maxheight.mode_);
  EXPECT_EQ(osr::conditional_numeric_unit::kCentimeter, maxheight.value_.unit_);
  EXPECT_EQ(380U, maxheight.value_.value_);

  ASSERT_EQ(3U, w.r_->conditional_condition_sets_.size());
  ASSERT_EQ(1U, w.r_->opening_hours_weekday_ranges_.size());
  EXPECT_EQ(1U, w.r_->opening_hours_weekday_ranges_[0].from_);
  EXPECT_EQ(5U, w.r_->opening_hours_weekday_ranges_[0].to_);
  ASSERT_EQ(1U, w.r_->opening_hours_time_spans_.size());
  EXPECT_EQ(6U * 60U, w.r_->opening_hours_time_spans_[0].from_minutes_);
  EXPECT_EQ(10U * 60U, w.r_->opening_hours_time_spans_[0].to_minutes_);
}

TEST(conditional_extract, writes_conditionals_to_geojson_debug_output) {
  auto const osm_path =
      write_osm("osr_conditionals_geojson.osm", kConditionalRestrictionsOsm);
  auto const dir = prepare_extract_dir("osr_conditionals_geojson_dir");

  osr::extract(false, osm_path.generic_string(), dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto const way = w.find_way(osr::osm_way_idx_t{10});
  ASSERT_TRUE(way.has_value());

  auto writer = osr::geojson_writer{w};
  writer.write_way(*way);
  auto const json = writer.string();

  EXPECT_NE(std::string::npos, json.find("\"has_conditionals\":true"));
  EXPECT_NE(std::string::npos, json.find("\"timezone\":\"Europe/Berlin\""));
  EXPECT_NE(std::string::npos,
            json.find("access:conditional = no @ (Mo-Fr 06:00-10:00)"));
  EXPECT_NE(std::string::npos,
            json.find("maxheight:hgv:conditional = 3.8 m @ (weight > 7.5 t)"));
  EXPECT_NE(std::string::npos,
            json.find("hazmat:water:conditional = no @ (trailer)"));
  EXPECT_EQ(std::string::npos, json.find("vehicle:conditional"));
  EXPECT_EQ(std::string::npos, json.find("maxstay:conditional"));
  EXPECT_EQ(std::string::npos, json.find("opening_hours:conditional"));
}

TEST(conditional_extract, ignores_unsupported_conditionals) {
  auto const osm_path = write_osm("osr_conditionals_unsupported.osm",
                                  kConditionalRestrictionsOsm);
  auto const dir = prepare_extract_dir("osr_conditionals_unsupported_dir");

  osr::extract(false, osm_path.generic_string(), dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto const way = w.find_way(osr::osm_way_idx_t{20});
  ASSERT_TRUE(way.has_value());

  EXPECT_FALSE(w.r_->way_properties_[*way].has_conditionals());
  EXPECT_EQ(nullptr, w.r_->get_conditional_restrictions(*way));
}

TEST(conditional_extract, stores_hgv_access_values_separately) {
  auto const osm_path = write_osm("osr_hgv_access.osm", kHgvAccessOsm);
  auto const dir = prepare_extract_dir("osr_hgv_access_dir");

  osr::extract(false, osm_path.generic_string(), dir, {});

  auto w = osr::ways{dir, cista::mmap::protection::READ};
  auto const get_hgv_access = [&](osr::osm_way_idx_t const osm_way) {
    auto const way = w.find_way(osm_way);
    if (!way.has_value()) {
      return osr::access_value::kUnknown;
    }
    auto const* info = w.r_->get_hgv_info(*way);
    if (info == nullptr || !info->has(osr::hgv_info_field::kAccessFwd)) {
      return osr::access_value::kUnknown;
    }
    return info->hgv_access_fwd();
  };

  EXPECT_EQ(osr::access_value::kYes, get_hgv_access(osr::osm_way_idx_t{10}));
  EXPECT_EQ(osr::access_value::kDesignated,
            get_hgv_access(osr::osm_way_idx_t{20}));
  EXPECT_EQ(osr::access_value::kDelivery,
            get_hgv_access(osr::osm_way_idx_t{30}));
  EXPECT_EQ(osr::access_value::kDestination,
            get_hgv_access(osr::osm_way_idx_t{40}));
  EXPECT_EQ(osr::access_value::kNo, get_hgv_access(osr::osm_way_idx_t{50}));
}
