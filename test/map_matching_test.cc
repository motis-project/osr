#include <filesystem>
#include <vector>

#include "gmock/gmock-matchers.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "utl/to_vec.h"

#include "osr/extract/extract.h"
#include "osr/geojson.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/map_matching.h"
#include "osr/ways.h"

using namespace testing;

namespace fs = std::filesystem;

fs::path extract(fs::path const& osm_path) {
  auto const dir = fs::temp_directory_path() / "osr" / osm_path;
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, osm_path, dir, {});

  return dir;
}

MATCHER(LatLngMatches, "matches latlng") {
  const auto& [actual, expected] = arg;

  return ExplainMatchResult(AllOf(Property("lat", &geo::latlng::lat,
                                           DoubleNear(expected.lat(), 10e-6)),
                                  Property("lng", &geo::latlng::lng,
                                           DoubleNear(expected.lng(), 10e-6))),
                            actual, result_listener);
}

MATCHER(PolylineMatches, "matches polyline") {
  const auto& [actual, expected] = arg;

  return ExplainMatchResult(Pointwise(LatLngMatches(), expected), actual,
                            result_listener);
}

class map_matching_test : public Test {
protected:
  explicit map_matching_test(fs::path const& osm_path)
      : dir_{extract(osm_path)},
        w_{dir_, cista::mmap::protection::READ},
        l_{w_, dir_, cista::mmap::protection::READ} {}

  fs::path dir_;
  osr::ways w_;
  osr::lookup l_;
};

class map_matching_karlsruhe : public map_matching_test {
protected:
  map_matching_karlsruhe()
      : map_matching_test("test/karlsruhe-kirchfeld.osm.pbf") {}

  std::vector<std::vector<geo::latlng>> expected_routed_polylines_{
      {
          {49.03885227389645, 8.394025734884261},
          {49.0391489, 8.393971},
          {49.0393213, 8.3939406},
          {49.0395005, 8.3939066},
      },
      {
          {49.0395005, 8.3939066},
          {49.0397209, 8.3938697},
      },
      {
          {49.0397209, 8.3938697},
          {49.0399036, 8.3938389},
      },
      {
          {49.0399036, 8.3938389},
          {49.0401948, 8.3937867},
      },
      {
          {49.0401948, 8.3937867},
          {49.0402059, 8.3936638},
      },
      {
          {49.0402059, 8.3936638},
          {49.0402096, 8.3936308},
          {49.0402203, 8.3934966},
      },
      {
          {49.0402203, 8.3934966},
          {49.0402211, 8.3933707},
      },
      {
          {49.0402211, 8.3933707},
          {49.0402265, 8.3933543},
          {49.0402299, 8.393327},
          {49.0402291, 8.3933109},
          {49.0402248, 8.39329},
          {49.0402171, 8.3932579},
          {49.0402132, 8.3932312},
          {49.0402106, 8.3931997},
          {49.0402107, 8.393165},
      },
      {
          {49.0402107, 8.393165},
          {49.0404398, 8.3928511},
      },
      {
          {49.0404398, 8.3928511},
          {49.04044077816971, 8.392849737311106},
      },
      {
          {49.04044077816971, 8.392849737311106},
          {49.0406891, 8.3925038},
      },
      {
          {49.0406891, 8.3925038},
          {49.0408573, 8.3922695},
          {49.0412997, 8.3916955},
      },
      {
          {49.0412997, 8.3916955},
          {49.0420094, 8.3908177},
      },
      {
          {49.0420094, 8.3908177},
          {49.04222459796325, 8.390558216925339},
      },
  };
};

TEST_F(map_matching_karlsruhe, all_routed) {
  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.038863, 8.394161}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0404738, 8.3929049}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0422707, 8.3906472}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(14U, mr.path_.segments_.size());
  EXPECT_EQ(2U, mr.n_routed_);
  EXPECT_EQ(0U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 10U, 14U));

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_routed_polylines_));
}

TEST_F(map_matching_karlsruhe, all_outside_2pts) {
  // all the points are outside the osm map extract -> expect a single beeline

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.02453, 8.44947}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.01561, 8.34768}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(1U, mr.path_.segments_.size());
  EXPECT_EQ(0U, mr.n_routed_);
  EXPECT_EQ(1U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 1U));

  auto const expected_polylines = std::vector<std::vector<geo::latlng>>{
      {
          {49.02453, 8.44947},
          {49.01561, 8.34768},
      },
  };

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

TEST_F(map_matching_karlsruhe, all_outside_3pts) {
  // all the points are outside the osm map extract -> expect 2 beelines

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.02453, 8.44947}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.01561, 8.34768}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {48.99892, 8.29536}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(2U, mr.path_.segments_.size());
  EXPECT_EQ(0U, mr.n_routed_);
  EXPECT_EQ(2U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 1U, 2U));

  auto const expected_polylines = std::vector<std::vector<geo::latlng>>{
      {
          {49.02453, 8.44947},
          {49.01561, 8.34768},
      },
      {
          {49.01561, 8.34768},
          {48.99892, 8.29536},
      },
  };

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

TEST_F(map_matching_karlsruhe, start_beeline) {
  // the first point is outside the map extract
  // -> expect a beeline to the first matched point (!= input coordinates)
  // as the first segment

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.02453, 8.44947}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.038863, 8.394161}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0404738, 8.3929049}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0422707, 8.3906472}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(15U, mr.path_.segments_.size());
  EXPECT_EQ(2U, mr.n_routed_);
  EXPECT_EQ(1U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 1U, 11U, 15U));

  auto expected_polylines = std::vector<std::vector<geo::latlng>>{
      {
          {49.02453, 8.44947},
          {49.03885227389645, 8.394025734884261},
      },
  };
  expected_polylines.insert(expected_polylines.end(),
                            expected_routed_polylines_.begin(),
                            expected_routed_polylines_.end());

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

TEST_F(map_matching_karlsruhe, end_beeline) {
  // the last point is outside the map extract
  // -> expect a beeline from the last matched point (!= input coordinates)
  // as the last segment

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.038863, 8.394161}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0404738, 8.3929049}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0422707, 8.3906472}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.01561, 8.34768}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(15U, mr.path_.segments_.size());
  EXPECT_EQ(2U, mr.n_routed_);
  EXPECT_EQ(1U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 10U, 14U, 15U));

  auto expected_polylines = expected_routed_polylines_;
  expected_polylines.emplace_back(std::vector<geo::latlng>{
      {49.04222459796325, 8.390558216925339},
      {49.01561, 8.34768},
  });

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

TEST_F(map_matching_karlsruhe, start_and_end_beeline) {
  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.02453, 8.44947}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.038863, 8.394161}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0404738, 8.3929049}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0422707, 8.3906472}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.01561, 8.34768}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(16U, mr.path_.segments_.size());
  EXPECT_EQ(2U, mr.n_routed_);
  EXPECT_EQ(2U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 1U, 11U, 15U, 16U));

  auto expected_polylines = std::vector<std::vector<geo::latlng>>{
      {
          {49.02453, 8.44947},
          {49.03885227389645, 8.394025734884261},
      },
  };
  expected_polylines.insert(expected_polylines.end(),
                            expected_routed_polylines_.begin(),
                            expected_routed_polylines_.end());
  expected_polylines.emplace_back(std::vector<geo::latlng>{
      {49.04222459796325, 8.390558216925339},
      {49.01561, 8.34768},
  });

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

TEST_F(map_matching_karlsruhe, beeline_in_the_middle) {
  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kBus, osr::bus::parameters{},
      std::vector{
          osr::location{.pos_ = {49.038863, 8.394161}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0422707, 8.3906472}, .lvl_ = osr::kNoLevel},
          // the following point is outside the map extract
          osr::location{.pos_ = {49.04717, 8.37875}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.053476, 8.399949}, .lvl_ = osr::kNoLevel},
          osr::location{.pos_ = {49.0591517, 8.3998284}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(33U, mr.path_.segments_.size());
  EXPECT_EQ(2U, mr.n_routed_);
  EXPECT_EQ(2U, mr.n_beelined_);
  EXPECT_THAT(mr.segment_offsets_, ElementsAre(0U, 13U, 14U, 15U, 33U));

  auto expected_polylines = std::vector<std::vector<geo::latlng>>{
      {
          {49.0388522738965, 8.39402573488426},
          {49.0391489, 8.393971},
          {49.0393213, 8.3939406},
          {49.0395005, 8.3939066},
      },
      {
          {49.0395005, 8.3939066},
          {49.0397209, 8.3938697},
      },
      {
          {49.0397209, 8.3938697},
          {49.0399036, 8.3938389},
      },
      {
          {49.0399036, 8.3938389},
          {49.0401948, 8.3937867},
      },
      {
          {49.0401948, 8.3937867},
          {49.0402059, 8.3936638},
      },
      {
          {49.0402059, 8.3936638},
          {49.0402096, 8.3936308},
          {49.0402203, 8.3934966},
      },
      {
          {49.0402203, 8.3934966},
          {49.0402211, 8.3933707},
      },
      {
          {49.0402211, 8.3933707},
          {49.0402265, 8.3933543},
          {49.0402299, 8.393327},
          {49.0402291, 8.3933109},
          {49.0402248, 8.39329},
          {49.0402171, 8.3932579},
          {49.0402132, 8.3932312},
          {49.0402106, 8.3931997},
          {49.0402107, 8.393165},
      },
      {
          {49.0402107, 8.393165},
          {49.0404398, 8.3928511},
      },
      {
          {49.0404398, 8.3928511},
          {49.0406891, 8.3925038},
      },
      {
          {49.0406891, 8.3925038},
          {49.0408573, 8.3922695},
          {49.0412997, 8.3916955},
      },
      {
          {49.0412997, 8.3916955},
          {49.0420094, 8.3908177},
      },
      {
          {49.0420094, 8.3908177},
          {49.0422245979632, 8.39055821692534},
      },
      {
          {49.0422245979632, 8.39055821692534},
          {49.04717, 8.37875},
      },
      {
          {49.04717, 8.37875},
          {49.0535068318588, 8.3999442369763},
      },
      {
          {49.0535068318588, 8.3999442369763},
          {49.0535074, 8.3999528},
      },
      {
          {49.0535074, 8.3999528},
          {49.0535167, 8.4001217},
          {49.0535269, 8.40022},
          {49.0535455, 8.400315},
          {49.0535689, 8.4003775},
      },
      {
          {49.0535689, 8.4003775},
          {49.0536166, 8.4004593},
          {49.0536853, 8.4005369},
      },
      {
          {49.0536853, 8.4005369},
          {49.0537789, 8.4005899},
          {49.0538421, 8.4006087},
          {49.0539098, 8.4006041},
      },
      {
          {49.0539098, 8.4006041},
          {49.0539592, 8.400599},
      },
      {
          {49.0539592, 8.400599},
          {49.0543124, 8.4005484},
      },
      {
          {49.0543124, 8.4005484},
          {49.0546627, 8.4004982},
      },
      {
          {49.0546627, 8.4004982},
          {49.0554416, 8.4003874},
      },
      {
          {49.0554416, 8.4003874},
          {49.0558504, 8.4003315},
      },
      {
          {49.0558504, 8.4003315},
          {49.0560508, 8.4003186},
          {49.0562035, 8.4002836},
      },
      {
          {49.0562035, 8.4002836},
          {49.0562257, 8.4002786},
      },
      {
          {49.0562257, 8.4002786},
          {49.0563843, 8.4002423},
      },
      {
          {49.0563843, 8.4002423},
          {49.0566375, 8.4002063},
      },
      {
          {49.0566375, 8.4002063},
          {49.0569099, 8.4001655},
      },
      {
          {49.0569099, 8.4001655},
          {49.0571721, 8.4001276},
      },
      {
          {49.0571721, 8.4001276},
          {49.0582586, 8.3999713},
      },
      {
          {49.0582586, 8.3999713},
          {49.0586434, 8.3999179},
      },
      {
          {49.0586434, 8.3999179},
          {49.05915189714306, 8.399831089367737},
      },
  };

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}

class map_matching_london : public map_matching_test {
protected:
  map_matching_london()
      : map_matching_test("test/london-northern-line.osm.pbf") {}
};

TEST_F(map_matching_london, northern_line) {
  // This tests a case where two consecutive points are matched to the same osr
  // way and there are no osr nodes between the two matched points, thus
  // requiring a direct edge between the two additional nodes.
  // Otherwise, a giant detour is generated (over the River Thames).

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kRailway, osr::railway::parameters{},
      std::vector{
          // Balham
          osr::location{.pos_ = {51.443429, -0.152782}, .lvl_ = osr::kNoLevel},
          // Tooting Bec
          osr::location{.pos_ = {51.435750, -0.159402}, .lvl_ = osr::kNoLevel},
          // Tooting Broadway
          osr::location{.pos_ = {51.427805, -0.168136}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":689129018,"cost":326,"distance":980},"geometry":{"type":"LineString","coordinates":[[-1.5277022640749896E-1,5.14434248276116E1],[-1.528186E-1,5.14433718E1],[-1.5296E-1,5.1443199E1],[-1.531195E-1,5.14430038E1],[-1.533013E-1,5.14427826E1],[-1.55187E-1,5.14404878E1],[-1.559461E-1,5.143928E1],[-1.566684E-1,5.14378739E1],[-1.572287E-1,5.14372959E1],[-1.583335E-1,5.14363843E1],[-1.589128E-1,5.14360081E1],[-1.590407E-1,5.14359333E1],[-1.592987E-1,5.14357824E1],[-1.5938785722441887E-1,5.143573876914223E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":689129018,"cost":14,"distance":42},"geometry":{"type":"LineString","coordinates":[[-1.5938785722441887E-1,5.143573876914223E1],[-1.598208E-1,5.14355269E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":689129018,"cost":350,"distance":1052},"geometry":{"type":"LineString","coordinates":[[-1.598208E-1,5.14355269E1],[-1.604979E-1,5.14352418E1],[-1.612097E-1,5.14348656E1],[-1.615151E-1,5.14346777E1],[-1.621663E-1,5.14342415E1],[-1.627351E-1,5.14337909E1],[-1.634107E-1,5.14331139E1],[-1.635821E-1,5.14328656E1],[-1.637225E-1,5.14325488E1],[-1.639167E-1,5.14320762E1],[-1.640189E-1,5.14318777E1],[-1.647738E-1,5.14310947E1],[-1.666569E-1,5.14292455E1],[-1.67733E-1,5.1428231E1],[-1.67951E-1,5.14279696E1],[-1.681201E-1,5.14278007E1],[-1.6812182130106906E-1,5.142779909364883E1]]}}]})",
      osr::to_featurecollection(w_, mr.path_, false));
}

class map_matching_neuss : public map_matching_test {
protected:
  map_matching_neuss() : map_matching_test("test/neuss-s11.osm.pbf") {}
};

TEST_F(map_matching_neuss, s11) {
  // This tests a combination of matching distance penalty + turn angle penalty
  // In this case the stop coordinates are pretty exact and the tracks closest
  // to these input coordinates should be used.
  // Shorter routes that we don't want include:
  // - A route using zig-zag crossover tracks at Neuss Hbf
  // - A route using one of the more southern/eastern tracks, which would be
  //   shorter, but use the wrong track at Neuss Hbf

  auto const mr = osr::map_match(
      w_, l_, osr::search_profile::kRailway, osr::railway::parameters{},
      std::vector{
          // Neuss Am Kaiser
          osr::location{.pos_ = {51.220429, 6.697381}, .lvl_ = osr::kNoLevel},
          // Neuss Hauptbahnhof
          osr::location{.pos_ = {51.205150, 6.685020}, .lvl_ = osr::kNoLevel},
          // Neuss Süd S
          osr::location{.pos_ = {51.185667, 6.690437}, .lvl_ = osr::kNoLevel},
      });

  EXPECT_EQ(
      R"({"type":"FeatureCollection","metadata":{},"features":[{"type":"Feature","properties":{"level":0E0,"osm_way_id":19634486,"cost":32,"distance":97},"geometry":{"type":"LineString","coordinates":[[6.697383843743055E0,5.12204331175352E1],[6.6971558E0,5.12204949E1],[6.6968037E0,5.12205721E1],[6.6964531E0,5.12206343E1],[6.6960707E0,5.1220685E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":23238429,"cost":8,"distance":25},"geometry":{"type":"LineString","coordinates":[[6.6960707E0,5.1220685E1],[6.695893E0,5.12207035E1],[6.6957089E0,5.12207184E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":23237222,"cost":65,"distance":196},"geometry":{"type":"LineString","coordinates":[[6.6957089E0,5.12207184E1],[6.6953843E0,5.12207331E1],[6.6951269E0,5.12207371E1],[6.6948075E0,5.12207316E1],[6.6945134E0,5.12207185E1],[6.6941953E0,5.12206942E1],[6.6938652E0,5.12206549E1],[6.6935814E0,5.12206116E1],[6.69325E0,5.12205488E1],[6.6929485E0,5.1220481E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":272645834,"cost":123,"distance":371},"geometry":{"type":"LineString","coordinates":[[6.6929485E0,5.1220481E1],[6.6925943E0,5.12203933E1],[6.6921272E0,5.12202691E1],[6.6912969E0,5.12200421E1],[6.6905936E0,5.12198467E1],[6.6895996E0,5.12195576E1],[6.6892685E0,5.12194462E1],[6.6889873E0,5.12193407E1],[6.6887076E0,5.12192237E1],[6.6884585E0,5.12191148E1],[6.6882035E0,5.12189887E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":27721183,"cost":6,"distance":19},"geometry":{"type":"LineString","coordinates":[[6.6882035E0,5.12189887E1],[6.6879959E0,5.12188758E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":26436277,"cost":122,"distance":366},"geometry":{"type":"LineString","coordinates":[[6.6879959E0,5.12188758E1],[6.6877619E0,5.1218728E1],[6.6875957E0,5.12186071E1],[6.6874151E0,5.12184584E1],[6.6872614E0,5.12183124E1],[6.687091E0,5.12181184E1],[6.6869336E0,5.12179052E1],[6.6867833E0,5.12176387E1],[6.6867183E0,5.12174869E1],[6.6866531E0,5.12172868E1],[6.6866125E0,5.12170508E1],[6.6865956E0,5.12168659E1],[6.6866007E0,5.12166983E1],[6.6866527E0,5.12162774E1],[6.6867032E0,5.12160432E1],[6.6867514E0,5.1215836E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":28061570,"cost":35,"distance":105},"geometry":{"type":"LineString","coordinates":[[6.6867514E0,5.1215836E1],[6.6869507E0,5.12149029E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":28061569,"cost":54,"distance":162},"geometry":{"type":"LineString","coordinates":[[6.6869507E0,5.12149029E1],[6.6872709E0,5.12134617E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":28061571,"cost":21,"distance":64},"geometry":{"type":"LineString","coordinates":[[6.6872709E0,5.12134617E1],[6.6874021E0,5.12128901E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":149165317,"cost":198,"distance":596},"geometry":{"type":"LineString","coordinates":[[6.6874021E0,5.12128901E1],[6.6874541E0,5.12126719E1],[6.6874942E0,5.12124381E1],[6.6875572E0,5.12119832E1],[6.6877022E0,5.12105441E1],[6.6877491E0,5.12101251E1],[6.6877689E0,5.12098781E1],[6.6877711E0,5.12096458E1],[6.6877665E0,5.12095078E1],[6.6877495E0,5.12093289E1],[6.6877278E0,5.12091626E1],[6.6876999E0,5.12089978E1],[6.687647E0,5.12087852E1],[6.6876099E0,5.12086544E1],[6.6875427E0,5.12084643E1],[6.6873711E0,5.12080636E1],[6.6872526E0,5.1207792E1],[6.6871508E0,5.12075834E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":322512528,"cost":42,"distance":126},"geometry":{"type":"LineString","coordinates":[[6.6871508E0,5.12075834E1],[6.6870314E0,5.12073683E1],[6.6868197E0,5.12070106E1],[6.6866639E0,5.12067864E1],[6.6865369E0,5.12066149E1],[6.6864731E0,5.12065374E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":322512528,"cost":8,"distance":26},"geometry":{"type":"LineString","coordinates":[[6.6864731E0,5.12065374E1],[6.6863011E0,5.12063285E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":119787806,"cost":15,"distance":45},"geometry":{"type":"LineString","coordinates":[[6.6863011E0,5.12063285E1],[6.6859782E0,5.12059796E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":188544625,"cost":38,"distance":114},"geometry":{"type":"LineString","coordinates":[[6.6859782E0,5.12059796E1],[6.6859423E0,5.12059423E1],[6.6857513E0,5.12057591E1],[6.6855555E0,5.12055823E1],[6.6852605E0,5.12053336E1],[6.685021399201864E0,5.120514928749347E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":188544625,"cost":26,"distance":78},"geometry":{"type":"LineString","coordinates":[[6.685021399201864E0,5.120514928749347E1],[6.6849956E0,5.12051294E1],[6.68472E0,5.12049337E1],[6.6845759E0,5.12048425E1],[6.684325E0,5.12046909E1],[6.6842478E0,5.12046464E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":322512529,"cost":5,"distance":17},"geometry":{"type":"LineString","coordinates":[[6.6842478E0,5.12046464E1],[6.684071E0,5.12045414E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":119787807,"cost":29,"distance":89},"geometry":{"type":"LineString","coordinates":[[6.684071E0,5.12045414E1],[6.6838198E0,5.12043927E1],[6.6836093E0,5.120426E1],[6.6831722E0,5.12039691E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":20890432,"cost":5,"distance":17},"geometry":{"type":"LineString","coordinates":[[6.6831722E0,5.12039691E1],[6.6830298E0,5.12038767E1],[6.6830068E0,5.12038614E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":18602208,"cost":2,"distance":6},"geometry":{"type":"LineString","coordinates":[[6.6830068E0,5.12038614E1],[6.6829488E0,5.12038162E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":461315578,"cost":10,"distance":31},"geometry":{"type":"LineString","coordinates":[[6.6829488E0,5.12038162E1],[6.6826558E0,5.12036054E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":89329143,"cost":70,"distance":212},"geometry":{"type":"LineString","coordinates":[[6.6826558E0,5.12036054E1],[6.6822372E0,5.12032933E1],[6.6820278E0,5.12031389E1],[6.6818192E0,5.12029943E1],[6.6815297E0,5.12027993E1],[6.6812422E0,5.12026261E1],[6.6811182E0,5.1202558E1],[6.6809846E0,5.12024901E1],[6.6808304E0,5.12024191E1],[6.6806781E0,5.1202356E1],[6.6804883E0,5.12022891E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1491959883,"cost":8,"distance":24},"geometry":{"type":"LineString","coordinates":[[6.6804883E0,5.12022891E1],[6.6803347E0,5.1202243E1],[6.6801775E0,5.12022019E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":28338821,"cost":13,"distance":41},"geometry":{"type":"LineString","coordinates":[[6.6801775E0,5.12022019E1],[6.6799949E0,5.12021612E1],[6.6798088E0,5.12021272E1],[6.67962E0,5.12021E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":28338822,"cost":81,"distance":244},"geometry":{"type":"LineString","coordinates":[[6.67962E0,5.12021E1],[6.6794181E0,5.12020807E1],[6.6792433E0,5.120207E1],[6.6790098E0,5.12020635E1],[6.6788138E0,5.12020653E1],[6.678086E0,5.12020875E1],[6.6778981E0,5.1202092E1],[6.6777127E0,5.12020916E1],[6.6775219E0,5.12020883E1],[6.6773122E0,5.12020728E1],[6.677163E0,5.12020584E1],[6.6770119E0,5.12020383E1],[6.6768683E0,5.12020152E1],[6.6767105E0,5.12019855E1],[6.6764673E0,5.12019279E1],[6.6761842E0,5.1201855E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":146635070,"cost":24,"distance":73},"geometry":{"type":"LineString","coordinates":[[6.6761842E0,5.1201855E1],[6.6757454E0,5.12016955E1],[6.6754977E0,5.12015925E1],[6.6752971E0,5.12015024E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":88719318,"cost":7,"distance":23},"geometry":{"type":"LineString","coordinates":[[6.6752971E0,5.12015024E1],[6.6750376E0,5.12013686E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":88719325,"cost":11,"distance":33},"geometry":{"type":"LineString","coordinates":[[6.6750376E0,5.12013686E1],[6.6748727E0,5.12012714E1],[6.6747792E0,5.12012115E1],[6.6747031E0,5.12011606E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":88719315,"cost":22,"distance":68},"geometry":{"type":"LineString","coordinates":[[6.6747031E0,5.12011606E1],[6.6746567E0,5.12011295E1],[6.6745144E0,5.12010209E1],[6.6744094E0,5.1200935E1],[6.6742781E0,5.12008101E1],[6.6741384E0,5.120066E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":88719340,"cost":15,"distance":46},"geometry":{"type":"LineString","coordinates":[[6.6741384E0,5.120066E1],[6.6739936E0,5.12004595E1],[6.6738872E0,5.12002764E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":88719312,"cost":56,"distance":169},"geometry":{"type":"LineString","coordinates":[[6.6738872E0,5.12002764E1],[6.6738141E0,5.12001346E1],[6.6737424E0,5.11999708E1],[6.6736746E0,5.11996969E1],[6.6736432E0,5.11995183E1],[6.6736427E0,5.11993042E1],[6.673663E0,5.11990977E1],[6.6736976E0,5.11989632E1],[6.6737577E0,5.11987824E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":398580306,"cost":19,"distance":57},"geometry":{"type":"LineString","coordinates":[[6.6737577E0,5.11987824E1],[6.6738025E0,5.11986742E1],[6.6739066E0,5.11984718E1],[6.674022E0,5.11982989E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87365392,"cost":10,"distance":32},"geometry":{"type":"LineString","coordinates":[[6.674022E0,5.11982989E1],[6.6741496E0,5.11981261E1],[6.6742224E0,5.11980393E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":91877473,"cost":209,"distance":628},"geometry":{"type":"LineString","coordinates":[[6.6742224E0,5.11980393E1],[6.6743048E0,5.11979422E1],[6.6744933E0,5.11977505E1],[6.6747524E0,5.11975216E1],[6.6749158E0,5.11973931E1],[6.6751173E0,5.11972499E1],[6.675487E0,5.11970089E1],[6.6762104E0,5.11965611E1],[6.6769951E0,5.11960755E1],[6.6778791E0,5.11955268E1],[6.6782121E0,5.11953173E1],[6.6784303E0,5.11951722E1],[6.67864E0,5.11950159E1],[6.6788515E0,5.11948245E1],[6.6790573E0,5.11946077E1],[6.6791678E0,5.1194467E1],[6.6792551E0,5.11943405E1],[6.6794435E0,5.11940083E1],[6.6795748E0,5.11937792E1],[6.679658E0,5.11936263E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87365402,"cost":49,"distance":149},"geometry":{"type":"LineString","coordinates":[[6.679658E0,5.11936263E1],[6.679701E0,5.11935522E1],[6.6798902E0,5.11932085E1],[6.6800686E0,5.11929033E1],[6.6802319E0,5.11926416E1],[6.6804097E0,5.11923714E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87367394,"cost":14,"distance":43},"geometry":{"type":"LineString","coordinates":[[6.6804097E0,5.11923714E1],[6.6804916E0,5.11922429E1],[6.6806495E0,5.11920156E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87365403,"cost":8,"distance":26},"geometry":{"type":"LineString","coordinates":[[6.6806495E0,5.11920156E1],[6.6808094E0,5.11918056E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87365406,"cost":12,"distance":37},"geometry":{"type":"LineString","coordinates":[[6.6808094E0,5.11918056E1],[6.6810551E0,5.11915135E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87365385,"cost":51,"distance":154},"geometry":{"type":"LineString","coordinates":[[6.6810551E0,5.11915135E1],[6.6812477E0,5.11913183E1],[6.6813573E0,5.11912072E1],[6.6816433E0,5.1190935E1],[6.6818326E0,5.11907864E1],[6.6819677E0,5.11906817E1],[6.682346E0,5.11903915E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":231371894,"cost":62,"distance":186},"geometry":{"type":"LineString","coordinates":[[6.682346E0,5.11903915E1],[6.6826927E0,5.11901566E1],[6.683082E0,5.11899134E1],[6.6833545E0,5.11897499E1],[6.6834889E0,5.11896693E1],[6.6842664E0,5.11892249E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":20886379,"cost":35,"distance":107},"geometry":{"type":"LineString","coordinates":[[6.6842664E0,5.11892249E1],[6.6853978E0,5.11885801E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":87367388,"cost":2,"distance":6},"geometry":{"type":"LineString","coordinates":[[6.6853978E0,5.11885801E1],[6.6854614E0,5.11885429E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1354256981,"cost":4,"distance":13},"geometry":{"type":"LineString","coordinates":[[6.6854614E0,5.11885429E1],[6.6855984E0,5.11884629E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":1354256978,"cost":2,"distance":7},"geometry":{"type":"LineString","coordinates":[[6.6855984E0,5.11884629E1],[6.6856749E0,5.11884182E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":232568688,"cost":15,"distance":46},"geometry":{"type":"LineString","coordinates":[[6.6856749E0,5.11884182E1],[6.6861626E0,5.11881359E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":127961202,"cost":94,"distance":284},"geometry":{"type":"LineString","coordinates":[[6.6861626E0,5.11881359E1],[6.6876218E0,5.11872934E1],[6.6883375E0,5.11868802E1],[6.68917E0,5.11864071E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":127961205,"cost":13,"distance":39},"geometry":{"type":"LineString","coordinates":[[6.68917E0,5.11864071E1],[6.6895816E0,5.11861645E1]]}},{"type":"Feature","properties":{"level":1E0,"osm_way_id":127961215,"cost":25,"distance":77},"geometry":{"type":"LineString","coordinates":[[6.6895816E0,5.11861645E1],[6.6903949E0,5.11856914E1]]}},{"type":"Feature","properties":{"level":0E0,"osm_way_id":127961215,"cost":1,"distance":4},"geometry":{"type":"LineString","coordinates":[[6.6903949E0,5.11856914E1],[6.6904370240837645E0,5.118566701634578E1]]}}]})",
      osr::to_featurecollection(w_, mr.path_, false));
}
