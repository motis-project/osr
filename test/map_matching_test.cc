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
          osr::location{.pos_ = {49.058654, 8.399863}, .lvl_ = osr::kNoLevel},
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
          {49.0586578437431, 8.39991543416418},
      },
  };

  auto const actual_polylines = utl::to_vec(
      mr.path_.segments_, [](auto const& seg) { return seg.polyline_; });

  EXPECT_THAT(actual_polylines,
              Pointwise(PolylineMatches(), expected_polylines));
}
