#include "gtest/gtest.h"
#include "osr/location.h"
#include "test_util.h"
#include "osr/extract/extract.h"

namespace fs = std::filesystem;

TEST(routing_instructions, intersection_right_turn) {
  auto const from =
    osr::location{49.883991, 8.676619, osr::level_t{0.F}};
  auto const to =
      osr::location{49.884657, 8.677458, osr::level_t{0.F}};
  const auto op = extract_and_route("test/komponistenviertel-darmstadt-pbf.osm.pbf", from, to);
  ASSERT_TRUE(op.has_value());
}

TEST(routing_instructions, intersection_left_turn) {
  auto const from =
    osr::location{49.883991, 8.676619, osr::level_t{0.F}};
  auto const to =
      osr::location{49.883848, 8.675409, osr::level_t{0.F}};
  const auto op = extract_and_route("test/komponistenviertel-darmstadt-pbf.osm.pbf", from, to);
  ASSERT_TRUE(op.has_value());
}

TEST(routing_instructions, intersection_continue) {
  auto const from =
    osr::location{49.883991, 8.676619, osr::level_t{0.F}};
  auto const to =
      osr::location{49.884584, 8.675908, osr::level_t{0.F}};
  const auto op = extract_and_route("test/komponistenviertel-darmstadt-pbf.osm.pbf", from, to);
  ASSERT_TRUE(op.has_value());
}
