#include "gtest/gtest.h"

#include "osr/extract/accessibility_tags.h"

using namespace osr;

TEST(osr, parse_incline_percent) {
  EXPECT_FLOAT_EQ(8.0F, *parse_incline_percent("8%"));
  EXPECT_FLOAT_EQ(-6.5F, *parse_incline_percent("-6.5 %"));
  EXPECT_FLOAT_EQ(4.0F, *parse_incline_percent("4"));
  EXPECT_NEAR(8.749F, *parse_incline_percent("5\xC2\xB0"), 0.01F);
  EXPECT_FALSE(parse_incline_percent("up").has_value());
  EXPECT_FALSE(parse_incline_percent("down").has_value());
  EXPECT_FALSE(parse_incline_percent("").has_value());
  EXPECT_FALSE(parse_incline_percent("5%x").has_value());
}

TEST(osr, parse_width_meters) {
  EXPECT_FLOAT_EQ(1.2F, *parse_width_meters("1.2"));
  EXPECT_FLOAT_EQ(0.9F, *parse_width_meters("0.9 m"));
  EXPECT_FLOAT_EQ(1.2F, *parse_width_meters("120 cm"));
  EXPECT_FLOAT_EQ(0.85F, *parse_width_meters("850mm"));
  EXPECT_FALSE(parse_width_meters("3'").has_value());
  EXPECT_FALSE(parse_width_meters("narrow").has_value());
  EXPECT_FALSE(parse_width_meters("0").has_value());
}
