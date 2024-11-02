#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include "osr/ways.h"

using namespace osr;

TEST(osr, level) {
  auto const lvl_0 = level_t{0.1F}.to_float();
  EXPECT_EQ(0.0F, lvl_0);

  auto const lvl_neg4 = level_t{-4.0F}.to_float();
  EXPECT_EQ(-4.0F, lvl_neg4);

  auto const lvl_4 = level_t{4.0F}.to_float();
  EXPECT_EQ(4.0F, lvl_4);

  auto const lvl_minus_3 = level_t{-3.0F}.to_float();
  EXPECT_EQ(-3.0F, lvl_minus_3);
}