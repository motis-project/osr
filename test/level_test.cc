#ifdef _WIN32
#include "windows.h"
#endif

#include "gtest/gtest.h"

#include "osr/ways.h"

using namespace osr;

TEST(osr, level) {
  auto const lvl_0 = to_float(to_level(0.1F));
  EXPECT_EQ(0.0F, lvl_0);

  auto const lvl_4 = to_float(to_level(4.0F));
  EXPECT_EQ(4.0F, lvl_4);

  auto const lvl_minus_3 = to_float(to_level(-3.0F));
  EXPECT_EQ(-3.0F, lvl_minus_3);
}