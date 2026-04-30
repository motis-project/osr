#include "gtest/gtest.h"
#include <vector>
#include <numeric>
#include "osr/util/sliding_window.h"

#include "utl/enumerate.h"

TEST(sliding_window, basic1) {
  std::vector<int> v(5);
  std::iota(v.begin(), v.end(), 0);

  // Radius 1: width 3.
  // Windows centered at 0, 1, 2, 3, 4.
  // [null, 0, 1]
  // [0, 1, 2]
  // [1, 2, 3]
  // [2, 3, 4]
  // [3, 4, null]

  auto count = 0;

  const auto window_range = osr::sliding_window<1>(v);
  for (const auto& [i, w] : utl::enumerate(window_range)) {
    if (i == 0U) {
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(w[-1], v.end());
      EXPECT_EQ(*w.focus(), 0);
      EXPECT_EQ(*w[1], 1);
      EXPECT_EQ(w[2], v.end());
    } else if (i == 1U) {
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(*w[-1], 0);
      EXPECT_EQ(*w.focus(), 1);
      EXPECT_EQ(*w[1], 2);
      EXPECT_EQ(w[2], v.end());
    } else if (i == 2U) {
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(*w[-1], 1);
      EXPECT_EQ(*w.focus(), 2);
      EXPECT_EQ(*w[1], 3);
      EXPECT_EQ(w[2], v.end());
    } else if (i == 3U) {
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(*w[-1], 2);
      EXPECT_EQ(*w.focus(), 3);
      EXPECT_EQ(*w[1], 4);
      EXPECT_EQ(w[2], v.end());
    } else if (i == 4U) {
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(*w[-1], 3);
      EXPECT_EQ(*w.focus(), 4);
      EXPECT_EQ(w[1], v.end());
      EXPECT_EQ(w[2], v.end());
    }
    count++;
  }
  EXPECT_EQ(count, 5);
}

TEST(sliding_window, empty) {
  std::vector<int> v;

  const auto window_range = osr::sliding_window<1>(v);
  EXPECT_EQ(window_range.begin(), window_range.end());
}

TEST(sliding_window, singleton) {
  std::vector<int> v = { 1 };
  const auto window_range = osr::sliding_window<1>(v);
  auto iter = window_range.begin();

  EXPECT_NE(iter, window_range.end());
  const auto w = *iter;

  // Radius 1: width 3.
  // Expected [null, 1, null]
  EXPECT_EQ(w[-2], v.end());
  EXPECT_EQ(w[-1], v.end());
  EXPECT_EQ(*w.focus(), 1);
  EXPECT_EQ(w[1], v.end());
  EXPECT_EQ(w[2], v.end());

  ++iter;
  EXPECT_EQ(iter, window_range.end());
}

TEST(sliding_window, basic2) {
  std::vector<int> v(5);
  std::iota(v.begin(), v.end(), 0);
  std::reverse(v.begin(), v.end());
  // Radius 2: width 5.
  // Windows centered at 4, 3, 2, 1, 0.
  // [null, null, 4, 3, 2]
  // [null, 4, 3, 2, 1]
  // [4, 3, 2, 1, 0]
  // [3, 2, 1, 0, null]
  // [2, 1, 0, null, null]

  auto count = 0;

  const auto window_range = osr::sliding_window<2>(v);
  for (const auto& [i, w] : utl::enumerate(window_range)) {
    if (i == 0U) {
      EXPECT_EQ(w[-3], v.end());
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(w[-1], v.end());
      EXPECT_EQ(*w.focus(), 4);
      EXPECT_EQ(*w[1], 3);
      EXPECT_EQ(*w[2], 2);
      EXPECT_EQ(w[3], v.end());
    } else if (i == 1U) {
      EXPECT_EQ(w[-3], v.end());
      EXPECT_EQ(w[-2], v.end());
      EXPECT_EQ(*w[-1], 4);
      EXPECT_EQ(*w.focus(), 3);
      EXPECT_EQ(*w[1], 2);
      EXPECT_EQ(*w[2], 1);
      EXPECT_EQ(w[3], v.end());
    } else if (i == 2U) {
      EXPECT_EQ(w[-3], v.end());
      EXPECT_EQ(*w[-2], 4);
      EXPECT_EQ(*w[-1], 3);
      EXPECT_EQ(*w.focus(), 2);
      EXPECT_EQ(*w[1], 1);
      EXPECT_EQ(*w[2], 0);
      EXPECT_EQ(w[3], v.end());
    } else if (i == 3U) {
      EXPECT_EQ(w[-3], v.end());
      EXPECT_EQ(*w[-2], 3);
      EXPECT_EQ(*w[-1], 2);
      EXPECT_EQ(*w.focus(), 1);
      EXPECT_EQ(*w[1], 0);
      EXPECT_EQ(w[2], v.end());
      EXPECT_EQ(w[3], v.end());
    } else if (i == 4U) {
      EXPECT_EQ(w[-3], v.end());
      EXPECT_EQ(*w[-2], 2);
      EXPECT_EQ(*w[-1], 1);
      EXPECT_EQ(*w.focus(), 0);
      EXPECT_EQ(w[1], v.end());
      EXPECT_EQ(w[2], v.end());
      EXPECT_EQ(w[3], v.end());
    }
    count++;
  }
  EXPECT_EQ(count, 5);
}
