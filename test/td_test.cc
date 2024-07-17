#include "gtest/gtest.h"

#include "utl/enumerate.h"

#include "osr/routing/td.h"

using namespace osr;

TEST(osr, td) {
  auto const t = [](auto&& i) { return i; };
  auto const n = [](auto&& i) { return node_idx_t{i}; };

  auto const b = blocked{
      .is_blocked_ = bitvec<node_idx_t>{"1111"},
      .blocked_times_ = {
          {n(0), t(5), t(11)}, {n(0), t(15), t(20)}, {n(2), t(0), t(2)}}};

  auto const expected_0 = std::vector<unsigned>{
      0, 0, 0, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 5, 4, 3, 2, 1, 0, 0};
  for (auto const [i, e] : utl::enumerate(expected_0)) {
    EXPECT_EQ(e, b.get_wait_time<direction::kForward>(n(0), t(i)))
        << "i=" << i << ", expected=" << std::boolalpha << e;
  }

  for (auto i = 0U; i != 100; ++i) {
    EXPECT_EQ(0, b.get_wait_time<direction::kForward>(n(1), t(i)));
    EXPECT_EQ(0, b.get_wait_time<direction::kForward>(n(3), t(i)));
  }

  auto const expected_2 = std::vector<unsigned>{2, 1, 0, 0, 0, 0, 0};
  for (auto const [i, e] : utl::enumerate(expected_2)) {
    EXPECT_EQ(e, b.get_wait_time<direction::kForward>(n(2), t(i)))
        << "i=" << i << ", expected=" << std::boolalpha << e;
  }
}