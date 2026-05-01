#include "gtest/gtest.h"
#include "osr/elevation_storage.h"
#include "osr/routing/profiles/foot.h"
#include "osr/types.h"

using namespace osr;

static float factor(float dist, elevation_storage::elevation e = {}, bool  exist = true) {
  auto const dx = std::max(to_idx(e.up_), to_idx(e.down_));
  auto const grad = dist > 0U ? static_cast<float>(dx) / static_cast<float>(dist) : 0.F;
  return  std::exp(-3.5F * std::abs(grad + (!exist ? 0 : 0.05F)));
}

// tobler_speed = speed * exp(-3.5 * |grad + (0.05 ? has_elevation : 0)|)
// formula: penalty = dist/tobler_speed - dist/base_speed

using elevation = elevation_storage::elevation;
using mono = elevation_monotonic_t;

TEST(elevation_cost, no_storage_zero_cost) {
  auto f = factor(100.F, {}, false);
  EXPECT_EQ(1, f);
}

TEST(elevation_cost, no_elevation_small_cost) {
  auto f = factor(100, {});
  EXPECT_NEAR(1, f, 0.2);
}

TEST(elevation_cost, ascend_descend_eq) {
  auto u = factor(100.F, {mono{5}, mono{0}});
  auto d = factor(100.F, {mono{0}, mono{5}});
  EXPECT_EQ(u, d);
}

