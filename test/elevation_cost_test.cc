#include "gtest/gtest.h"
#include "osr/elevation_storage.h"
#include "osr/routing/profiles/foot.h"
#include "osr/types.h"

using namespace osr;

static float factor(float dist, elevation_storage::elevation e = {}, float speed = 1.2F, bool  exist = true) {
  auto const dx = std::max(to_idx(e.up_), to_idx(e.down_));
  auto const grad = dist > 0U ? static_cast<float>(dx) / static_cast<float>(dist) : 0.F;
  auto const tobler_speed = speed * std::exp(-3.5F * std::abs(grad + (!exist ? 0 : 0.05F)));
  return tobler_speed/speed;
}

// tobler_speed = speed * exp(-3.5 * |grad + (0.05 ? has_elevation : 0)|)
// formula: penalty = dist/tobler_speed - dist/base_speed

using elevation = elevation_storage::elevation;
using mono = elevation_monotonic_t;

TEST(elevation_cost, no_storage_zero_cost) {
  auto f = factor(100.F, {}, 1.2, false);
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

TEST(elevation_cost, ascend_cost) {
  auto f = factor(100, {mono{10}, mono{0}});
  EXPECT_LE(f, 1);
}

TEST(elevation_cost, linear_scale) {
  auto base_speed = 2.F;
  auto f = factor(100, {mono{10}, mono{0}}, base_speed);

  for (auto i = 2; i < 10; ++i) {
    auto s = factor(100, {mono{10}, mono{0}}, base_speed * i);
    EXPECT_NEAR(f, s, 1e-6);
  }
}

