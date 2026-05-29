#include "gtest/gtest.h"

#include "osr/routing/profiles/foot.h"

osr::cost_t cost(const osr::way_properties p) {
  return osr::foot<false>::way_cost(
             {}, osr::ways::routing{}, osr::way_idx_t::invalid(), p,
             osr::direction::kForward, 1, std::nullopt, osr::duration_t{0},
             osr::direction::kForward)
      .cost_;
}

TEST(way_cost_foot, combinations) {
  EXPECT_EQ(osr::kInfeasible, cost({
                                  .is_foot_accessible_ = false,
                                  .is_bike_accessible_ = false,
                                  .is_sidewalk_separate_ = false,
                              }));
  EXPECT_EQ(osr::kInfeasible, cost({
                                  .is_foot_accessible_ = false,
                                  .is_bike_accessible_ = false,
                                  .is_sidewalk_separate_ = true,
                              }));
  EXPECT_EQ(91, cost({
                    .is_foot_accessible_ = false,
                    .is_bike_accessible_ = true,
                    .is_sidewalk_separate_ = false,
                }));
  EXPECT_EQ(136, cost({
                     .is_foot_accessible_ = false,
                     .is_bike_accessible_ = true,
                     .is_sidewalk_separate_ = true,
                 }));
  EXPECT_EQ(1, cost({
                   .is_foot_accessible_ = true,
                   .is_bike_accessible_ = false,
                   .is_sidewalk_separate_ = false,
               }));
  EXPECT_EQ(46, cost({
                    .is_foot_accessible_ = true,
                    .is_bike_accessible_ = false,
                    .is_sidewalk_separate_ = true,
                }));
  EXPECT_EQ(1, cost({
                   .is_foot_accessible_ = true,
                   .is_bike_accessible_ = true,
                   .is_sidewalk_separate_ = false,
               }));
  EXPECT_EQ(46, cost({
                    .is_foot_accessible_ = true,
                    .is_bike_accessible_ = true,
                    .is_sidewalk_separate_ = true,
                }));
}
