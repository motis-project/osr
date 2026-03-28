#include "gtest/gtest.h"

#include "osr/routing/profiles/foot.h"

osr::cost_t cost(const osr::way_properties p) {
  return osr::foot<false>::way_cost({}, p, osr::direction::kForward, 1);
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
