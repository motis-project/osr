#include "gtest/gtest.h"

#include "osr/lookup.h"

using namespace osr;

namespace {

match_result::nodes some_nodes(node_idx_t::value_t const n) {
  return {.left_ = {.node_ = node_idx_t{n}, .dist_to_node_ = 1.0F, .cost_ = 2U},
          .right_ = {
              .node_ = node_idx_t{n + 1U}, .dist_to_node_ = 3.0F, .cost_ = 4U}};
}

}  // namespace

// An empty match is normal (a location with no usable candidate). The bucket
// bounds then point one past the end, which must not be dereferenced.
TEST(match_result, empty_match_trailing) {
  auto m = match_result{};
  m.start(level_t{0.F});
  m.add(1.0F, way_idx_t{7U}, some_nodes(1U));
  m.finish();
  m.start(level_t{0.F});  // empty, and last -> from == size()
  m.finish();

  ASSERT_EQ(2U, m.size());
  EXPECT_EQ(1U, m[match_idx_t{0U}].size());

  auto const empty = m[match_idx_t{1U}];
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(0U, empty.size());
  EXPECT_TRUE(empty.way_.empty());
  EXPECT_TRUE(empty.dist_to_way_.empty());
  EXPECT_TRUE(empty.nodes_.empty());
}

TEST(match_result, empty_match_between) {
  auto m = match_result{};
  m.start(level_t{0.F});
  m.finish();  // empty, first
  m.start(level_t{0.F});
  m.finish();  // empty, middle
  m.start(level_t{0.F});
  m.add(2.0F, way_idx_t{9U}, some_nodes(5U));
  m.finish();

  ASSERT_EQ(3U, m.size());
  EXPECT_TRUE(m[match_idx_t{0U}].empty());
  EXPECT_TRUE(m[match_idx_t{1U}].empty());
  ASSERT_EQ(1U, m[match_idx_t{2U}].size());
  EXPECT_EQ(way_idx_t{9U}, m[match_idx_t{2U}].way_[0]);
}

// Every match is empty: all bucket bounds sit at size() == 0.
TEST(match_result, all_matches_empty) {
  auto m = match_result{};
  for (auto i = 0U; i != 3U; ++i) {
    m.start(level_t{0.F});
    m.finish();
  }
  ASSERT_EQ(3U, m.size());
  for (auto i = 0U; i != 3U; ++i) {
    EXPECT_TRUE(m[match_idx_t{i}].empty());
  }
}

TEST(match_result, out_of_range_throws) {
  auto m = match_result{};
  m.start(level_t{0.F});
  m.finish();
  ASSERT_EQ(1U, m.size());
  EXPECT_ANY_THROW(m[match_idx_t{1U}]);
  EXPECT_ANY_THROW(m[match_idx_t{99U}]);
}

TEST(match_result, append_and_clear) {
  auto src = match_result{};
  src.start(level_t{0.F});
  src.finish();  // empty
  src.start(level_t{0.F});
  src.add(1.5F, way_idx_t{3U}, some_nodes(11U));
  src.finish();

  auto dst = match_result{};
  dst.append(src, match_idx_t{1U});
  dst.append(src, match_idx_t{0U});  // appending an empty match

  ASSERT_EQ(2U, dst.size());
  ASSERT_EQ(1U, dst[match_idx_t{0U}].size());
  EXPECT_EQ(way_idx_t{3U}, dst[match_idx_t{0U}].way_[0]);
  EXPECT_TRUE(dst[match_idx_t{1U}].empty());

  dst.clear();
  EXPECT_EQ(0U, dst.size());
  EXPECT_TRUE(dst.empty());
}
