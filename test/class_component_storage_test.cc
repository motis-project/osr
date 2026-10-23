#include "gtest/gtest.h"

#include <algorithm>
#include <array>
#include <limits>
#include <vector>

#include "cista/io.h"
#include "cista/serialization.h"

#include "osr/ways.h"

namespace osr {

constexpr auto const kInaccessible = std::numeric_limits<std::uint32_t>::max();

TEST(class_component_storage, absent_and_single_way_components) {
  auto const comp = std::array{0U, 1U, kInaccessible};
  auto const sizes = std::array{1U, 1U};
  auto const c = class_components::build(comp, sizes);
  for (auto i = 0U; i != comp.size(); ++i) {
    EXPECT_FALSE(c.get(way_idx_t{i}));
  }
  EXPECT_TRUE(c.exception_way_.empty());
}

TEST(class_component_storage, distinct_ids_ranked_by_size) {
  auto const comp = std::array{0U, 1U, 2U, 0U, 1U, 2U, 1U, 2U, 1U};
  auto const sizes = std::array{2U, 4U, 3U};
  auto const c = class_components::build(comp, sizes);
  EXPECT_EQ(c.get(way_idx_t{0U}), 3U);
  EXPECT_EQ(c.get(way_idx_t{1U}), 1U);
  EXPECT_EQ(c.get(way_idx_t{2U}), 2U);
  EXPECT_EQ(c.get(way_idx_t{0U}), c.get(way_idx_t{3U}));
  EXPECT_NE(c.get(way_idx_t{0U}), c.get(way_idx_t{1U}));
}

TEST(class_component_storage, exception_boundary_and_serialization) {
  auto sizes = std::vector<std::uint32_t>(300U, 2U);
  auto comp = std::vector<std::uint32_t>{};
  for (auto i = 0U; i != sizes.size(); ++i) {
    comp.insert(end(comp), sizes[i], i);
  }
  comp.push_back(kInaccessible);
  sizes.push_back(1U);
  comp.push_back(300U);
  auto c = class_components::build(comp, sizes);
  auto bytes = cista::serialize<cista::kDefaultMode>(c);
  auto const* loaded =
      cista::deserialize<class_components, cista::kDefaultMode>(bytes);
  for (auto i = 0U; i != 300U; ++i) {
    EXPECT_EQ(c.get(way_idx_t{2U * i}), i + 1U);
    EXPECT_EQ(c.get(way_idx_t{2U * i + 1U}), i + 1U);
    EXPECT_EQ(loaded->get(way_idx_t{2U * i}), i + 1U);
  }
  EXPECT_FALSE(c.get(way_idx_t{600U}));
  EXPECT_FALSE(c.get(way_idx_t{601U}));
  EXPECT_FALSE(loaded->get(way_idx_t{601U}));
  EXPECT_TRUE(std::is_sorted(begin(c.exception_way_), end(c.exception_way_)));
  EXPECT_EQ(c.exception_way_.size(), c.exception_component_.size());
}

}  // namespace osr
