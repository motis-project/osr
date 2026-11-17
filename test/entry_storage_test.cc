#include "gtest/gtest.h"

#include <cstddef>
#include <cstdint>
#include <array>
#include <initializer_list>
#include <limits>
#include <vector>

#include "osr/routing/entry_storage.h"
#include "osr/routing/entry_storage_arena.h"
#include "osr/ways.h"

namespace osr {

namespace {

constexpr auto const kUnset = std::numeric_limits<std::uint32_t>::max();
constexpr auto const kNode = node_idx_t{0U};

struct test_slot {
  std::uint32_t value_{kUnset};
};

using storage_t = entry_storage<test_slot, 2U>;

ways::routing make_routing(
    std::initializer_list<std::size_t> const ways_per_node) {
  auto w = ways::routing{};
  for (auto const n : ways_per_node) {
    w.node_ways_.add_back_sized(n);
  }
  return w;
}

}  // namespace

TEST(entry_storage, unwritten_slots_read_as_unset) {
  auto const s = storage_t{};

  EXPECT_EQ(kUnset, s[storage_t::kInlineN - 1U].value_);
  EXPECT_EQ(kUnset, s[storage_t::kInlineN].value_);
  EXPECT_EQ(kUnset, s[storage_t::kN - 1U].value_);
}

TEST(entry_storage, promotion_preserves_inline_and_extra_slots) {
  auto const w = make_routing({3U});
  auto arena = entry_storage_arena{};
  auto s = storage_t{};

  for (auto i = std::size_t{0U}; i != storage_t::kInlineN; ++i) {
    s.slot(i, w, kNode, arena).value_ = static_cast<std::uint32_t>(100U + i);
  }
  ASSERT_FALSE(s.is_overflow());

  auto const promoting = storage_t::index(2U, direction::kForward);
  s.slot(promoting, w, kNode, arena).value_ = 200U;
  ASSERT_TRUE(s.is_overflow());

  for (auto i = std::size_t{0U}; i != storage_t::kInlineN; ++i) {
    EXPECT_EQ(100U + i, s[i].value_);
  }
  EXPECT_EQ(200U, s[promoting].value_);
  EXPECT_EQ(kUnset, s[storage_t::index(2U, direction::kBackward)].value_);
}

TEST(entry_storage, overflow_block_matches_node_degree) {
  auto const w = make_routing({3U});
  auto arena = entry_storage_arena{};
  auto s = storage_t{};

  s.slot(storage_t::index(2U, direction::kForward), w, kNode, arena).value_ =
      1U;

  EXPECT_EQ(storage_t::slot_count(3U) * sizeof(storage_t::slot_t),
            arena.offset_);
}

TEST(entry_storage, maximum_degree_exposes_every_slot) {
  auto const w = make_routing({kMaxWaysPerNode});
  auto arena = entry_storage_arena{};
  auto s = storage_t{};

  for (auto i = std::size_t{0U}; i != storage_t::kN; ++i) {
    s.slot(i, w, kNode, arena).value_ = static_cast<std::uint32_t>(i + 1U);
  }
  for (auto i = std::size_t{0U}; i != storage_t::kN; ++i) {
    EXPECT_EQ(i + 1U, s[i].value_);
  }
  EXPECT_EQ(storage_t::kN * sizeof(storage_t::slot_t), arena.offset_);
}

TEST(entry_storage, additional_nodes_use_maximum_size) {
  auto const w = make_routing({1U});
  auto arena = entry_storage_arena{};
  auto s = storage_t{};
  auto const additional_node = node_idx_t{1U};

  s.slot(storage_t::kN - 1U, w, additional_node, arena).value_ = 42U;

  EXPECT_EQ(42U, s[storage_t::kN - 1U].value_);
  EXPECT_EQ(storage_t::kN * sizeof(storage_t::slot_t), arena.offset_);
}

TEST(entry_storage_arena, accounts_for_alignment_padding) {
  auto arena = entry_storage_arena{};

  auto* const first = static_cast<std::byte*>(arena.allocate(1U, 1U));
  auto* const aligned = static_cast<std::byte*>(
      arena.allocate(sizeof(std::uint64_t), alignof(std::uint64_t)));
  auto* const after = static_cast<std::byte*>(arena.allocate(1U, 1U));

  EXPECT_EQ(0U,
            reinterpret_cast<std::uintptr_t>(aligned) % alignof(std::uint64_t));
  EXPECT_EQ(first + alignof(std::uint64_t), aligned);
  EXPECT_EQ(aligned + sizeof(std::uint64_t), after);
}

TEST(entry_storage_arena, reuses_chunks_after_reset) {
  struct block {
    std::array<std::uint64_t, 512U> data_{};
  };
  static_assert(entry_storage_arena::kChunkSize % sizeof(block) == 0U);

  auto arena = entry_storage_arena{};
  auto const per_chunk = entry_storage_arena::kChunkSize / sizeof(block);
  auto const n_blocks = 2U * per_chunk + 1U;
  auto first = std::vector<block*>{};
  auto second = std::vector<block*>{};

  for (auto i = std::size_t{0U}; i != n_blocks; ++i) {
    auto* const b = arena.create_array<block>(1U);
    b->data_[0] = i + 1U;
    first.push_back(b);
  }

  arena.reset();
  for (auto i = std::size_t{0U}; i != n_blocks; ++i) {
    second.push_back(arena.create_array<block>(1U));
  }

  EXPECT_EQ(first, second);
  EXPECT_EQ(0U, second.back()->data_[0]);
}

}  // namespace osr
