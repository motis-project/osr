#pragma once

#include <cassert>
#include <cstddef>

#include <algorithm>
#include <array>

#include "osr/routing/entry_storage_arena.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

// Inline storage for the per node entries during routing for way aware
// profiles, where entries provide a slot per (way, direction) pair of the node.
//
// Since most nodes have very few ways, the first kInlineWaysPerNode ways per
// entry are stored inline. If a slot outside that range is written, the entry
// is promoted to an overflow block allocated from the arena, which then stores
// all slots for that entry (= inline ignored).
//
// Extra is the number of profile specific slots that precede the way slots
// (e.g. walking state for car_sharing).
template <typename Slot, std::size_t Extra>
struct entry_storage {
  using slot_t = Slot;

  static constexpr auto const kExtra = Extra;
  static constexpr auto const kMaxWays = std::size_t{kMaxWaysPerNode};
  static constexpr auto const kInlineWays = std::size_t{kInlineWaysPerNode};
  static constexpr auto const kN = kExtra + 2U * kMaxWays;
  static constexpr auto const kInlineN = kExtra + 2U * kInlineWays;

  static_assert(kInlineWays <= kMaxWays);

  static constexpr std::size_t index(std::size_t const way,
                                     direction const dir) noexcept {
    return kExtra + 2U * way + (dir == direction::kForward ? 0U : 1U);
  }

  static constexpr std::size_t slot_count(std::size_t const n_ways) noexcept {
    return kExtra + 2U * n_ways;
  }

  static std::size_t slot_count(ways::routing const& w,
                                node_idx_t const n) noexcept {
    return to_idx(n) < w.node_ways_.size() ? slot_count(w.node_ways_[n].size())
                                           : kN /* (additional nodes) */;
  }

  Slot operator[](std::size_t const i) const noexcept {
    if (overflow_ != nullptr) [[unlikely]] {
      return overflow_[i];
    }
    return i < kInlineN ? inline_[i] : Slot{};
  }

  Slot& slot(std::size_t const i,
             ways::routing const& w,
             node_idx_t const n,
             entry_storage_arena& a) {
    if (overflow_ == nullptr) [[likely]] {
      if (i < kInlineN) [[likely]] {
        return inline_[i];
      }
      promote(slot_count(w, n), a);
      assert(i < slot_count(w, n));
    }
    return overflow_[i];
  }

  void promote(std::size_t const n_slots, entry_storage_arena& a) {
    assert(n_slots >= kInlineN && n_slots <= kN);
    auto* const o = a.template create_array<Slot>(n_slots);
    std::copy(begin(inline_), end(inline_), o);
    overflow_ = o;
  }

  std::array<Slot, kInlineN> inline_{};
  Slot* overflow_{nullptr};
};

}  // namespace osr
