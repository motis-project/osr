#pragma once

#include <cassert>
#include <cstddef>

#include <memory>
#include <type_traits>
#include <vector>

namespace osr {

// Bump allocator for the overflow blocks of routing entries
struct entry_storage_arena {
  static constexpr auto const kChunkSize = std::size_t{64U * 1024U};

  template <typename T>
  T* create_array(std::size_t const n) {
    static_assert(std::is_trivially_destructible_v<T>,
                  "arena blocks are never destructed");
    static_assert(alignof(T) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__);
    assert(n * sizeof(T) <= kChunkSize);
    auto* const p = static_cast<T*>(allocate(n * sizeof(T), alignof(T)));
    std::uninitialized_value_construct_n(p, n);
    return p;
  }

  void* allocate(std::size_t const size, std::size_t const align) {
    // Round the current offset up to the alignment boundary
    auto const offset = (offset_ + align - 1U) & ~(align - 1U);
    if (chunk_ < chunks_.size() && offset + size <= kChunkSize) [[likely]] {
      offset_ = offset + size;
      return chunks_[chunk_].get() + offset;
    }
    return allocate_chunk(size);
  }

  void reset() noexcept {
    chunk_ = 0U;
    offset_ = 0U;
  }

  void* allocate_chunk(std::size_t const size) {
    if (chunk_ < chunks_.size()) {
      ++chunk_;
    }
    if (chunk_ == chunks_.size()) {
      chunks_.emplace_back(
          std::make_unique_for_overwrite<std::byte[]>(kChunkSize));
    }
    offset_ = size;
    return chunks_[chunk_].get();
  }

  std::vector<std::unique_ptr<std::byte[]>> chunks_;
  std::size_t chunk_{0U};
  std::size_t offset_{0U};
};

}  // namespace osr
