#pragma once

#include <cinttypes>

#include <algorithm>
#include <memory>
#include <vector>

#include "cista/containers/bitset.h"

#include "osr/types.h"

namespace osr {

template <std::uint64_t DenseLimit = 14'000'000'000ULL>
struct multi_counter {
  using size_type = std::uint64_t;
  static constexpr auto kDenseLimit = static_cast<size_type>(DenseLimit);
  static constexpr auto kPageBits = size_type{1U} << 16U;

  struct page {
    cista::bitset<kPageBits> once_, multi_;
  };

  bool is_multi(size_type const i) const {
    if (i < kDenseLimit) {
      auto const p = i / kPageBits;
      return p < pages_.size() && pages_[p] != nullptr &&
             pages_[p]->multi_.test(i % kPageBits);
    }

    if (auto const it = sparse_counter_.find(i); it != end(sparse_counter_)) {
      return it->second >= 2U;
    }
    return false;
  }

  void increment(size_type const i) {
    if (i >= kDenseLimit) {
      auto& state = sparse_counter_[i];
      if (state < 2U) {
        ++state;
      }
      return;
    }

    dense_size_ = std::max(dense_size_, i + 1U);
    auto const p = i / kPageBits;
    if (p >= pages_.size()) {
      pages_.resize(p + 1U);
    }
    if (pages_[p] == nullptr) {
      pages_[p] = std::make_unique<page>();
    }
    auto const bit = i % kPageBits;
    auto& pg = *pages_[p];
    if (pg.once_.test(bit)) {
      pg.multi_.set(bit);
    } else {
      pg.once_.set(bit);
    }
  }

  size_type size() const noexcept {
    return dense_size_ + sparse_counter_.size();
  }

  void reserve(size_type const size) {
    pages_.reserve((std::min(size, kDenseLimit) + kPageBits - 1U) / kPageBits);
  }

  template <typename Fn>
  void for_each_multi(Fn&& f) const {
    auto&& fn = f;
    for (auto p = size_type{0U}; p != pages_.size(); ++p) {
      if (pages_[p] != nullptr) {
        pages_[p]->multi_.for_each_set_bit(
            [&](std::size_t const bit) { fn(p * kPageBits + bit); });
      }
    }

    auto sparse_multi = std::vector<size_type>{};
    sparse_multi.reserve(sparse_counter_.size());
    for (auto const& [idx, state] : sparse_counter_) {
      if (state >= 2U) {
        sparse_multi.push_back(idx);
      }
    }

    std::sort(begin(sparse_multi), end(sparse_multi));
    for (auto const idx : sparse_multi) {
      fn(idx);
    }
  }

  std::vector<std::unique_ptr<page>> pages_;
  size_type dense_size_{0U};
  hash_map<size_type, std::uint8_t> sparse_counter_;
};

}  // namespace osr