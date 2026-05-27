#pragma once

#include <algorithm>
#include <cinttypes>
#include <vector>

#include "osr/types.h"

namespace osr {

template <std::uint64_t DenseLimit = 14'000'000'000ULL>
struct multi_counter {
  using size_type = bitvec64::size_type;
  static constexpr auto kDenseLimit = static_cast<size_type>(DenseLimit);

  bool is_multi(size_type const i) const {
    if (i < kDenseLimit) {
      return multi_[i];
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

    once_.resize(std::max(once_.size(), i + 1U));
    multi_.resize(std::max(multi_.size(), i + 1U));
    if (once_[i]) {
      if (!multi_[i]) {
        multi_.set(i, true);
      }
    } else {
      once_.set(i, true);
    }
  }

  size_type size() const noexcept {
    return once_.size() + sparse_counter_.size();
  }

  void reserve(size_type const size) {
    auto const dense_size = std::min(size, kDenseLimit);
    once_.blocks_.reserve(bitvec64::num_blocks(dense_size));
    multi_.blocks_.reserve(bitvec64::num_blocks(dense_size));
  }

  template <typename Fn>
  void for_each_multi(Fn&& f) const {
    auto&& fn = f;
    multi_.for_each_set_bit(fn);

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

  bitvec64 once_, multi_;
  hash_map<size_type, std::uint8_t> sparse_counter_;
};

}  // namespace osr