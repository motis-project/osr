#pragma once

#include <cinttypes>

#include "osr/types.h"

namespace osr {

struct multi_counter {
  using size_type = bitvec64::size_type;

  static constexpr size_type once_idx(size_type const i) { return i * 2U; }

  static constexpr size_type multi_idx(size_type const i) {
    return i * 2U + 1U;
  }

  bool is_multi(size_type const i) const { return v_[multi_idx(i)]; }

  void increment(size_type const i) {
    v_.resize(std::max(v_.size(), multi_idx(i) + 1U));
    if (v_[once_idx(i)]) {
      v_.set(multi_idx(i), true);
    } else {
      v_.set(once_idx(i), true);
    }
  }

  size_type size() const noexcept { return v_.size() / 2; }

  bitvec64 v_;
};

}  // namespace osr