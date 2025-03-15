#pragma once

#include <cinttypes>

#include "osr/types.h"

namespace osr {

struct multi_counter {
  using size_type = bitvec64::size_type;

  bool is_multi(size_type const i) const { return multi_[i]; }
  
  // Force a node to be included in the graph as if it were at multiple ways intersection
  void make_multi(size_type const i) {
    once_.resize(std::max(once_.size(), i + 1U));
    multi_.resize(std::max(multi_.size(), i + 1U));
    once_.set(i, true);  // Mark as encountered once
    multi_.set(i, true); // Mark as encountered multiple times
  }

  void increment(size_type const i) {
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

  size_type size() const noexcept { return once_.size(); }

  void reserve(size_type const size) {
    once_.blocks_.reserve(size / bitvec64::bits_per_block);
    multi_.blocks_.reserve(size / bitvec64::bits_per_block);
  }

  bitvec64 once_, multi_;
};

}  // namespace osr