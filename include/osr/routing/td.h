#pragma once

#include <chrono>
#include <cinttypes>
#include <algorithm>

#include "osr/types.h"

namespace osr {

using unixtime_t = std::uint32_t;  // *minutes* since 1970/1/1 00:00
using duration_t = std::uint32_t;

struct blocked_time {
  friend bool operator<(blocked_time const& a, blocked_time const& b) {
    return std::tie(a.n_, a.from_) < std::tie(b.n_, b.from_);
  }

  friend std::ostream& operator<<(std::ostream& out, blocked_time const& b) {
    return out << "{ n=" << b.n_ << ", interval=[" << b.from_ << ", " << b.to_
               << "[ }";
  }

  bool contains(unixtime_t const t) const noexcept {
    return from_ <= t && t < to_;
  }

  template <direction SearchDir>
  duration_t wait_time(unixtime_t const t) const {
    assert(contains(t));
    return static_cast<duration_t>(
        SearchDir == direction::kForward ? to_ - t : t - from_);
  }

  node_idx_t n_;
  unixtime_t from_, to_;
};

using blocked_times_t = std::vector<blocked_time>;

struct blocked {
  template <direction SearchDir = direction::kForward>
  duration_t get_wait_time(node_idx_t const n, unixtime_t const t) const {
    if (!is_blocked_.test(n)) {
      return 0;
    }
    auto const it =
        std::lower_bound(begin(blocked_times_), end(blocked_times_),
                         blocked_time{.n_ = n, .from_ = t, .to_ = t});
    if (it->n_ == n && it->contains(t)) {
      return it->template wait_time<SearchDir>(t);
    } else if (it != begin(blocked_times_) && (it - 1)->n_ == n &&
               (it - 1)->contains(t)) {
      return (it - 1)->template wait_time<SearchDir>(t);
    }
    return 0;
  }

  bitvec<node_idx_t> is_blocked_;
  blocked_times_t blocked_times_;
};

}  // namespace osr