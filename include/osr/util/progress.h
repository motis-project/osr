#pragma once

#include <cstddef>

#include "utl/progress_tracker.h"

namespace osr {

constexpr auto const kProgressUpdateInterval = std::size_t{4096U};

inline auto sparse_update_fn(utl::progress_tracker& pt) {
  return [&pt](std::size_t const i) {
    if (i % kProgressUpdateInterval == 0U) {
      pt.update_monotonic(i);
    }
  };
}

}  // namespace osr
