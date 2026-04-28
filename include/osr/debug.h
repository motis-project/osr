#pragma once

#include "fmt/core.h"

namespace osr {

constexpr bool osr_tracing_enabled = true;

template<typename... Args>
void trace(fmt::format_string<Args...> fmt, Args&&... args) {
  if constexpr (osr_tracing_enabled) {
    fmt::print(fmt, std::forward<Args>(args)...);
  }
}

} // namespace osr