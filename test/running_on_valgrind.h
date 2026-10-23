#pragma once

#if __has_include(<valgrind/valgrind.h>)
#include <valgrind/valgrind.h>
#endif

namespace osr::test {

inline bool running_on_valgrind() {
#ifdef RUNNING_ON_VALGRIND
  return RUNNING_ON_VALGRIND != 0U;
#else
  return false;
#endif
}

}  // namespace osr::test
