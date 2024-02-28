#pragma once

#include "cista/containers/vecvec.h"
#include "cista/strong.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename K, typename V>
using vector_map = cista::raw::vector_map<K, V>;

struct graph {};

}  // namespace osr