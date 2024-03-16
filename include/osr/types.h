#pragma once

#include <cinttypes>
#include <filesystem>

#include "ankerl/cista_adapter.h"

#include "cista/containers/bitvec.h"
#include "cista/containers/vector.h"
#include "cista/containers/vecvec.h"
#include "cista/hashing.h"
#include "cista/memory_holder.h"
#include "cista/strong.h"

#include "osr/mmap_vec.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename K, typename V>
using vector_map = cista::basic_vector<V, cista::raw::ptr, false, K>;

template <typename T>
using vector = cista::basic_vector<T, cista::raw::ptr, false, std::uint64_t>;

using bitvec64 = cista::basic_bitvec<
    cista::basic_vector<std::uint64_t, cista::raw::ptr, false, std::uint64_t>>;

template <typename K,
          typename V,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_map = cista::raw::ankerl_map<K, V, Hash, Equality>;

using osm_node_idx_t = cista::strong<std::uint64_t, struct osm_node_idx_>;
using osm_way_idx_t = cista::strong<std::uint64_t, struct osm_way_idx_>;

using way_idx_t = cista::strong<std::uint32_t, struct way_idx_>;
using node_idx_t = cista::strong<std::uint32_t, struct node_idx_>;
using edge_idx_t = cista::strong<std::uint32_t, struct edge_idx_>;

using edge_flags_t = std::uint64_t;
using distance_t = std::uint16_t;

using edge_map_t = mmap_vec_map<edge_idx_t, osm_way_idx_t>;
using node_map_t = mmap_vec_map<node_idx_t, osm_node_idx_t>;

}  // namespace osr