#pragma once

#include <cinttypes>

#include "ankerl/cista_adapter.h"

#include "cista/containers/vecvec.h"
#include "cista/hashing.h"
#include "cista/strong.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename K, typename V>
using vector_map = cista::raw::vector_map<K, V>;

template <typename K,
          typename V,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_map = cista::raw::ankerl_map<K, V, Hash, Equality>;

using osm_node_idx_t = cista::strong<std::int64_t, struct osm_node_idx_>;
using osm_rel_idx_t = cista::strong<std::int64_t, struct osrm_rel_idx_>;

using node_idx_t = cista::strong<std::uint32_t, struct node_idx_>;
using edge_idx_t = cista::strong<std::uint32_t, struct edge_idx_>;

using edge_flags_t = std::uint64_t;
using edge_weight_t = std::uint8_t;

struct graph {
  vecvec<node_idx_t, edge_idx_t> out_edges_;
  vecvec<node_idx_t, edge_idx_t> in_edges_;
  vector_map<edge_idx_t, node_idx_t> edge_to_;
  vector_map<edge_idx_t, node_idx_t> edge_from_;
  vector_map<edge_idx_t, edge_flags_t> edge_flags_;
  vector_map<edge_idx_t, edge_weight_t> edge_weight_;
};

}  // namespace osr