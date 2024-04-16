#pragma once

#include <cinttypes>
#include <filesystem>

#include "ankerl/cista_adapter.h"

#include "cista/containers/bitvec.h"
#include "cista/containers/mmap_vec.h"
#include "cista/containers/paged.h"
#include "cista/containers/paged_vecvec.h"
#include "cista/containers/vector.h"
#include "cista/containers/vecvec.h"
#include "cista/strong.h"

namespace osr {

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using vecvec = cista::raw::vecvec<K, V, SizeType>;

template <typename K, typename V>
using vector_map = cista::basic_vector<V, cista::raw::ptr, false, K>;

template <typename T>
using vector = cista::basic_vector<T, cista::raw::ptr, false, std::uint64_t>;

template <typename K, typename V>
using mm_vec_map = cista::basic_mmap_vec<V, K>;

template <typename T>
using mm_vec = cista::basic_mmap_vec<T, std::uint64_t>;

template <typename K>
using mm_bitvec = cista::basic_bitvec<mm_vec<std::uint64_t>, K>;

template <typename K, typename V, typename SizeType = cista::base_t<K>>
using mm_vecvec = cista::basic_vecvec<K, mm_vec<V>, mm_vec<SizeType>>;

template <typename Key, typename T>
struct mmap_paged_vecvec_helper {
  using data_t = cista::paged<mm_vec<T>>;
  using idx_t = mm_vec<typename data_t::page_t>;
  using type = cista::paged_vecvec<idx_t, data_t, Key>;
};

template <typename Key, typename T>
using mm_paged_vecvec = mmap_paged_vecvec_helper<Key, T>::type;

template <typename T>
using vector = cista::basic_vector<T, cista::raw::ptr, false, std::uint64_t>;

using bitvec64 = cista::basic_bitvec<
    cista::basic_vector<std::uint64_t, cista::raw::ptr, false, std::uint64_t>>;

template <typename First, typename Second>
using pair = cista::pair<First, Second>;

template <typename K,
          typename V,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_map = cista::raw::ankerl_map<K, V, Hash, Equality>;

template <typename K,
          typename Hash = cista::hash_all,
          typename Equality = cista::equals_all>
using hash_set = cista::raw::ankerl_set<K, Hash, Equality>;

using osm_node_idx_t = cista::strong<std::uint64_t, struct osm_node_idx_>;
using osm_way_idx_t = cista::strong<std::uint64_t, struct osm_way_idx_>;

using way_idx_t = cista::strong<std::uint32_t, struct way_idx_>;
using node_idx_t = cista::strong<std::uint32_t, struct node_idx_>;

using level_t = cista::strong<std::uint8_t, struct level_>;

using distance_t = std::uint16_t;

using way_pos_t = std::uint8_t;

using cost_t = std::uint16_t;

constexpr auto const kInfeasible = std::numeric_limits<cost_t>::max();

enum class direction : std::uint8_t {
  kForward,
  kBackward,
};

constexpr direction opposite(direction const dir) {
  return dir == direction::kForward ? direction::kBackward
                                    : direction::kForward;
}

constexpr std::string_view to_str(direction const d) {
  switch (d) {
    case direction::kForward: return "forward";
    case direction::kBackward: return "backward";
  }
  std::unreachable();
}

}  // namespace osr