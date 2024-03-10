#include "osr/db.h"

#include <span>

#include "lmdb/lmdb.hpp"

#include "utl/byte_literals.h"

/**
 * Binary layout:
 * - 8 bytes latitude
 * - 8 bytes longitude
 * - [... 8 bytes relation index]
 */
constexpr auto const kNodeDb = "NODE_DB";

/**
 * Binary layout
 * - 8 byte edge flags
 * - [... 8 bytes osm node index]
 */
constexpr auto const kRelDb = "REL_NODES_DB";

namespace osr {

template <typename... T, typename Vec>
void write_buf(std::vector<std::uint8_t>& buf, Vec&& data, T&&... t) {
  static_assert((std::is_trivially_copyable_v<std::decay_t<T>> && ...));
  static_assert(std::is_trivially_copyable_v<
                std::decay_t<typename std::decay_t<Vec>::value_type>>);

  buf.resize((sizeof(t) + ...) +
             data.size() *
                 sizeof(std::decay_t<typename std::decay_t<Vec>::value_type>));

  auto offset = std::size_t{0U};
  auto const write = [&](auto&& x) {
    std::memcpy(&buf[offset], &x, sizeof(std::decay_t<decltype(x)>));
    offset += sizeof(x);
  };

  for (auto const& x : data) {
    write(x);
  }
}

template <typename... T, typename Vec>
void read_buf(std::span<std::uint8_t const>& buf, Vec& data, T&... t) {
  static_assert((std::is_trivially_copyable_v<T> && ...));
  static_assert(
      std::is_trivially_copyable_v<typename std::decay_t<Vec>::value_type>);

  data.resize((buf.size() - (sizeof(std::decay_t<T>) + ...)) /
              sizeof(typename Vec::value_type));

  auto offset = std::size_t{0U};
  auto const read = [&](auto& x) {
    std::memcpy(&x, &buf[offset], sizeof(x));
    offset += sizeof(x);
  };

  (read(t), ...);
  for (auto& x : data) {
    read(x);
  }
}

std::string_view view(std::vector<std::uint8_t> const& v) {
  return {reinterpret_cast<char const*>(v.data()), v.size()};
}

std::span<std::uint8_t const> span(std::string_view s) {
  return {reinterpret_cast<std::uint8_t const*>(s.data()), s.size()};
}

struct db::impl {
  impl(std::filesystem::path const& path, std::uint64_t const max_size) {
    env_.set_maxdbs(2);
    env_.set_mapsize(max_size);
    env_.open(path.c_str(),
              lmdb::env_open_flags::NOSUBDIR | lmdb::env_open_flags::NOTLS);

    auto t = lmdb::txn{env_};
    t.dbi_open(kNodeDb, lmdb::dbi_flags::CREATE | lmdb::dbi_flags::INTEGERKEY);
    t.dbi_open(kRelDb, lmdb::dbi_flags::CREATE | lmdb::dbi_flags::INTEGERKEY);
    t.commit();
  }

  void add_node(osm_node_idx_t, geo::latlng) {}

  void add_relation(osm_rel_idx_t) {}

  void write_cache() {
    auto buf = std::vector<std::uint8_t>{};
    auto t = lmdb::txn{env_};

    {
      auto db = t.dbi_open(kRelDb);

      for (auto const& [r, rel] : rel_cache_) {
        auto const& [edge_flags, osm_nodes] = rel;
        write_buf(buf, osm_nodes, edge_flags);
        t.put(db, to_idx(r), view(buf));
      }
    }

    {
      auto db = t.dbi_open(kNodeDb);

      for (auto const& [n, node] : rel_cache_) {
        auto const& [pos, osm_rels] = node;
        auto const existing = t.get(db, to_idx(n));
        if (existing.has_value()) {
        }
      }
    }

    t.commit();
  }

  hash_map<osm_node_idx_t, std::pair<geo::latlng, std::vector<osm_rel_idx_t>>>
      node_cache_;
  hash_map<osm_rel_idx_t, std::pair<edge_flags_t, std::vector<osm_node_idx_t>>>
      rel_cache_;
  lmdb::env env_;
};

db::db(std::filesystem::path const& path, std::uint64_t const max_size)
    : impl_(std::make_unique<impl>(path, max_size)) {}

void db::add_node(osm_node_idx_t const i, geo::latlng const& pos) {
  impl_->add_node(i, pos);
}

void db::add_relation(osm_rel_idx_t const i) { impl_->add_relation(i); }

}  // namespace osr