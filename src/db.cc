#include "osr/db.h"

#include <span>

#include "osmium/osm/way.hpp"

#include "lmdb/lmdb.hpp"

#include "utl/get_or_create.h"
#include "utl/helpers/algorithm.h"

#include "osr/point.h"
#include "utl/to_vec.h"

/**
 * Binary layout:
 * - point [4 bytes lat, 4 bytes lng]
 * - [8 bytes relation index, ...]   <-- invariant: sorted, unique
 */
constexpr auto const kNodeDb = "NODE_DB";

/**
 * Binary layout
 * - 8 byte edge flags
 * - [8 bytes osm node index, ...]   <-- exact order
 */
constexpr auto const kRelDb = "REL_NODES_DB";

constexpr auto const kFlushSize = 100'000;

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
    constexpr auto const size = sizeof(std::decay_t<decltype(x)>);
    std::memcpy(&buf[offset], &x, size);
    offset += size;
  };

  for (auto const& x : data) {
    write(x);
  }
}

template <typename... T, typename Vec>
void read_buf(std::span<std::uint8_t const> buf, Vec& data, T&... t) {
  static_assert((std::is_trivially_copyable_v<T> && ...));
  static_assert(
      std::is_trivially_copyable_v<typename std::decay_t<Vec>::value_type>);

  auto offset = std::size_t{0U};
  auto const read = [&](auto& x) {
    constexpr auto const size = sizeof(std::decay_t<decltype(x)>);
    std::memcpy(&x, &buf[offset], size);
    offset += size;
  };

  (read(t), ...);

  data.resize((buf.size() - (sizeof(std::decay_t<T>) + ...)) /
              sizeof(typename Vec::value_type));
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

  ~impl() { flush(); }

  void add_node(osm_node_idx_t const i, point const p) {
    node_cache_[i].first = p;
    if (rel_cache_.size() >= kFlushSize) {
      flush();
    }
  }

  void add_relation(osm_rel_idx_t const i, osmium::WayNodeList const& l) {
    rel_cache_[i].second = utl::to_vec(
        l, [](osmium::NodeRef const& x) { return osm_node_idx_t{x.ref()}; });
    for (auto const& x : l) {
      node_cache_[osm_node_idx_t{x.ref()}].second.emplace_back(i);
    }
    if (rel_cache_.size() >= kFlushSize) {
      flush();
    }
  }

  void flush() {
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
      auto osm_rels_buf = std::vector<osm_rel_idx_t>{};
      auto pos_buf = point{};
      for (auto& [n, node] : node_cache_) {
        auto& [pos, osm_rels] = node;
        utl::sort(osm_rels);

        auto const existing = t.get(db, to_idx(n));
        if (existing.has_value()) {
          read_buf(span(*existing), osm_rels_buf, pos_buf);
          auto const middle = osm_rels_buf.insert(
              end(osm_rels_buf), begin(osm_rels), end(osm_rels));
          std::inplace_merge(begin(osm_rels_buf), middle, end(osm_rels_buf));
          osm_rels_buf.erase(
              std::unique(begin(osm_rels_buf), end(osm_rels_buf)),
              end(osm_rels_buf));
          write_buf(buf, osm_rels_buf, pos_buf);
        } else {
          write_buf(buf, osm_rels, pos);
        }

        t.put(db, to_idx(n), view(buf));
      }
    }

    t.commit();

    node_cache_.clear();
    rel_cache_.clear();
  }

  hash_map<osm_node_idx_t, std::pair<point, std::vector<osm_rel_idx_t>>>
      node_cache_;
  hash_map<osm_rel_idx_t, std::pair<edge_flags_t, std::vector<osm_node_idx_t>>>
      rel_cache_;
  lmdb::env env_;
};

db::db(std::filesystem::path const& path, std::uint64_t const max_size)
    : impl_(std::make_unique<impl>(path, max_size)) {}

db::~db() = default;

void db::add_node(osm_node_idx_t const i, point const pos) {
  impl_->add_node(i, pos);
}

void db::add_relation(osm_rel_idx_t const i, osmium::WayNodeList const& l) {
  impl_->add_relation(i, l);
}

}  // namespace osr