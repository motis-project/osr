#pragma once

#include <cinttypes>
#include <string_view>
#include <vector>

#include "fmt/ranges.h"
#include "fmt/std.h"

#include "osr/buf_io.h"
#include "osr/point.h"
#include "osr/types.h"

namespace osr {

struct node_info {
  static bool has_more_than_one_way(std::string_view s) {
    return s.size() >
           (sizeof(point) + sizeof(node_idx_t) + sizeof(osm_way_idx_t));
  }

  void merge(node_info const& o) {
    auto const middle = ways_.insert(end(ways_), begin(o.ways_), end(o.ways_));
    std::inplace_merge(begin(ways_), middle, end(ways_));
    ways_.erase(std::unique(begin(ways_), end(ways_)), end(ways_));
  }

  static void write_node_idx(std::string_view s, node_idx_t const n) {
    auto const s_mut = const_cast<char*>(s.data());
    std::memcpy(s_mut, &n, sizeof(node_idx_t));
  }

  void write(std::vector<std::uint8_t>& buf) const {
    write_buf(buf, ways_, n_, p_);
  }

  node_info& read(std::string_view s) {
    read_buf(span(s), ways_, n_, p_);
    return *this;
  }

  static node_idx_t read_node_idx(std::string_view s) {
    auto x = node_idx_t{};
    std::memcpy(&x, s.data(), sizeof(node_idx_t));
    return x;
  }

  static point read_point(std::string_view s) {
    auto x = point{};
    std::memcpy(&x, s.data() + sizeof(node_idx_t), sizeof(point));
    return x;
  }

  node_idx_t n_{node_idx_t::invalid()};
  point p_;
  std::vector<osm_way_idx_t> ways_;
};

inline auto format_as(node_info const& i) {
  return std::tuple{i.n_, i.p_, i.ways_};
}

}  // namespace osr