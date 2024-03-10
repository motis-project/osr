#pragma once

#include <filesystem>

#include "geo/latlng.h"

#include "osr/types.h"

namespace osr {

struct db {
  db(std::filesystem::path const& path, std::uint64_t const max_size);

  void add_node(osm_node_idx_t, geo::latlng const&);
  void add_relation(osm_rel_idx_t);

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr