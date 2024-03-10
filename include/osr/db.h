#pragma once

#include <filesystem>

#include "osmium/osm/way.hpp"

#include "geo/latlng.h"

#include "osr/point.h"
#include "osr/types.h"

namespace osr {

struct db {
  db(std::filesystem::path const& path, std::uint64_t const max_size);
  ~db();

  void add_node(osm_node_idx_t, point);
  void add_relation(osm_rel_idx_t, osmium::WayNodeList const&);

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr