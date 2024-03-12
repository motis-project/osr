#pragma once

#include <filesystem>

#include "osmium/osm/way.hpp"

#include "geo/latlng.h"

#include "osr/node_info.h"
#include "osr/point.h"
#include "osr/types.h"
#include "osr/way_info.h"

namespace osr {

struct db {
  db(std::filesystem::path const& path, std::uint64_t const max_size);
  ~db();

  point get_node_pos(osm_node_idx_t);
  void get_node(osm_node_idx_t, node_info&);
  void get_way(osm_way_idx_t, way_info&);

  void write(hash_map<osm_way_idx_t, way_info>&);
  void write(hash_map<osm_node_idx_t, node_info>&);

  void write_graph(std::filesystem::path const& graph_path,
                   std::filesystem::path const& node_map_path,
                   std::filesystem::path const& edge_map_path);

  void write_debug(std::ostream&);

  void flush();

  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr