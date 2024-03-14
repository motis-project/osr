#include "osr/config.h"

namespace osr {

config::config(std::filesystem::path in, std::filesystem::path const& out)
    : configuration{"Options"},
      in_{std::move(in)},
      graph_{out / "graph.bin"},
      node_map_{out / "node2osm.bin"},
      edge_map_{out / "edge2osm.bin"},
      db_{out / "db.mdb"},
      tmp_{out},
      db_max_size_{196ULL * 1024ULL * 1024ULL * 1024ULL} {
  param(in_, "in", "OpenStreetMap .osm.pbf input path");
  param(graph_, "graph", "output graph path");
  param(node_map_, "node_map", "node map path");
  param(edge_map_, "edge_map", "edge map path");
  param(db_, "db", "database path");
  param(tmp_, "tmp", "temporary directory path");
  param(db_max_size_, "db_max_size", "maximum memory map size");
}

config::~config() = default;

}  // namespace osr