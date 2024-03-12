#pragma once

#include <filesystem>

#include "conf/configuration.h"

namespace osr {

struct config : conf::configuration {
  config(std::filesystem::path in, std::filesystem::path const& out)
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

  std::filesystem::path in_;
  std::filesystem::path graph_;
  std::filesystem::path node_map_;
  std::filesystem::path edge_map_;
  std::filesystem::path db_;
  std::filesystem::path tmp_;
  std::size_t db_max_size_;
};

}  // namespace osr