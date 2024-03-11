#pragma once

#include <filesystem>

namespace osr {

struct config {
  config(std::filesystem::path in, std::filesystem::path const& out)
      : in_{std::move(in)},
        graph_{out / "graph.bin"},
        node_map_{out / "node2osm.bin"},
        edge_map_{out / "edge2osm.bin"},
        db_{out / "db.mdb"},
        tmp_{out},
        db_max_size_{128ULL * 1024ULL * 1024ULL * 1024ULL} {}

  std::filesystem::path in_;
  std::filesystem::path graph_;
  std::filesystem::path node_map_;
  std::filesystem::path edge_map_;
  std::filesystem::path db_;
  std::filesystem::path tmp_;
  std::size_t db_max_size_;
};

}  // namespace osr