#include "osr/types.h"

namespace osr {

struct graph {
  edge_idx_t add_edge(node_idx_t from, node_idx_t to, edge_flags_t, distance_t);
  void write(cista::memory_holder&) const;
  void write(std::filesystem::path const&) const;
  static cista::wrapped<graph> read(cista::memory_holder&&);

  vecvec<node_idx_t, edge_idx_t> out_edges_;
  vecvec<node_idx_t, edge_idx_t> in_edges_;
  vector_map<edge_idx_t, node_idx_t> edge_to_;
  vector_map<edge_idx_t, node_idx_t> edge_from_;
  vector_map<edge_idx_t, edge_flags_t> edge_flags_;
  vector_map<edge_idx_t, distance_t> edge_distance_;
};

}  // namespace osr