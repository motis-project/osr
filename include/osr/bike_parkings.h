#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include "cista/serialization.h"

#include "geo/latlng.h"

#include "osr/routing/additional_edge.h"
#include "osr/routing/sharing_data.h"
#include "osr/types.h"

namespace osr {

struct bike_parking_edge {
  node_idx_t from_{};
  node_idx_t to_{};
  distance_t distance_{};
  way_idx_t underlying_way_{way_idx_t::invalid()};
  bool reverse_{false};
};

struct bike_parking_data {
  static cista::wrapped<bike_parking_data> read(std::filesystem::path const&);
  void write(std::filesystem::path const&) const;

  vec<geo::latlng> additional_node_coordinates_{};
  vec<bike_parking_edge> edges_{};
  bitvec<node_idx_t> start_allowed_{};
  bitvec<node_idx_t> end_allowed_{};
};

struct bike_parkings {
  static std::unique_ptr<bike_parkings> try_open(std::filesystem::path const&);

  sharing_data get_sharing_data(
      node_idx_t::value_t additional_node_offset) const;

  cista::wrapped<bike_parking_data> data_;
  std::vector<geo::latlng> additional_node_coordinates_{};
  hash_map<node_idx_t, std::vector<additional_edge>> additional_edges_{};
};

}  // namespace osr
