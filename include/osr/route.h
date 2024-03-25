#pragma once

#include <string_view>
#include <vector>

#include "osr/dijkstra.h"
#include "osr/lookup.h"
#include "osr/types.h"

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kBike,
  kCar,
};

search_profile read_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  dist_t time_;
  std::vector<geo::latlng> polyline_;
};

struct routing_state {
  std::vector<start_dist> start_candidates_, dest_candidates_;
  dijkstra_state dijkstra_state_;
};

std::optional<path> route(ways const&,
                          lookup const&,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t max_dist,
                          routing_state&,
                          search_profile);

}  // namespace osr