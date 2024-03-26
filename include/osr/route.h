#pragma once

#include <string_view>
#include <vector>

#include "osr/dijkstra.h"
#include "osr/lookup.h"
#include "osr/types.h"

namespace osr {

enum class search_dir : std::uint8_t {
  kForward,
  kBackward,
};

enum class search_profile : std::uint8_t {
  kFoot,
  kBike,
  kCar,
};

search_profile read_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  dist_t time_;
  double dist_{0.0};
  std::vector<geo::latlng> polyline_;
};

struct routing_state {
  std::vector<start_dist> start_candidates_, dest_candidates_;
  dijkstra_state dijkstra_state_;
};

void route(ways const&,
           start_dist const& from,
           dist_t max_dist,
           search_profile,
           search_dir,
           routing_state&);

std::optional<path> reconstruct(ways const&,
                                routing_state const&,
                                start_dist const& to,
                                search_profile,
                                search_dir);

}  // namespace osr