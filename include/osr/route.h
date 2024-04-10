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

search_profile to_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  struct segment {
    std::vector<geo::latlng> polyline_;
    level_t level_;
    way_idx_t way_;
  };

  dist_t time_{kInfeasible};
  double dist_{0.0};
  std::vector<segment> segments_{};
};

std::optional<path> route(ways const&,
                          lookup const&,
                          location const& from,
                          location const& to,
                          dist_t max_dist,
                          search_profile,
                          dijkstra_state&);

std::vector<std::optional<path>> route(ways const&,
                                       lookup const&,
                                       location const& from,
                                       std::vector<location> const& to,
                                       dist_t max_dist,
                                       search_profile,
                                       direction,
                                       dijkstra_state&);

}  // namespace osr