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
  dist_t time_{kInfeasible};
  double dist_{0.0};
  std::vector<geo::latlng> polyline_{};
};

std::optional<path> route(ways const&,
                          lookup const&,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t max_dist,
                          search_profile,
                          dijkstra_state&);

std::vector<std::optional<path>> route(ways const&,
                                       lookup const&,
                                       geo::latlng const& from,
                                       std::vector<geo::latlng> const& to,
                                       dist_t max_dist,
                                       search_profile,
                                       direction,
                                       dijkstra_state&);

}  // namespace osr