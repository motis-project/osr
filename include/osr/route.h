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

search_profile to_profile(std::string_view s);

std::string_view to_str(search_profile const p);

struct path {
  dist_t time_;
  double dist_{0.0};
  std::vector<geo::latlng> polyline_;
};

// void route(ways const&,
//            match_t const& from,
//            dist_t max_dist,
//            search_profile,
//            search_dir,
//            dijkstra_state&);

std::optional<path> route(ways const&,
                          lookup const&,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t max_dist,
                          search_profile,
                          dijkstra_state&);

std::optional<path> reconstruct(ways const&,
                                dijkstra_state const&,
                                match_t const& to,
                                search_profile,
                                search_dir);

}  // namespace osr