#pragma once

#include <vector>

#include "osr/dijkstra.h"
#include "osr/lookup.h"

namespace osr {

enum class search_profile : std::uint8_t {
  kPedestrian,
  kBike,
  kCar,
};

struct path {
  dist_t time_;
  std::vector<geo::latlng> polyline_;
};

struct routing_state {
  std::vector<start_dist> start_candidates_, dest_candidates_;
  dijkstra_state dijkstra_state_;
};

template <typename EdgeWeightFn>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t const max_dist,
                          routing_state& s,
                          EdgeWeightFn&& edge_weight_fn) {
  l.find(from, s.start_candidates_);
  utl::verify(!s.start_candidates_.empty(),
              "no start candidates around {} found", from);

  l.find(to, s.dest_candidates_);
  utl::verify(!s.dest_candidates_.empty(),
              "no destination candidates around {} found", to);

  s.dijkstra_state_.reset(w, max_dist);
  dijkstra(w, s.dijkstra_state_, max_dist, edge_weight_fn);
}

std::optional<path> route(ways const& w,
                          lookup const& l,
                          geo::latlng const& from,
                          geo::latlng const& to,
                          dist_t const max_dist,
                          routing_state& s,
                          search_profile const profile) {
  switch (profile) {
    case search_profile::kPedestrian:
      return route(w, l, from, to, max_dist, s, pedestrian{});
    case search_profile::kBike:
      return route(w, l, from, to, max_dist, s, bike{});
    case search_profile::kCar: return route(w, l, from, to, max_dist, s, car{});
  }
}

}  // namespace osr