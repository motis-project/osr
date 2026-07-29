#pragma once

#include <string_view>
#include <vector>

#include "geo/polyline.h"

#include "osr/elevation_storage.h"
#include "osr/location.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/mode.h"
#include "osr/routing/parameters.h"
#include "osr/routing/path.h"
#include "osr/routing/profile.h"
#include "osr/routing/with_profile.h"
#include "osr/types.h"

namespace osr {

struct ways;

template <Profile, bool EarlyTermination>
struct dijkstra;

template <Profile>
struct bidirectional;

struct sharing_data;

template <Profile P>
bidirectional<P>& get_bidirectional();

template <Profile P>
dijkstra<P, false>& get_dijkstra();

namespace {
std::optional<path> try_direct(osr::location const& from,
                               osr::location const& to);
}

template <Profile P>
struct search_state {
  void run(cost_t const distance) { can_continue_ = !d_.run(...); }

  std::vector<std::optional<path>> results() {
    auto const distance_lng_degrees =
        geo::approx_distance_lng_degrees(from_.pos_);

    auto results = std::vector<std::optional<path>>{};

    for (auto const [m, t] : utl::zip(to_match_, to_)) {
      if (auto const direct = try_direct(from_, t); direct.has_value()) {
        results.push_back(direct);
      } else {
        auto const limit_squared_max_matching_distance =
            geo::approx_squared_distance(from_.pos_, t.pos_,
                                         distance_lng_degrees) /
            kMaxMatchingDistanceSquaredRatio;

        for (auto const start : from_match_) {
          auto const c =
              best_candidate(params, w, d_, t.lvl_, m, distance_, dir_, true,
                             start, limit_squared_max_matching_distance);

          if (c.has_value()) {
            auto [nc, wc, n, p] = *c;
            d.cost_.at(n.get_key()).write(n, p);
            if (do_reconstruct(p)) {
              p = reconstruct<P>(params, w, l, blocked, sharing, elevations, d_,
                                 from, t, start, *wc, *nc, n, p.cost_, dir);
              p.uses_elevator_ = true;
            }
            results.push_back(p);
          }
        }
      }
    }
  }

  dijkstra<P, false> d_;
  location const& from_;
  std::vector<location> const& to_;
  match_view_t const& from_match_;
  std::vector<match_t> const& to_match_;
  direction const dir_;
  cost_t distance_;
  bool can_continue_ = true;
};

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

std::optional<path> route(profile_parameters const&,
                          ways const&,
                          lookup const&,
                          search_profile,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra);

std::vector<std::optional<path>> route(
    profile_parameters const&,
    ways const&,
    lookup const&,
    search_profile const,
    location const& from,
    std::vector<location> const& to,
    match_view_t from_match,
    std::vector<match_t> const& to_match,
    cost_t const max,
    direction const,
    bitvec<node_idx_t> const* blocked = nullptr,
    sharing_data const* sharing = nullptr,
    elevation_storage const* = nullptr,
    std::function<bool(path const&)> const& do_reconstruct = [](path const&) {
      return false;
    });

std::optional<path> route(profile_parameters const&,
                          ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          location const& from,
                          location const& to,
                          match_view_t from_match,
                          match_view_t to_match,
                          cost_t const max,
                          direction const dir,
                          bitvec<node_idx_t> const* blocked = nullptr,
                          sharing_data const* sharing = nullptr,
                          elevation_storage const* = nullptr,
                          routing_algorithm = routing_algorithm::kDijkstra);

}  // namespace osr
