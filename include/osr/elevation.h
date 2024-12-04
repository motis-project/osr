#pragma once

#include <filesystem>
#include <memory>

#include "cista/mmap.h"
#include "osr/preprocessing/elevation/dem_source.h"
#include "osr/preprocessing/elevation/elevation.h"
#include "osr/preprocessing/elevation/location.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "utl/enumerate.h"
#include "utl/parallel_for.h"
#include "utl/progress_tracker.h"

namespace osr {

struct elevation {
  elevation(std::filesystem::path const& p, cista::mmap::protection const mode);
  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  void set_elevations(ways& w, preprocessing::elevation::dem_source const& dem, std::shared_ptr<utl::progress_tracker>& pt) {
    pt->in_high(w.n_ways());
    auto elevations_up = std::vector<std::vector<preprocessing::elevation::elevation_t>>{};
    auto elevations_down = std::vector<std::vector<preprocessing::elevation::elevation_t>>{};
    elevations_up.resize(w.n_ways());
    elevations_down.resize(w.n_ways());
    utl::parallel_for_run(w.n_ways(), [&](auto const way_idx){
      auto total_up = preprocessing::elevation::elevation_t{};
      auto total_down = preprocessing::elevation::elevation_t{};
      auto last_elevation = preprocessing::elevation::NO_ELEVATION_DATA;
      auto& up = elevations_up[way_idx];
      auto& down = elevations_down[way_idx];
      for (auto const& node : w.r_->way_nodes_[way_idx_t{way_idx}]) {
        auto const point = w.get_node_pos(node);
        // TODO get(point)
        auto const elevation = dem.get(preprocessing::elevation::location{point.lat_, point.lng_});
        if (elevation != preprocessing::elevation::NO_ELEVATION_DATA) {
          if (last_elevation == preprocessing::elevation::NO_ELEVATION_DATA) {
            last_elevation = elevation;
          }
          if (elevation > last_elevation) {
            total_up += elevation - last_elevation;
          } else {
            total_down += last_elevation - elevation;
          }
          last_elevation = elevation;
        }
        up.push_back(total_up);
        down.push_back(total_down);
      }
      if (total_up == preprocessing::elevation::elevation_t{0}) {
        up.clear();
      }
      if (total_down == preprocessing::elevation::elevation_t{0}) {
        down.clear();
      }
    }, pt->update_fn());
    // Insert elevations
    for (auto& [source, destination] : std::vector{
      std::pair{std::ref(elevations_up), std::ref(w.r_->way_node_elevation_up_)},
      std::pair{std::ref(elevations_down), std::ref(w.r_->way_node_elevation_down_)},
    }) {
      auto& target = destination.get();
      for (auto const [i, elevations] : utl::enumerate(source.get())) {
        if (!elevations.empty()) {
          target.resize(i);
          target.emplace_back(elevations);
        }
      }
    }
  }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  mm_vecvec<way_idx_t, int> elevation_up_m_;
  mm_vecvec<way_idx_t, int> elevation_down_m_;
};
}  // namespace osr