#pragma once

#include <filesystem>
#include <optional>

#include "cista/mmap.h"
#include "osr/preprocessing/elevation/dem_source.h"
#include "osr/preprocessing/elevation/elevation.h"
#include "osr/preprocessing/elevation/location.h"
#include "osr/types.h"
#include "osr/ways.h"
#include "utl/helpers/algorithm.h"
#include "utl/pipes/iota.h"
#include "utl/pipes/remove_if.h"
#include "utl/pipes/vec.h"

namespace osr {

struct elevation {
  elevation(std::filesystem::path const& p, cista::mmap::protection const mode);
  cista::mmap mm(char const* file) {
    return cista::mmap{(p_ / file).generic_string().c_str(), mode_};
  }

  void set_elevations(ways& w, preprocessing::elevation::dem_source const& dem) {
    auto elevation_up = preprocessing::elevation::elevation_t{0};
    auto elevation_down = preprocessing::elevation::elevation_t{0};
    auto last_elevation = preprocessing::elevation::elevation_t{preprocessing::elevation::NO_ELEVATION_DATA};
    for (auto way = way_idx_t{0U}; way < w.n_ways(); ++way) {
      auto elevations_up = std::vector<preprocessing::elevation::elevation_t>{};
      auto elevations_down = std::vector<preprocessing::elevation::elevation_t>{};
      for (auto const& node : w.r_->way_nodes_[way]) {
        auto const point = w.get_node_pos(node);
        // TODO get(point)
        auto const elevation = dem.get(preprocessing::elevation::location{point.lat_, point.lng_});
        if (elevation != preprocessing::elevation::NO_ELEVATION_DATA) {
          if (last_elevation == preprocessing::elevation::NO_ELEVATION_DATA) {
            last_elevation = elevation;
          }
          if (elevation > last_elevation) {
            elevation_up += elevation - last_elevation;
          } else {
            elevation_down += last_elevation - elevation;
          }
          last_elevation = elevation;
        }
        elevations_up.push_back(elevation_up);
        elevations_down.push_back(elevation_down);
      }
      if (elevation_up == preprocessing::elevation::elevation_t{0}) {
        elevations_up.clear();
      }
      if (elevation_down == preprocessing::elevation::elevation_t{0}) {
        elevations_down.clear();
      }
      w.r_->way_node_elevation_up_.emplace_back(std::move(elevations_up));
      w.r_->way_node_elevation_down_.emplace_back(std::move(elevations_down));
    }
  }

  std::filesystem::path p_;
  cista::mmap::protection mode_;
  mm_vecvec<way_idx_t, int> elevation_up_m_;
  mm_vecvec<way_idx_t, int> elevation_down_m_;
};
}  // namespace osr