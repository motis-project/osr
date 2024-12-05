#include "osr/elevation.h"

#include "cista/mmap.h"

#include "utl/enumerate.h"
#include "utl/parallel_for.h"

#include "osr/preprocessing/elevation/dem_source.h"

namespace osr {

cista::mmap mm(std::filesystem::path const& p,
               char const* file,
               cista::mmap::protection const mode) {
  return cista::mmap{(p / file).generic_string().c_str(), mode};
}

elevation::elevation(std::filesystem::path const& p,
                     cista::mmap::protection const mode)
    : elevation_up_m_{mm_vec<int>{mm(p, "elevation_up_data.bin", mode)},
                      mm_vec<unsigned>(mm(p, "elevation_up_idx.bin", mode))},
      elevation_down_m_{
          mm_vec<int>{mm(p, "elevation_down_data.bin", mode)},
          mm_vec<unsigned>(mm(p, "elevation_down_idx.bin", mode))} {}

void elevation::set_elevations(ways& w,
                               preprocessing::elevation::dem_source const& dem,
                               std::shared_ptr<utl::progress_tracker>& pt) {
  pt->in_high(w.n_ways());
  auto elevations_up = std::vector<std::vector<elevation_t>>{};
  auto elevations_down = std::vector<std::vector<elevation_t>>{};
  elevations_up.resize(w.n_ways());
  elevations_down.resize(w.n_ways());
  utl::parallel_for_run(
      w.n_ways(),
      [&](auto const way_idx) {
        auto total_up = elevation_t{};
        auto total_down = elevation_t{};
        auto last_elevation = NO_ELEVATION_DATA;
        auto& up = elevations_up[way_idx];
        auto& down = elevations_down[way_idx];
        for (auto const& node : w.r_->way_nodes_[way_idx_t{way_idx}]) {
          auto const elevation = dem.get(w.get_node_pos(node));
          if (elevation != NO_ELEVATION_DATA) {
            if (last_elevation == NO_ELEVATION_DATA) {
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
        if (total_up == elevation_t{0}) {
          up.clear();
        }
        if (total_down == elevation_t{0}) {
          down.clear();
        }
      },
      pt->update_fn());
  // Insert elevations
  for (auto& [source, destination] : std::vector{
           std::pair{std::ref(elevations_up),
                     std::ref(w.r_->way_node_elevation_up_)},
           std::pair{std::ref(elevations_down),
                     std::ref(w.r_->way_node_elevation_down_)},
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

std::pair<elevation_t, elevation_t> get_elevations(ways::routing const& r,
                                                   way_idx_t const way,
                                                   std::uint16_t const from,
                                                   std::uint16_t const to) {
  auto const& [f, t] = from <= to ? std::pair{from, to} : std::pair{to, from};
  auto const& [up, down] = std::pair{
      way >= r.way_node_elevation_up_.size() ||
              r.way_node_elevation_up_[way].empty()
          ? elevation_t{0}
          : static_cast<elevation_t>(r.way_node_elevation_up_[way].at(t) -
                                     r.way_node_elevation_up_[way].at(f)),
      way >= r.way_node_elevation_down_.size() ||
              r.way_node_elevation_down_[way].empty()
          ? elevation_t{0}
          : static_cast<elevation_t>(r.way_node_elevation_down_[way].at(t) -
                                     r.way_node_elevation_down_[way].at(f)),
  };
  return from <= to ? std::pair{up, down} : std::pair{down, up};
}

}  // namespace osr
