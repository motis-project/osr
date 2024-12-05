#include "osr/elevation.h"

#include <array>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/parallel_for.h"

#include "osr/preprocessing/elevation/dem_source.h"

namespace osr {

cista::mmap mm(std::filesystem::path const& p,
               char const* file,
               cista::mmap::protection const mode) {
  return cista::mmap{(p / file).generic_string().c_str(), mode};
}

constexpr auto const kUpDataName = "elevation_up_data.bin";
constexpr auto const kUpIndexName = "elevation_up_idx.bin";
constexpr auto const kDownDataName = "elevation_down_data.bin";
constexpr auto const kDownIndexName = "elevation_down_idx.bin";

elevation::elevation(std::filesystem::path const& p,
                     cista::mmap::protection const mode)
    : elevation_up_m_{mm_vec<int>{mm(p, kUpDataName, mode)},
                      mm_vec<unsigned>(mm(p, kUpIndexName, mode))},
      elevation_down_m_{mm_vec<int>{mm(p, kDownDataName, mode)},
                        mm_vec<unsigned>(mm(p, kDownIndexName, mode))} {}

std::unique_ptr<elevation> elevation::try_open(
    std::filesystem::path const& path) {
  if (utl::any_of(
          std::array{kUpDataName, kUpIndexName, kDownDataName, kDownIndexName},
          [&](char const* const filename) {
            auto const full_path = path / filename;
            if (std::filesystem::exists(full_path)) {
              return false;
            } else {
              std::cout << full_path << " does not exist\n";
              return true;
            }
          })) {
    return {};
  }
  return std::make_unique<elevation>(path, cista::mmap::protection::READ);
}

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
  for (auto& [source, destination] : std::array{
           std::pair{std::ref(elevations_up), std::ref(elevation_up_m_)},
           std::pair{std::ref(elevations_down), std::ref(elevation_down_m_)},
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

std::pair<elevation_t, elevation_t> elevation::get_elevations(
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to) const {
  auto const& [f, t] = from <= to ? std::pair{from, to} : std::pair{to, from};
  auto const& [up, down] = std::pair{
      way >= elevation_up_m_.size() || elevation_up_m_[way].empty()
          ? elevation_t{0}
          : static_cast<elevation_t>(elevation_up_m_[way].at(t) -
                                     elevation_up_m_[way].at(f)),
      way >= elevation_down_m_.size() || elevation_down_m_[way].empty()
          ? elevation_t{0}
          : static_cast<elevation_t>(elevation_down_m_[way].at(t) -
                                     elevation_down_m_[way].at(f)),
  };
  return from <= to ? std::pair{up, down} : std::pair{down, up};
}

std::pair<elevation_t, elevation_t> get_elevations(elevation const* elevation,
                                                   way_idx_t const way,
                                                   std::uint16_t const from,
                                                   std::uint16_t const to) {
  return elevation == nullptr ? std::pair{elevation_t{0}, elevation_t{0}}
                              : elevation->get_elevations(way, from, to);
}

}  // namespace osr
