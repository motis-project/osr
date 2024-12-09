#include "osr/elevation_storage.h"

#include <array>
#include <ranges>

#include "utl/enumerate.h"
#include "utl/helpers/algorithm.h"
#include "utl/parallel_for.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/dem_source.h"
#include "osr/preprocessing/elevation/step_size.h"

namespace osr {

cista::mmap mm(std::filesystem::path const& p,
               char const* file,
               cista::mmap::protection const mode) {
  return cista::mmap{(p / file).string().data(), mode};
}

namespace elevation_files {
constexpr auto const kUpDataName = "elevation_up_data.bin";
constexpr auto const kUpIndexName = "elevation_up_idx.bin";
constexpr auto const kDownDataName = "elevation_down_data.bin";
constexpr auto const kDownIndexName = "elevation_down_idx.bin";
};  // namespace elevation_files

elevation_storage::elevation_storage(std::filesystem::path const& p,
                                     cista::mmap::protection const mode)
    : elevation_up_{mm_vec<elevation_t>{
                        mm(p, elevation_files::kUpDataName, mode)},
                    mm_vec<unsigned>(
                        mm(p, elevation_files::kUpIndexName, mode))},
      elevation_down_{
          mm_vec<elevation_t>{mm(p, elevation_files::kDownDataName, mode)},
          mm_vec<unsigned>(mm(p, elevation_files::kDownIndexName, mode))} {}

std::unique_ptr<elevation_storage> elevation_storage::try_open(
    std::filesystem::path const& path) {
  if (utl::all_of(std::array{elevation_files::kUpDataName,
                             elevation_files::kUpIndexName,
                             elevation_files::kDownDataName,
                             elevation_files::kDownIndexName},
                  [&](char const* const filename) {
                    auto const full_path = path / filename;
                    auto const exists = std::filesystem::exists(full_path);
                    if (!exists) {
                      std::cout << full_path << " does not exist\n";
                    }
                    return exists;
                  })) {
    return std::make_unique<elevation_storage>(path,
                                               cista::mmap::protection::READ);
  }
  return {};
}

std::pair<elevation_t, elevation_t> get_way_elevation(
    preprocessing::elevation::dem_source const& dem,
    point const& from,
    point const& to,
    preprocessing::elevation::step_size const& max_step_size) {
  auto elevation_up = elevation_t{0};
  auto elevation_down = elevation_t{0};
  auto a = dem.get(from);
  auto const b = dem.get(to);
  if (a != NO_ELEVATION_DATA && b != NO_ELEVATION_DATA) {
    // TODO Approximation only for short ways
    // Value should be larger to not skip intermediate values
    auto const steps =
        2 * static_cast<int>(std::max(
                std::ceil(std::abs(to.lat() - from.lat()) / max_step_size.x_),
                std::ceil(std::abs(to.lng() - from.lng()) / max_step_size.y_)));
    if (steps > 0) {
      auto const from_lat = from.lat();
      auto const from_lng = from.lng();
      auto const step_size = preprocessing::elevation::step_size{
          .x_ = (to.lat() - from.lat()) / steps,
          .y_ = (to.lng() - from.lng()) / steps};
      for (auto const mid :
           std::views::iota(1, steps) |
               std::views::transform([&](const auto i) -> point {
                 return point::from_latlng(from_lat + i * step_size.x_,
                                           from_lng + i * step_size.y_);
               })) {
        auto const m = dem.get(mid);
        if (a < m) {
          elevation_up += m - a;
        } else {
          elevation_down += a - m;
        }
        a = m;
      }
    }
    if (a < b) {
      elevation_up += b - a;
    } else {
      elevation_down += a - b;
    }
  }
  return {elevation_up, elevation_down};
}

void elevation_storage::set_elevations(
    ways& w,
    preprocessing::elevation::dem_source const& dem,
    std::shared_ptr<utl::progress_tracker>& pt) {
  pt->in_high(w.n_ways());
  auto const max_step_size = dem.get_step_size();
  auto elevations_up = std::vector<std::vector<elevation_t>>{};
  auto elevations_down = std::vector<std::vector<elevation_t>>{};
  elevations_up.resize(w.n_ways());
  elevations_down.resize(w.n_ways());
  utl::parallel_for_run(
      w.n_ways(),
      [&](auto const way_idx) {
        auto& way_elevations_up = elevations_up[way_idx];
        auto& way_elevations_down = elevations_down[way_idx];
        auto total_elevation_up = elevation_t{0};
        auto total_elevation_down = elevation_t{0};
        for (auto const [from, to] :
             w.r_->way_nodes_[way_idx_t{way_idx}]  //
                 | std::views::transform([&](node_idx_t const node) {
                     return w.get_node_pos(node);
                   })  //
                 | std::views::pairwise) {
          auto const [elevation_up, elevation_down] =
              get_way_elevation(dem, from, to, max_step_size);
          way_elevations_up.push_back(total_elevation_up += elevation_up);
          way_elevations_down.push_back(total_elevation_down += elevation_down);
        }
        if (total_elevation_up == elevation_t{0}) {
          way_elevations_up.clear();
        }
        if (total_elevation_down == elevation_t{0}) {
          way_elevations_down.clear();
        }
      },
      pt->update_fn());
  // Insert elevations
  for (auto& [source, destination] : std::array{
           std::pair{std::ref(elevations_up), std::ref(elevation_up_)},
           std::pair{std::ref(elevations_down), std::ref(elevation_down_)},
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

elevation_t elevation_at(mm_vecvec<way_idx_t, elevation_t> const& elevations,
                         way_idx_t const way_idx,
                         std::uint16_t const idx) {
  return idx == 0U || way_idx >= elevations.size() ||
                 elevations[way_idx].empty()
             ? elevation_t{0}
             : elevations[way_idx].at(idx - 1U);
}

elevation_storage::elevation elevation_storage::get_elevations(
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to) const {
  auto const [f, t] = from <= to ? std::pair{from, to} : std::pair{to, from};
  auto const [up, down] = std::pair<elevation_t, elevation_t>{
      elevation_at(elevation_up_, way, t) - elevation_at(elevation_up_, way, f),
      elevation_at(elevation_down_, way, t) -
          elevation_at(elevation_down_, way, f),
  };
  return from <= to ? elevation{up, down} : elevation{down, up};
}

elevation_storage::elevation get_elevations(elevation_storage const* elevations,
                                            way_idx_t const way,
                                            std::uint16_t const from,
                                            std::uint16_t const to) {
  return elevations == nullptr
             ? elevation_storage::elevation{elevation_t{0}, elevation_t{0}}
             : elevations->get_elevations(way, from, to);
}

}  // namespace osr
