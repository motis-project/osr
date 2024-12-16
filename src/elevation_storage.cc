#include "osr/elevation_storage.h"

#include <array>

#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/provider.h"
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
    : elevations_up_{mm_vec<elevation_t>{
                         mm(p, elevation_files::kUpDataName, mode)},
                     mm_vec<unsigned>(
                         mm(p, elevation_files::kUpIndexName, mode))},
      elevations_down_{
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

elevation_storage::elevation get_way_elevation(
    preprocessing::elevation::provider const& dem,
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
    auto const steps = static_cast<int>(std::max(
        std::ceil(2 * std::abs(to.lat() - from.lat()) / max_step_size.x_),
        std::ceil(2 * std::abs(to.lng() - from.lng()) / max_step_size.y_)));
    if (steps > 0) {
      auto const from_lat = from.lat();
      auto const from_lng = from.lng();
      auto const step_size = preprocessing::elevation::step_size{
          .x_ = (to.lat() - from.lat()) / steps,
          .y_ = (to.lng() - from.lng()) / steps};
      for (auto s = 1; s < steps; ++s) {
        auto const m = dem.get(point::from_latlng(from_lat + s * step_size.x_,
                                                  from_lng + s * step_size.y_));
        if (m != NO_ELEVATION_DATA) {
          if (a < m) {
            elevation_up += m - a;
          } else {
            elevation_down += a - m;
          }
          a = m;
        }
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
    preprocessing::elevation::provider const& dem,
    std::shared_ptr<utl::progress_tracker>& pt) {
  pt->in_high(w.n_ways());
  auto const max_step_size = dem.get_step_size();
  auto elevations_up = std::vector<elevation_t>{};
  auto elevations_down = std::vector<elevation_t>{};
  auto points = std::vector<point>{};
  for (auto nodes : w.r_->way_nodes_) {
    elevations_up.clear();
    elevations_down.clear();
    points.clear();
    for (auto const& node : nodes) {
      points.emplace_back(w.get_node_pos(node));
    }
    auto has_elevation_up = false;
    auto has_elevation_down = false;
    for (auto const [from, to] : utl::pairwise(points)) {
      auto const elevation = get_way_elevation(dem, from, to, max_step_size);
      elevations_up.push_back(elevation.up_);
      elevations_down.push_back(elevation.down_);
      if (elevation.up_ > elevation_t{0}) {
        has_elevation_up = true;
      }
      if (elevation.down_ > elevation_t{0}) {
        has_elevation_down = true;
      }
    }
    if (!has_elevation_up) {
      elevations_up.clear();
    }
    if (!has_elevation_down) {
      elevations_down.clear();
    }
    elevations_up_.emplace_back(elevations_up);
    elevations_down_.emplace_back(elevations_down);

    pt->increment();
  }
}

elevation_t elevation_at(mm_vecvec<way_idx_t, elevation_t> const& elevations,
                         way_idx_t const way_idx,
                         std::uint16_t const idx) {
  return way_idx < elevations.size() && idx < elevations[way_idx].size()
             ? elevations[way_idx].at(idx)
             : elevation_t{0};
}

elevation_storage::elevation elevation_storage::get_elevations(
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to) const {
  return (from < to) ? elevation{elevation_at(elevations_up_, way, from),
                                 elevation_at(elevations_down_, way, from)}
                     : elevation{elevation_at(elevations_down_, way, to),
                                 elevation_at(elevations_up_, way, to)};
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
