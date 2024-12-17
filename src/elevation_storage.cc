#include "osr/elevation_storage.h"

#include <algorithm>
#include <array>

#include "utl/helpers/algorithm.h"
#include "utl/pairwise.h"
#include "utl/progress_tracker.h"

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
constexpr auto const kDataName = "elevation_data.bin";
constexpr auto const kIndexName = "elevation_idx.bin";
};  // namespace elevation_files

elevation_storage::elevation_storage(std::filesystem::path const& p,
                                     cista::mmap::protection const mode)
    : elevations_{
          mm_vec<compressed_elevation>{mm(p, elevation_files::kDataName, mode)},
          mm_vec<unsigned>(mm(p, elevation_files::kIndexName, mode))} {}

std::unique_ptr<elevation_storage> elevation_storage::try_open(
    std::filesystem::path const& path) {
  if (utl::all_of(
          std::array{elevation_files::kDataName, elevation_files::kIndexName},
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
    ways& w, preprocessing::elevation::provider const& dem) {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Load elevation data").out_bounds(85, 90).in_high(w.n_ways());
  auto const max_step_size = dem.get_step_size();
  auto elevations = std::vector<compressed_elevation>{};
  auto points = std::vector<point>{};
  for (auto nodes : w.r_->way_nodes_) {
    elevations.clear();
    points.clear();
    for (auto const& node : nodes) {
      points.emplace_back(w.get_node_pos(node));
    }
    auto idx = std::size_t{0U};
    for (auto const [from, to] : utl::pairwise(points)) {
      auto const elevation = get_way_elevation(dem, from, to, max_step_size);
      auto const segment_elevation = compressed_elevation{
          .up_ = compressed_elevation::encode(elevation.up_),
          .down_ = compressed_elevation::encode(elevation.down_),
      };
      if (!segment_elevation.is_flat()) {
        elevations.resize(idx);
        elevations.push_back(segment_elevation);
      }
      ++idx;
    }
    elevations_.emplace_back(elevations);

    pt->increment();
  }
}

elevation_storage::elevation elevation_storage::get_elevations(
    way_idx_t const way,
    std::uint16_t const from,
    std::uint16_t const to) const {
  auto const idx = (from < to) ? from : to;
  auto const elev = (way < elevations_.size() && idx < elevations_[way].size())
                        ? elevations_[way][idx]
                        : compressed_elevation{0U, 0U};
  return (from < to)
             ? elevation{.up_ = compressed_elevation::decode(elev.up_),
                         .down_ = compressed_elevation::decode(elev.down_)}
             : elevation{.up_ = compressed_elevation::decode(elev.down_),
                         .down_ = compressed_elevation::decode(elev.up_)};
}

elevation_storage::elevation get_elevations(elevation_storage const* elevations,
                                            way_idx_t const way,
                                            std::uint16_t const from,
                                            std::uint16_t const to) {
  return elevations == nullptr
             ? elevation_storage::elevation{elevation_t{0}, elevation_t{0}}
             : elevations->get_elevations(way, from, to);
}

constexpr auto const kCompressedValues = std::array<elevation_t, 16>{
    // 0, 1, 2, 4, 6, 8, 11, 14, 17, 21, 25, 29, 34, 38, 43, 48};
    0, 1, 2, 3, 4, 5, 6, 7, 17, 21, 25, 29, 34, 38, 43, 48};

elevation_storage::compressed_elevation::coding
elevation_storage::compressed_elevation::encode(elevation_t const e) {
  auto const c = std::ranges::lower_bound(kCompressedValues, e);
  return (c == end(kCompressedValues))
             ? kCompressedValues.size() - 1
             : static_cast<coding>(c - begin(kCompressedValues));
}

elevation_t elevation_storage::compressed_elevation::decode(coding const c) {
  assert(c < kCompressedValues.size());
  return kCompressedValues.at(c);
}

bool elevation_storage::compressed_elevation::is_flat() const {
  return up_ == 0U && down_ == 0U;
}

}  // namespace osr
