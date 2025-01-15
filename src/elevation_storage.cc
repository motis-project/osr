#include "osr/elevation_storage.h"

#include <algorithm>
#include <array>
#include <execution>
#include <mutex>
#include <ranges>

#include "utl/enumerate.h"
#include "utl/pairwise.h"
#include "utl/parallel_for.h"
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
    : elevations_{cista::paged<mm_vec32<encoding>>{mm_vec32<encoding>{
                      mm(p, elevation_files::kDataName, mode)}},
                  mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
                      mm(p, elevation_files::kIndexName, mode)}} {}

std::unique_ptr<elevation_storage> elevation_storage::try_open(
    std::filesystem::path const& path) {
  for (auto const& filename :
       {elevation_files::kDataName, elevation_files::kIndexName}) {
    auto const full_path = path / filename;
    if (!std::filesystem::exists(full_path)) {
      std::cout << full_path << " does not exist\n";
      return nullptr;
    }
  }
  return std::make_unique<elevation_storage>(path,
                                             cista::mmap::protection::READ);
}

elevation_storage::elevation get_way_elevation(
    preprocessing::elevation::provider const& provider,
    point const& from,
    point const& to,
    preprocessing::elevation::step_size const& max_step_size) {
  auto elevation = elevation_storage::elevation{};
  auto a = provider.get(from);
  auto const b = provider.get(to);
  if (a != NO_ELEVATION_DATA && b != NO_ELEVATION_DATA) {
    // TODO Approximation only for short ways
    // Use slightly larger value to not skip intermediate values
    constexpr auto const kSafetyFactor = 1.000001;
    auto const min_lng_diff =
        std::abs(180 - std::abs((to.lng() - from.lng()) - 180));
    auto const lat_diff = std::abs(to.lat() - from.lat());
    auto const steps = static_cast<int>(
        std::max(std::ceil(kSafetyFactor * lat_diff / max_step_size.y_),
                 std::ceil(kSafetyFactor * min_lng_diff / max_step_size.x_)));
    if (steps > 1) {
      auto const from_lat = from.lat();
      auto const from_lng = from.lng();
      auto const step_size = preprocessing::elevation::step_size{
          .x_ = (to.lat() - from_lat) / steps,
          .y_ = (to.lng() - from_lng) / steps};
      for (auto s = 1; s < steps; ++s) {
        auto const m = provider.get(point::from_latlng(
            from_lat + s * step_size.x_, from_lng + s * step_size.y_));
        if (m != NO_ELEVATION_DATA) {
          if (a < m) {
            elevation.up_ += m - a;
          } else {
            elevation.down_ += a - m;
          }
          a = m;
        }
      }
    }
    if (a < b) {
      elevation.up_ += b - a;
    } else {
      elevation.down_ += a - b;
    }
  }
  return elevation;
}

void elevation_storage::set_elevations(
    ways const& w, preprocessing::elevation::provider const& provider) {
  auto const size = w.n_ways();

  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  pt->status("Load elevation data").out_bounds(85, 90).in_high(size);

  elevations_.resize(size);
  auto const max_step_size = provider.get_step_size();
  auto m = std::mutex{};

  auto sorted_way_indices = std::vector<
      std::pair<preprocessing::elevation::provider::point_idx, way_idx_t>>{};
  sorted_way_indices.reserve(size);
  for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_)) {
    auto const p = w.get_node_pos(way.front());
    auto const start_idx = provider.get_point_idx(p);
    sorted_way_indices.emplace_back(
        std::pair{std::move(start_idx), way_idx_t{way_idx}});
  }
  std::sort(std::execution::par_unseq, begin(sorted_way_indices),
            end(sorted_way_indices), [](auto const& lhs, auto const& rhs) {
              return (lhs.first < rhs.first) ||
                     ((lhs.first == rhs.first) && (lhs.second < rhs.second));
            });

  utl::parallel_for_run(
      std::move(size),
      [&](std::size_t const idx) {
        thread_local auto elevations = std::vector<encoding>{};
        thread_local auto points = std::vector<point>{};
        elevations.clear();
        points.clear();

        auto const way_idx = sorted_way_indices[idx].second;
        auto const& nodes = w.r_->way_nodes_[way_idx];

        for (auto const& node : nodes) {
          points.emplace_back(w.get_node_pos(node));
        }
        auto elevations_idx = std::size_t{0U};
        for (auto const [from, to] : utl::pairwise(points)) {
          auto const elevation =
              encoding{get_way_elevation(provider, from, to, max_step_size)};
          if (elevation) {
            elevations.resize(elevations_idx);
            elevations.push_back(std::move(elevation));
          }
          ++elevations_idx;
        }
        {
          auto const guard = std::lock_guard{m};
          auto bucket = elevations_[way_idx];
          for (auto const e : elevations) {
            bucket.push_back(e);
          }
        }
      },
      pt->update_fn());
}

elevation_storage::elevation elevation_storage::get_elevations(
    way_idx_t const way, std::uint16_t const segment) const {
  return (way < elevations_.size() && segment < elevations_[way].size())
             ? elevations_[way][segment].decode()
             : elevation{0U, 0U};
}

elevation_storage::elevation get_elevations(elevation_storage const* elevations,
                                            way_idx_t const way,
                                            std::uint16_t const segment) {
  return elevations == nullptr
             ? elevation_storage::elevation{elevation_t{0}, elevation_t{0}}
             : elevations->get_elevations(way, segment);
}

elevation_storage::elevation& elevation_storage::elevation::operator+=(
    elevation const& other) {
  up_ += other.up_;
  down_ += other.down_;
  return *this;
}

elevation_storage::elevation elevation_storage::elevation::swap() const {
  return {
      .up_ = down_,
      .down_ = up_,
  };
}

constexpr auto const kCompressedValues = std::array<elevation_t, 16>{
    0, 1, 2, 4, 6, 8, 11, 14, 17, 21, 25, 29, 34, 38, 43, 48};

elevation_storage::encoding::coding encode(elevation_t const e) {
  auto const c = std::ranges::lower_bound(kCompressedValues, e);
  return (c == end(kCompressedValues))
             ? static_cast<elevation_storage::encoding::coding>(
                   kCompressedValues.size() - 1)
             : static_cast<elevation_storage::encoding::coding>(
                   c - begin(kCompressedValues));
}

elevation_t decode_helper(elevation_storage::encoding::coding const c) {
  assert(c < kCompressedValues.size());
  return kCompressedValues.at(c);
}

elevation_storage::encoding::encoding(elevation const& e)
    : up_{encode(e.up_)}, down_{encode(e.down_)} {}

elevation_storage::elevation elevation_storage::encoding::decode() const {
  return {
      .up_ = decode_helper(up_),
      .down_ = decode_helper(down_),
  };
}

elevation_storage::encoding::operator bool() const {
  return up_ != 0U || down_ != 0U;
}

}  // namespace osr
