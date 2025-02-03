#include "osr/elevation_storage.h"

#include <algorithm>
#include <array>
#include <execution>
#include <mutex>
#include <ranges>
#include <string>
#include <type_traits>

#include "utl/enumerate.h"
#include "utl/pairwise.h"
#include "utl/parallel_for.h"
#include "utl/progress_tracker.h"
#include "utl/raii.h"

#include "cista/strong.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"

constexpr auto const kTimeIt = false;

auto timeit(std::string_view const name, auto&& f, auto&&... args) {
  constexpr auto const returns_value =
      !std::is_same_v<std::invoke_result_t<decltype(f), decltype(args)...>,
                      void>;
  if constexpr (!kTimeIt) {
    if constexpr (returns_value) {

      return f(std::forward<decltype(args)>(args)...);
    } else {

      f(std::forward<decltype(args)>(args)...);
      return;
    }
  }
#include <chrono>
  std::cout << "\nProcessing " << name << " ...\n" << std::endl;
  auto const start = std::chrono::high_resolution_clock::now();
  auto const show_report = [&start]() {
    auto const end = std::chrono::high_resolution_clock::now();
    std::cout << "\nDone: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
              << "\n"
              << std::endl;
  };
  if constexpr (returns_value) {
    auto result = f(std::forward<decltype(args)>(args)...);
    show_report();
    return result;
  } else {
    f(std::forward<decltype(args)>(args)...);
    show_report();
  }
}

namespace osr {

using preprocessing::elevation::elevation_meters_t;
using preprocessing::elevation::provider;
using preprocessing::elevation::step_size;
using preprocessing::elevation::tile_idx_t;

constexpr auto const kWriteMode = cista::mmap::protection::WRITE;

cista::mmap mm(std::filesystem::path const& path,
               cista::mmap::protection const mode) {
  return cista::mmap{path.string().data(), mode};
}

namespace elevation_files {
constexpr auto const kDataName = "elevation_data.bin";
constexpr auto const kIndexName = "elevation_idx.bin";
};  // namespace elevation_files

elevation_storage::elevation_storage(std::filesystem::path const& p,
                                     cista::mmap::protection const mode)
    : elevations_{cista::paged<mm_vec32<encoding>>{mm_vec32<encoding>{
                      mm(p / elevation_files::kDataName, mode)}},
                  mm_vec<cista::page<std::uint32_t, std::uint16_t>>{
                      mm(p / elevation_files::kIndexName, mode)}} {}

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

elevation_storage::elevation get_way_elevation(provider const& provider,
                                               point const& from,
                                               point const& to,
                                               step_size const& max_step_size) {
  auto elevation = elevation_storage::elevation{};
  auto a = provider.get(from);
  auto const b = provider.get(to);
  if (a != elevation_meters_t::invalid() &&
      b != elevation_meters_t::invalid()) {
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
      auto const way_step_size = step_size{.x_ = (to.lat() - from_lat) / steps,
                                           .y_ = (to.lng() - from_lng) / steps};
      for (auto s = 1; s < steps; ++s) {
        auto const m = provider.get(point::from_latlng(
            from_lat + s * way_step_size.x_, from_lng + s * way_step_size.y_));
        if (m != elevation_meters_t::invalid()) {
          if (a < m) {
            elevation.up_ += static_cast<cista::base_t<elevation_monotonic_t>>(
                to_idx(m - a));
          } else {
            elevation.down_ +=
                static_cast<cista::base_t<elevation_monotonic_t>>(
                    to_idx(a - m));
          }
          a = m;
        }
      }
    }
    if (a < b) {
      elevation.up_ +=
          static_cast<cista::base_t<elevation_monotonic_t>>(to_idx(b - a));
    } else {
      elevation.down_ +=
          static_cast<cista::base_t<elevation_monotonic_t>>(to_idx(a - b));
    }
  }
  return elevation;
}

struct way_ordering_t {
  way_idx_t way_idx_;
  tile_idx_t order_;
};

using way_ordering_vec = mm_vec<way_ordering_t>;
way_ordering_vec calculate_way_order(std::vector<std::filesystem::path>& paths,
                                     ways const& w,
                                     provider const& provider,
                                     utl::progress_tracker_ptr& pt) {
  auto const& path = paths.emplace_back(std::filesystem::temp_directory_path() /
                                        "temp_osr_extract_way_ordering");

  auto ordering = mm_vec<way_ordering_t>{mm(path, kWriteMode)};
  ordering.reserve(w.n_ways());

  pt->in_high(w.n_ways());
  for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_)) {
    if (!way.empty()) {
      auto const node_point = w.get_node_pos(way.front());
      auto const tile_idx = provider.tile_idx(node_point);
      if (tile_idx != tile_idx_t::invalid()) {
        ordering.emplace_back(way_idx_t{way_idx}, std::move(tile_idx));
      }
    }
    pt->increment();
  }

  std::sort(
#if __cpp_lib_execution
      std::execution::par_unseq,
#endif
      begin(ordering), end(ordering),
      [](way_ordering_t const& a, way_ordering_t const& b) {
        return a.order_ < b.order_;
      });

  return ordering;
}

mm_vec_map<node_idx_t, point> calculate_points(
    std::vector<std::filesystem::path>& paths,
    ways const& w,
    utl::progress_tracker_ptr& pt) {
  auto const& path = paths.emplace_back(std::filesystem::temp_directory_path() /
                                        "temp_osr_extract_points");
  auto points =
      mm_vec_map<node_idx_t, point>{mm(path, cista::mmap::protection::WRITE)};

  auto const size = w.n_nodes();
  points.resize(size);
  pt->in_high(size);

  utl::parallel_for_run(
      w.n_nodes(),
      [&](std::size_t const& idx) {
        auto const node_idx = node_idx_t{idx};
        if (!w.r_->node_ways_[node_idx].empty()) {
          points[node_idx] = w.get_node_pos(node_idx);
        }
      },
      pt->update_fn());

  return points;
}

using sort_idx_t = cista::strong<std::uint32_t, struct sort_idx_>;

struct mapping_t {
  way_idx_t way_idx_;
  sort_idx_t sort_idx_;
};

struct encoding_result_t {
  osr::mm_vec<mapping_t> mappings_;
  mm_vecvec<sort_idx_t, elevation_storage::encoding> encodings_;
};

encoding_result_t calculate_way_encodings(
    std::vector<std::filesystem::path>& paths,
    ways const& w,
    provider const& provider,
    mm_vec<way_ordering_t> const& ordering,
    mm_vec_map<node_idx_t, point> const& points,
    utl::progress_tracker_ptr& pt) {
  paths.reserve(paths.size() + 3U);
  auto const& mapping_path = paths.emplace_back(
      std::filesystem::temp_directory_path() / "temp_osr_extract_mapping");
  auto const& encoding_data_path =
      paths.emplace_back(std::filesystem::temp_directory_path() /
                         "temp_osr_extract_unordered_encoding_data");
  auto const& encoding_idx_path =
      paths.emplace_back(std::filesystem::temp_directory_path() /
                         "temp_osr_extract_unordered_encoding_idx");

  auto result = encoding_result_t{
      .mappings_ = osr::mm_vec<mapping_t>{mm(mapping_path, kWriteMode)},
      .encodings_ =
          osr::mm_vecvec<sort_idx_t, elevation_storage::encoding>{
              mm_vec<elevation_storage::encoding>{
                  mm(encoding_data_path, kWriteMode)},
              mm_vec<cista::base_t<sort_idx_t>>{
                  mm(encoding_idx_path, kWriteMode)}},
  };
  result.mappings_.reserve(ordering.size());
  auto const max_step_size = provider.get_step_size();
  auto m = std::mutex{};

  pt->in_high(ordering.size());
  utl::parallel_for(
      ordering,
      [&](way_ordering_t const way_ordering) {
        thread_local auto elevations =
            std::vector<elevation_storage::encoding>{};

        auto const& way_idx = way_ordering.way_idx_;

        // Calculate elevations
        auto elevations_idx = std::size_t{0U};
        elevations.clear();
        for (auto const [from, to] : utl::pairwise(w.r_->way_nodes_[way_idx])) {
          auto const elevation = elevation_storage::encoding{get_way_elevation(
              provider, points[from], points[to], max_step_size)};
          if (elevation) {
            elevations.resize(elevations_idx);
            elevations.push_back(std::move(elevation));
          }
          ++elevations_idx;
        }

        // Store elevations unordered
        if (!elevations.empty()) {
          auto const lock = std::lock_guard{m};
          result.mappings_.emplace_back(way_idx,
                                        sort_idx_t{result.encodings_.size()});
          result.encodings_.emplace_back(elevations);
        }
      },
      pt->update_fn());

  return result;
}

void write_ordered_encodings(elevation_storage& storage,
                             encoding_result_t&& result,
                             utl::progress_tracker_ptr& pt) {
  std::sort(
#if __cpp_lib_execution
      std::execution::par_unseq,
#endif
      begin(result.mappings_), end(result.mappings_),
      [](mapping_t const& a, mapping_t const& b) {
        return a.way_idx_ < b.way_idx_;
      });

  pt->in_high(result.mappings_.size());
  for (auto const& mapping : result.mappings_) {
    storage.elevations_.resize(to_idx(mapping.way_idx_) + 1U);
    auto bucket = storage.elevations_.back();
    for (auto const e : result.encodings_[mapping.sort_idx_]) {
      bucket.push_back(e);
    }
    pt->increment();
  }
}

void elevation_storage::set_elevations(ways const& w,
                                       provider const& provider) {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  auto cleanup_paths =
      utl::make_raii(std::vector<std::filesystem::path>{},
                     [](std::vector<std::filesystem::path> const& paths) {
                       for (auto const& path : paths) {
                         std::filesystem::remove(path);
                       }
                     });

  pt->status("Calculating way order").out_bounds(85, 86);
  auto const processing_order = timeit("calculate order", calculate_way_order,
                                       cleanup_paths.get(), w, provider, pt);
  pt->status("Precalculating way points").out_bounds(87, 87);
  auto const points =
      timeit("Calculate points", calculate_points, cleanup_paths.get(), w, pt);
  pt->status("Calculating way elevations").out_bounds(88, 89);
  auto unordered_encodings =
      timeit("calculate order", calculate_way_encodings, cleanup_paths.get(), w,
             provider, processing_order, points, pt);
  pt->status("Storing ordered elevations").out_bounds(90, 90);
  timeit("write ordered", write_ordered_encodings, *this,
         std::move(unordered_encodings), pt);
}

elevation_storage::elevation elevation_storage::get_elevations(
    way_idx_t const way, std::uint16_t const segment) const {
  return (way < elevations_.size() && segment < elevations_[way].size())
             ? elevations_[way][segment].decode()
             : elevation{};
}

elevation_storage::elevation get_elevations(elevation_storage const* elevations,
                                            way_idx_t const way,
                                            std::uint16_t const segment) {
  return elevations == nullptr ? elevation_storage::elevation{}
                               : elevations->get_elevations(way, segment);
}

elevation_storage::elevation& elevation_storage::elevation::operator+=(
    elevation const& other) {
  up_ += to_idx(other.up_);
  down_ += to_idx(other.down_);
  return *this;
}

elevation_storage::elevation elevation_storage::elevation::swap() const {
  return {
      .up_ = down_,
      .down_ = up_,
  };
}

constexpr auto const kCompressedValues = std::array<elevation_monotonic_t, 16>{
    elevation_monotonic_t{0U},  elevation_monotonic_t{1U},
    elevation_monotonic_t{2U},  elevation_monotonic_t{4U},
    elevation_monotonic_t{6U},  elevation_monotonic_t{8U},
    elevation_monotonic_t{11U}, elevation_monotonic_t{14U},
    elevation_monotonic_t{17U}, elevation_monotonic_t{21U},
    elevation_monotonic_t{25U}, elevation_monotonic_t{29U},
    elevation_monotonic_t{34U}, elevation_monotonic_t{38U},
    elevation_monotonic_t{43U}, elevation_monotonic_t{48U}};

elevation_storage::encoding::coding encode(elevation_monotonic_t const e) {
  auto const c = std::ranges::lower_bound(kCompressedValues, e);
  return (c == end(kCompressedValues))
             ? static_cast<elevation_storage::encoding::coding>(
                   kCompressedValues.size() - 1)
             : static_cast<elevation_storage::encoding::coding>(
                   c - begin(kCompressedValues));
}

elevation_monotonic_t decode_helper(
    elevation_storage::encoding::coding const c) {
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
