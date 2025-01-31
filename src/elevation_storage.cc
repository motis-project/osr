#include "osr/elevation_storage.h"

#include <algorithm>
#include <array>
#include <execution>
#include <mutex>
#include <ranges>
#include <string>

#include "utl/enumerate.h"
#include "utl/pairwise.h"
#include "utl/parallel_for.h"
#include "utl/progress_tracker.h"

#include "cista/strong.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/shared.h"
#include "osr/preprocessing/elevation/step_size.h"

auto timeit(std::string_view const name, auto&& f, auto&&... args) {
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
  constexpr auto const returns_value =
      !std::is_same_v<std::invoke_result_t<decltype(f), decltype(args)...>,
                      void>;
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

constexpr auto const kWriteMode = cista::mmap::protection::WRITE;

cista::mmap mm(std::filesystem::path const& path,
               cista::mmap::protection const mode) {
  return cista::mmap{path.string().data(), mode};
}

// TODO Remove
cista::mmap mm(std::filesystem::path const& p,
               char const* file,
               cista::mmap::protection const mode) {
  return mm(p / file, mode);
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
// DEBUG EXTRACT START
using sort_idx_t = cista::strong<std::uint32_t, struct sort_idx_>;

struct mapping_t {
  way_idx_t way_idx_;
  sort_idx_t sort_idx_;
};

struct result_t {
  osr::mm_vec<mapping_t> mappings_;
  osr::mm_vecvec<sort_idx_t, elevation_storage::encoding> encodings_;
};

void fill_points(mm_vec_map<node_idx_t, point>& points, ways const& w) {
  points.resize(w.n_nodes());
  // utl::parallel_for(utl::iota(node_idx_t{0U}, w.n_nodes()), [&](node_idx_t
  // const node_idx) {
  utl::parallel_for_run(w.n_nodes(), [&](std::size_t const& idx) {
    auto const node_idx = node_idx_t{idx};
    if (!w.r_->node_ways_[node_idx].empty()) {
      points[node_idx] = w.get_node_pos(node_idx);
    }
  });
  // points.reserve(w.n_nodes());
  // // for (auto const n : std::views::iota(0U, w.n_nodes()) |
  // std::views::transform([](auto const idx) { return node_idx_t{idx}; })) {
  // for (auto node_idx = node_idx_t{0U}; node_idx < node_idx_t{w.n_nodes()};
  // ++node_idx) {
  //   auto const p = w.r_->node_ways_[node_idx].empty() ? point{}
  //   :w.get_node_pos(node_idx);
  //   // std::cout << node_idx << ", ";
  //   // points.emplace_back(w.get_node_pos(node_idx));
  //   points.emplace_back(p);
  // }
}

void calculate_elevations(
    preprocessing::elevation::provider const& provider,
    ways const& w,
    std::vector<elevation_storage::encoding>& elevations,
    mm_vec_map<node_idx_t, point> const& points,
    way_idx_t const way_idx,
    preprocessing::elevation::step_size const& max_step_size) {

  elevations.clear();

  auto const& nodes = w.r_->way_nodes_[way_idx];

  // for (auto const& node : nodes) {
  //   points.emplace_back(w.get_node_pos(node));
  // }
  auto elevations_idx = std::size_t{0U};
  for (auto const [from, to] : utl::pairwise(nodes)) {
    if (from >= points.size() || to >= points.size()) {
      std::cout << "\n\n\nDEBUG: " << from << ", " << to << ", "
                << points.size() << "\n\n\n"
                << std::endl;
    }
    auto const elevation = elevation_storage::encoding{
        get_way_elevation(provider, points[from], points[to], max_step_size)};
    if (elevation) {
      elevations.resize(elevations_idx);
      elevations.push_back(std::move(elevation));
    }
    ++elevations_idx;
  }
}

void store_elevations(result_t& result,
                      way_idx_t const way_idx,
                      auto const& elevations) {
  result.mappings_.emplace_back(way_idx, sort_idx_t{result.encodings_.size()});
  result.encodings_.emplace_back(elevations);
}
// DEBUG EXTRACT START

#include <chrono>

struct way_ordering_t {
  way_idx_t way_idx_;
  preprocessing::elevation::tile_idx_t order_;
};

mm_vec<way_ordering_t> calculate_way_order(
    ways const& w, preprocessing::elevation::provider const& provider) {
  auto const path =
      std::filesystem::temp_directory_path() / "temp_osr_extract_way_ordering";
  // auto ordering = utl::make_raii<osr::mm_vec<way_ordering_t>>({mm(path,
  // kWriteMode)}, [p = std::move(path)](auto* mm){ std::destroy_at(mm); });
  // auto ordering = utl::make_raii(osr::mm_vec<way_ordering_t>{mm(path,
  // kWriteMode)}, [p = std::move(path)](auto* mm){ std::destroy_at(mm); });
  // auto& ord = ordering.get();
  // ord.reserve(w.n_ways());

  auto ordering = mm_vec<way_ordering_t>{mm(path, kWriteMode)};
  ordering.reserve(w.n_ways());

  // for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_) |
  // utl::remove_if([](auto const& enumerated_way) -> bool {
  //   return enumerated_way.second.empty();
  // })) {
  for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_)) {
    if (!way.empty()) {
      auto const node_point = w.get_node_pos(way.front());
      auto const tile_idx = provider.tile_idx(node_point);
      if (tile_idx != preprocessing::elevation::tile_idx_t::invalid()) {
        ordering.emplace_back(way_idx_t{way_idx}, std::move(tile_idx));
      }
    }
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

mm_vec_map<node_idx_t, point> calculate_points(ways const& w) {
  auto points = mm_vec_map<node_idx_t, point>{
      mm(std::filesystem::temp_directory_path(), "temp_osr_extract_points",
         cista::mmap::protection::WRITE)};

  points.resize(w.n_nodes());
  utl::parallel_for_run(w.n_nodes(), [&](std::size_t const& idx) {
    auto const node_idx = node_idx_t{idx};
    if (!w.r_->node_ways_[node_idx].empty()) {
      points[node_idx] = w.get_node_pos(node_idx);
    }
  });

  return points;
}

struct encoding_result_t {
  osr::mm_vec<mapping_t> mappings_;
  mm_vecvec<sort_idx_t, elevation_storage::encoding> encodings_;
};

encoding_result_t calculate_way_encodings(
    ways const& w,
    preprocessing::elevation::provider const& provider,
    mm_vec<way_ordering_t> const& ordering,
    mm_vec_map<node_idx_t, point> const& points) {
  auto const mapping_path =
      std::filesystem::temp_directory_path() / "temp_osr_extract_mapping";
  auto const encoding_data_path = std::filesystem::temp_directory_path() /
                                  "temp_osr_extract_unordered_encoding_data";
  auto const encoding_idx_path = std::filesystem::temp_directory_path() /
                                 "temp_osr_extract_unordered_encoding_idx";

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

  utl::parallel_for(ordering, [&](way_ordering_t const way_ordering) {
    thread_local auto elevations = std::vector<elevation_storage::encoding>{};
    auto const& way_idx = way_ordering.way_idx_;

    calculate_elevations(provider, w, elevations, points, way_idx,
                         max_step_size);

    if (!elevations.empty()) {
      auto const lock = std::lock_guard{m};
      result.mappings_.emplace_back(way_idx,
                                    sort_idx_t{result.encodings_.size()});
      result.encodings_.emplace_back(elevations);
    }
  });

  return result;
}

void write_ordered_encodings(elevation_storage& storage,
                             encoding_result_t&& result) {
  std::sort(
#if __cpp_lib_execution
      std::execution::par_unseq,
#endif
      begin(result.mappings_), end(result.mappings_),
      [](mapping_t const& a, mapping_t const& b) {
        return a.way_idx_ < b.way_idx_;
      });

  for (auto const& mapping : result.mappings_) {
    storage.elevations_.resize(to_idx(mapping.way_idx_) + 1U);
    auto bucket = storage.elevations_.back();
    for (auto const e : result.encodings_[mapping.sort_idx_]) {
      bucket.push_back(e);
    }
  }
}

void elevation_storage::set_elevations(
    ways const& w, preprocessing::elevation::provider const& provider) {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");

  pt->status("Calculating way order").out_bounds(85, 86).in_high(w.n_ways());
  // auto const processing_order = calculate_way_order(w, provider);
  auto const processing_order =
      timeit("calculate order", calculate_way_order, w, provider);
  pt->status("Precalculating way points")
      .out_bounds(87, 87)
      .in_high(w.n_nodes());
  // auto const points = calculate_points(w);
  auto const points = timeit("Calculate points", calculate_points, w);
  pt->status("Calculating way elevations")
      .out_bounds(88, 89)
      .in_high(w.n_ways());
  // auto unordered_encodings =
  //     calculate_way_encodings(w, provider, processing_order, points);
  auto unordered_encodings = timeit("calculate order", calculate_way_encodings,
                                    w, provider, processing_order, points);
  pt->status("Storing ordered elevations")
      .out_bounds(90, 90)
      .in_high(unordered_encodings.mappings_.size());
  // write_ordered_encodings(*this, std::move(unordered_encodings));
  timeit("write ordered", write_ordered_encodings, *this,
         std::move(unordered_encodings));
}

/*
void elevation_storage::set_elevations(
    ways const& w, preprocessing::elevation::provider const& provider) {
  auto const size = w.n_ways();

  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  auto m = std::mutex{};

  auto const t1 = std::chrono::high_resolution_clock::now();
  std::cout << "\nCalculating tile buckets ...\n" << std::endl;

  pt->status("Calculating tile buckets").out_bounds(85, 86).in_high(size);
  struct way_tile_idx_t {
    osr::way_idx_t way_idx_;
    preprocessing::elevation::tile_idx_t tile_idx_;
  };
  // using element_t = cista::pair<osr::way_idx_t, osr::elevation_bucket_idx_t>;
  // auto sorted_ways = osr::mm_vec<element_t>{
  auto sorted_ways = osr::mm_vec<way_tile_idx_t>{
      mm(std::filesystem::temp_directory_path(), "temp_osr_extract_sorted_ways",
         cista::mmap::protection::WRITE)};
  sorted_ways.reserve(size);

  for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_)) {
    if (!way.empty()) {
      auto const p = w.get_node_pos(way.front());
      auto const tile_idx = provider.tile_idx(p);
      if (tile_idx != preprocessing::elevation::tile_idx_t::invalid()) {
        sorted_ways.emplace_back(way_idx_t{way_idx}, tile_idx);
      }
    }
  }

  auto const t2 = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1)
            << "\n\n";
  std::cout << "\nGrouping by tile buckets ...\n" << std::endl;

  pt->status("Grouping by tile buckets")
      .out_bounds(86, 87)
      .in_high(sorted_ways.size());
  std::sort(
#if __cpp_lib_execution
      std::execution::par_unseq,
#endif
      begin(sorted_ways), end(sorted_ways),
      [](way_tile_idx_t const& a, way_tile_idx_t const& b) {
        return a.tile_idx_ < b.tile_idx_;
      });

  auto const t2a = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2a - t2)
            << "   n_nodes: " << w.n_nodes() << "\n\n";
  std::cout << "\nStoring points ...\n" << std::endl;

  auto points = mm_vec_map<node_idx_t, point>{
      mm(std::filesystem::temp_directory_path(), "temp_osr_extract_points",
         cista::mmap::protection::WRITE)};
  fill_points(points, w);

  auto const t3 = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2a)
            << "\n\n";
  std::cout << "\nWriting elevation data ...\n" << std::endl;

  auto const max_step_size = provider.get_step_size();

  auto results = result_t{
      .mappings_ = osr::mm_vec<mapping_t>{mm(
          std::filesystem::temp_directory_path(), "temp_osr_extract_mapping",
          cista::mmap::protection::WRITE)},
      .encodings_ =
          osr::mm_vecvec<sort_idx_t, encoding>{
              mm_vec<encoding>{mm(std::filesystem::temp_directory_path(),
                                  "temp_osr_extract_unsorted_elevations_data",
                                  cista::mmap::protection::WRITE)},
              mm_vec<cista::base_t<sort_idx_t>>{
                  mm(std::filesystem::temp_directory_path(),
                     "temp_osr_extract_unsorted_elevations_idx",
                     cista::mmap::protection::WRITE)}},
  };
  results.mappings_.reserve(sorted_ways.size());
  // unsorted_elevations.bucket_starts_.reserve(way_count);
  // std::cout << "\nSizes: " << size << ", " << sorted_ways.size() << ", " <<
  // way_count << ", " << unsorted_elevations.size() << ", " <<
  // unsorted_elevations.bucket_starts_.size() << "\n\n";

  pt->status("Calculating elevation data")
      .out_bounds(87, 88)
      .in_high(sorted_ways.size());
  utl::parallel_for(
      sorted_ways,
      [&](way_tile_idx_t const& way_bucket_idx) {
        thread_local auto elevations = std::vector<encoding>{};
        // thread_local auto points = std::vector<point>{};

        calculate_elevations(provider, w, elevations, points,
                             way_bucket_idx.way_idx_, max_step_size);

        // elevations.clear();
        // points.clear();

        auto const& way_idx = way_bucket_idx.way_idx_;
        // auto const& nodes = w.r_->way_nodes_[way_idx];

        // for (auto const& node : nodes) {
        //   points.emplace_back(w.get_node_pos(node));
        // }
        // auto elevations_idx = std::size_t{0U};
        // for (auto const [from, to] : utl::pairwise(points)) {
        //   auto const elevation =
        //       encoding{get_way_elevation(provider, from, to, max_step_size)};
        //   if (elevation) {
        //     elevations.resize(elevations_idx);
        //     elevations.push_back(std::move(elevation));
        //   }
        //   ++elevations_idx;
        // }
        if (!elevations.empty()) {
          auto const lock = std::lock_guard{m};
          store_elevations(results, way_idx, elevations);
          // mappings.emplace_back(way_idx,
          //                       sort_idx_t{unsorted_elevations.size()});
          // unsorted_elevations.emplace_back(elevations);
          // auto bucket = elevations_[way_idx];
          // for (auto const e : elevations) {
          //   bucket.push_back(e);
          // }
        }
      },
      pt->update_fn());

  auto const t4 = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3)
            << "\n\n"
            << std::endl;
  std::cout << "\nSorting results ...\n" << std::endl;
  pt->status("Sorting results")
      .out_bounds(88, 89)
      .in_high(results.mappings_.size());

  std::sort(
#if __cpp_lib_execution
      std::execution::par_unseq,
#endif
      begin(results.mappings_), end(results.mappings_),
      [](mapping_t const& a, mapping_t const& b) {
        return a.way_idx_ < b.way_idx_;
      });

  auto const t5 = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4)
            << "\n\n"
            << std::endl;
  std::cout << "\nStoring elevations ...\n" << std::endl;
  pt->status("Storing elevations")
      .out_bounds(89, 90)
      .in_high(results.mappings_.size());
  elevations_.resize(size);

  for (auto const& mapping : results.mappings_) {
    auto bucket = elevations_[mapping.way_idx_];
    for (auto const e : results.encodings_[mapping.sort_idx_]) {
      bucket.push_back(e);
    }
    pt->increment();
  }

  auto const t6 = std::chrono::high_resolution_clock::now();
  std::cout << "\nDone: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5)
            << "\n\n";
}
*/

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
