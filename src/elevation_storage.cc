#include "osr/elevation_storage.h"

#include <cmath>
#include <algorithm>
#include <array>
#include <filesystem>
#include <mutex>
#include <ranges>
#include <string>
#include <vector>

#include "osr/preprocessing/elevation/shared.h"
#include "osr/types.h"
#include "utl/enumerate.h"
#include "utl/pairwise.h"
#include "utl/parallel_for.h"
#include "utl/progress_tracker.h"
#include "utl/raii.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/provider.h"
#include "osr/preprocessing/elevation/resolution.h"

namespace ev = osr::preprocessing::elevation;
namespace fs = std::filesystem;

namespace osr {

namespace elevation_files {
constexpr auto const kDataName = "elevation_data.bin";
constexpr auto const kIndexName = "elevation_idx.bin";
constexpr auto const kOffsetIndexName = "elevation_offset_idx.bin";
};  // namespace elevation_files

using path_vec = std::vector<fs::path>;
using sort_idx_t = cista::strong<std::uint32_t, struct sort_idx_>;
using node_point_map = mm_vec_map<node_idx_t, point>;

struct way_ordering_t {
  way_idx_t way_idx_;
  ev::tile_idx_t order_;
};

struct mapping_t {
  way_idx_t way_idx_;
  sort_idx_t sort_idx_;
};

struct encoding_result_t {
  mm_vec<mapping_t> mappings_;
  elevation_storage::data_vec_t elevations_;
  mm_vecvec<sort_idx_t, elevation_storage::offset_t> offsets_;
};

using way_ordering_vec = mm_vec<way_ordering_t>;

constexpr auto const kWriteMode = cista::mmap::protection::WRITE;

cista::mmap mm(fs::path const& path, cista::mmap::protection const mode) {
  return cista::mmap{path.string().data(), mode};
}

elevation_storage::elevation_storage(fs::path const& p,
                                     cista::mmap::protection const mode)
    : elevations_{mm(p / elevation_files::kDataName, mode)},
      offsets_{mm_vec<offset_t>{mm(p / elevation_files::kIndexName, mode)},
               mm_vec<cista::base_t<way_idx_t>>{
                   mm(p / elevation_files::kOffsetIndexName, mode)}} {}

std::unique_ptr<elevation_storage> elevation_storage::try_open(
    fs::path const& path) {
  for (auto const& filename :
       {elevation_files::kDataName, elevation_files::kIndexName}) {
    auto const full_path = path / filename;
    if (!fs::exists(full_path)) {
      std::clog << "Elevation file " << full_path << " does not exist\n";
      return nullptr;
    }
  }
  return std::make_unique<elevation_storage>(path,
                                             cista::mmap::protection::READ);
}

std::vector<ev::elevation_meters_t> get_way_elevation(
    ev::provider const& provider,
    point const from,
    point const to,
    ev::resolution const& res,
    ev::elevation_meters_t min_elevation,
    double min_rel_elevation) {
  auto a = provider.get(from);
  auto const b = provider.get(to);
  if (a == ev::elevation_meters_t::invalid() ||
      b == ev::elevation_meters_t::invalid()) {
    return {};
  }
  auto elevation = std::vector<ev::elevation_meters_t>{};

  // TODO Approximation only for short ways
  // Use slightly larger value to not skip intermediate values
  constexpr auto const kSafetyFactor = 1.000001;
  // map longitude in [-360, 360] to [-180, 180]
  auto const adjust_lng = [](double const x) {
    if (x < -180.) {
      return x + 360.;
    } else if (x <= 180.) {
      return x;
    } else {
      return x - 360.;
    }
  };
  auto const from_lat = from.lat();
  auto const from_lng = from.lng();
  auto const min_lng_diff = adjust_lng(to.lng() - from_lng);
  auto const lat_diff = to.lat() - from_lat;
  auto const steps = static_cast<unsigned>(
      std::max(std::ceil(kSafetyFactor * std::abs(lat_diff) / res.y_),
               std::ceil(kSafetyFactor * std::abs(min_lng_diff) / res.x_)));

  auto step_index = std::vector<unsigned>{};
  unsigned n = 2;
  elevation.push_back(a);
  step_index.push_back(0);

  if (steps <= 1) {
    // TODO
  }

  elevation.push_back(b);
  step_index.push_back(steps - 1);

  // only call with at least 2 points, never with s = steps - 1
  auto const evaluate = [&](auto const& s) {
    auto const i1 = static_cast<std::size_t>(
        std::distance(begin(step_index),
                      std::lower_bound(begin(step_index), end(step_index), s)));
    auto const i0 = i1 - 1;

    auto const y0 = elevation.at(i0);
    auto const y1 = elevation.at(i1);
    auto const x0 = step_index.at(i0);
    auto const x1 = step_index.at(i1);
    auto const e_estimate = ev::elevation_meters_t{s - x0} * (y1 - y0) /
                                ev::elevation_meters_t{x1 - x0} +
                            y0;
    return e_estimate;
  };

  auto const way_dist =
      std::sqrt(std::pow(min_lng_diff, 2) + std::pow(lat_diff, 2));
  auto const is_significant = [&](auto const& estimate, auto const& value) {
    auto const abs_diff =
        (estimate < value) ? value - estimate : estimate - value;
    auto const rel_diff = cista::to_idx(abs_diff) / way_dist;

    return abs_diff > min_elevation || rel_diff > min_rel_elevation;
  };

  auto const way_dir =
      ev::resolution{.x_ = min_lng_diff / steps, .y_ = lat_diff / steps};

  auto const get = [&](auto const s) {
    return provider.get(
        {from_lat + s * way_dir.y_, adjust_lng(from_lng + s * way_dir.x_)});
  };

  auto stop = false;
  while (n <= 32) {
    stop = true;
    for (auto s = unsigned{1}; s < steps - 1; ++s) {
      auto const m = get(s);

      if (m == ev::elevation_meters_t::invalid() ||
          !is_significant(evaluate(s), m)) {
        continue;
      }
      stop = false;
      break;
    }

    if (stop) {
      break;
    }

    step_index.insert(--end(step_index), steps * (n - 1) / n);
    elevation.insert(--end(elevation), get(steps * (n - 1) / n));
    for (auto i = unsigned{1}; i < n - 1; ++i) {
      auto const new_s = steps * i / n;
      step_index[i] = new_s;
      elevation[i] = get(new_s);
    }
    n++;
  }

  if (elevation.size() == 2 &&
      !is_significant(a, b)) {  // flat way -> single point
    elevation.resize(1);
  }

  return elevation;
}

node_point_map calculate_points(path_vec& paths,
                                ways const& w,
                                utl::progress_tracker_ptr& pt) {
  auto const& path =
      paths.emplace_back(fs::temp_directory_path() / "temp_osr_extract_points");
  auto points = node_point_map{mm(path, cista::mmap::protection::WRITE)};

  auto const size = w.n_nodes();
  points.resize(size);
  pt->in_high(size);

  utl::parallel_for_run(
      size,
      [&](std::size_t const& idx) {
        auto const node_idx = node_idx_t{idx};
        if (!w.r_->node_ways_[node_idx].empty()) {
          points[node_idx] = w.get_node_pos(node_idx);
        }
      },
      pt->update_fn());

  return points;
}

way_ordering_vec calculate_way_order(path_vec& paths,
                                     ways const& w,
                                     node_point_map const& points,
                                     ev::provider const& provider,
                                     utl::progress_tracker_ptr& pt) {
  auto const& path = paths.emplace_back(fs::temp_directory_path() /
                                        "temp_osr_extract_way_ordering");
  auto ordering = mm_vec<way_ordering_t>{mm(path, kWriteMode)};

  auto const size = w.n_ways();
  ordering.reserve(size);
  pt->in_high(size);

  for (auto const [way_idx, way] : utl::enumerate(w.r_->way_nodes_)) {
    if (!way.empty()) {
      auto const node_point = points[way.front()];
      auto const tile_idx = provider.tile_idx(node_point);
      if (tile_idx != ev::tile_idx_t::invalid()) {
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

encoding_result_t calculate_way_encodings(
    path_vec& paths,
    ways const& w,
    ev::provider const& provider,
    mm_vec<way_ordering_t> const& ordering,
    node_point_map const& points,
    utl::progress_tracker_ptr& pt,
    preprocessing::elevation::elevation_meters_t min_elevation,
    double min_rel_elevation) {
  paths.reserve(paths.size() + 3U);
  auto const& mapping_path = paths.emplace_back(fs::temp_directory_path() /
                                                "temp_osr_extract_mapping");
  auto const& encoding_data_path = paths.emplace_back(
      fs::temp_directory_path() / "temp_osr_extract_unordered_elevation_data");
  auto const& encoding_idx_path = paths.emplace_back(
      fs::temp_directory_path() / "temp_osr_extract_unordered_elevation_idx");
  auto const& encoding_offset_path =
      paths.emplace_back(fs::temp_directory_path() /
                         "temp_osr_extract_unordered_elevation_offset_idx");

  auto result = encoding_result_t{
      .mappings_ = mm_vec<mapping_t>{mm(mapping_path, kWriteMode)},
      .elevations_ =
          elevation_storage::data_vec_t{mm(encoding_data_path, kWriteMode)},
      .offsets_ =
          mm_vecvec<sort_idx_t, elevation_storage::offset_t>{
              mm_vec<cista::base_t<elevation_storage::offset_t>>{
                  mm(encoding_idx_path, kWriteMode)},
              mm_vec<cista::base_t<sort_idx_t>>{
                  mm(encoding_offset_path, kWriteMode)}},

  };
  result.mappings_.reserve(ordering.size());

  auto const res = provider.max_resolution();
  auto m = std::mutex{};

  pt->in_high(ordering.size());

  utl::parallel_for(
      ordering,
      [&](way_ordering_t const way_ordering) {
        thread_local auto elevations = std::vector<ev::elevation_meters_t>{};
        thread_local auto offsets = std::vector<elevation_storage::offset_t>{};

        auto const& way_idx = way_ordering.way_idx_;

        // Calculate elevations
        elevations.clear();
        offsets.clear();
        auto elevations_idx = std::size_t{0};
        for (auto const [from, to] : utl::pairwise(w.r_->way_nodes_[way_idx])) {
          auto const segment_elevation =
              get_way_elevation(provider, points[from], points[to], res,
                                min_elevation, min_rel_elevation);

          offsets.resize(elevations_idx);
          if (!segment_elevation.empty()) {
            auto& offset = offsets.emplace_back(
                elevations_idx, segment_elevation.size(),
                elevation_monotonic_t{0}, elevation_monotonic_t{0});
            if (segment_elevation.size() > 1) {
              auto prev = segment_elevation[0];
              for (auto it = ++begin(segment_elevation);
                   it != end(segment_elevation); ++it) {
                if (*it < prev) {
                  offset.down_ +=
                      static_cast<cista::base_t<elevation_monotonic_t>>(
                          to_idx(prev - (*it)));
                } else {
                  offset.up_ +=
                      static_cast<cista::base_t<elevation_monotonic_t>>(
                          to_idx((*it) - prev));
                }

                prev = *it;
              }
            }
            elevations.insert(end(elevations), begin(segment_elevation),
                              end(segment_elevation));
          }
          ++elevations_idx;
        }

        // Store elevations unordered
        if (!elevations.empty()) {
          auto const lock = std::lock_guard{m};
          result.mappings_.emplace_back(way_idx,
                                        sort_idx_t{result.offsets_.size()});
          offsets[0].idx_ = result.elevations_.size();
          result.offsets_.emplace_back(offsets);
          result.elevations_.insert(end(result.elevations_), begin(elevations),
                                    end(elevations));
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
    storage.offsets_.resize(to_idx(mapping.way_idx_) + 1U);
    auto bucket = storage.offsets_.back();
    for (auto const offset : result.offsets_[mapping.sort_idx_]) {
      if (offset.valid()) {
        bucket.push_back(
            elevation_storage::offset_t{storage.elevations_.size(), offset.n_});
        for (auto i = offset.idx_; i < offset.idx_ + offset.n_; ++i) {
          storage.elevations_.push_back(result.elevations_[i]);
        }
      } else {
        bucket.push_back(elevation_storage::offset_t{});
      }
    }
    pt->increment();
  }
}

void elevation_storage::set_elevations(
    ways const& w,
    ev::provider const& provider,
    preprocessing::elevation::elevation_meters_t min_elevation,
    double min_rel_elevation) {
  auto pt = utl::get_active_progress_tracker_or_activate("osr");
  auto cleanup_paths = utl::make_raii(path_vec{}, [](path_vec const& paths) {
    auto e = std::error_code{};
    for (auto const& path : paths) {
      fs::remove(path, e);
    }
  });

  pt->status("Precalculating way points").out_bounds(75, 77);
  auto const points = calculate_points(cleanup_paths.get(), w, pt);
  pt->status("Calculating way order").out_bounds(77, 81);
  auto const processing_order =
      calculate_way_order(cleanup_paths.get(), w, points, provider, pt);
  pt->status("Calculating way elevations").out_bounds(81, 89);
  auto unordered_encodings = calculate_way_encodings(
      cleanup_paths.get(), w, provider, processing_order, points, pt,
      min_elevation, min_rel_elevation);
  pt->status("Storing ordered elevations").out_bounds(89, 90);
  write_ordered_encodings(*this, std::move(unordered_encodings), pt);
}

std::vector<ev::elevation_meters_t> elevation_storage::get_elevations(
    way_idx_t const way, std::uint16_t const segment) const {
  if (way >= elevations_.size() || segment >= offsets_[way].size()) {
    return {};
  }

  auto const offset = offsets_[way][segment];
  return {std::next(begin(elevations_), offset.idx_),
          std::next(begin(elevations_), offset.idx_ + offset.n_)};
}

std::vector<ev::elevation_meters_t> get_elevations(
    elevation_storage const* elevations,
    way_idx_t const way,
    std::uint16_t const segment) {
  if (elevations == nullptr) {
    return {};
  }

  return elevations->get_elevations(way, segment);
}

}  // namespace osr
