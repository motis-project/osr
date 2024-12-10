#include "osr/preprocessing/elevation/dem_source.h"

#include <algorithm>
#include <filesystem>
#include <limits>
#include <ranges>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/dem_grid.h"
#include "osr/preprocessing/elevation/hgt.h"
#include "osr/preprocessing/elevation/step_size.h"

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

using raster_driver = std::variant<dem_grid, hgt<3601U>, hgt<1201U>>;

struct dem_source::impl {
  impl() = default;

  void add_grid_file(fs::path const& path) {
    grids_.emplace_back(path.string());
  }

  elevation_t get(::osr::point const& p) {
    for (auto const& grid : grids_) {
      auto const data = grid.get(p);
      if (data != NO_ELEVATION_DATA) {
        return data;
      }
    }
    return NO_ELEVATION_DATA;
  }

  std::vector<dem_grid> grids_;
};

dem_source::dem_source(std::filesystem::path const& p)
    : impl_(std::make_unique<impl>()) {
  if (std::filesystem::is_directory(p)) {
    for (auto const& file : std::filesystem::recursive_directory_iterator(p)) {
      auto const& path = file.path();
      if (file.is_regular_file() && path.extension().string() == ".hdr") {
        impl_->add_grid_file(path);
      }
    }
  }
}

dem_source::~dem_source() = default;

::osr::elevation_t dem_source::get(::osr::point const& p) const {
  return impl_->get(p);
}

std::size_t dem_source::size() const { return impl_->grids_.size(); }

step_size dem_source::get_step_size() const {
  return std::ranges::fold_left_first(
             impl_->grids_ | std::views::transform([](dem_grid const& grid) {
               return grid.get_step_size();
             }),
             [](step_size&& lhs, step_size&& rhs) -> step_size {
               return {
                   .x_ = std::min(lhs.x_, rhs.x_),
                   .y_ = std::min(lhs.y_, rhs.y_),
               };
             })
      .value_or(step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                          .y_ = std::numeric_limits<double>::quiet_NaN()});
}

}  // namespace osr::preprocessing::elevation
