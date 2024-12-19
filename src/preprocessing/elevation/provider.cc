#include "osr/preprocessing/elevation/provider.h"

#include <filesystem>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/dem_grid.h"
#include "osr/preprocessing/elevation/hgt_raster.h"
#include "osr/preprocessing/elevation/step_size.h"

namespace osr::preprocessing::elevation {

static_assert(IsRasterDriver<dem_grid>);
static_assert(IsRasterDriver<hgt_raster>);

using raster_driver = std::variant<dem_grid, hgt_raster>;

struct provider::impl {
  impl() = default;

  void add_grid_file(fs::path const& path) {
    drivers.emplace_back(std::in_place_type<dem_grid>, path.string());
  }

  void add_driver(hgt_raster&& driver) {
    drivers.emplace_back(std::move(driver));
  }

  elevation_t get(::osr::point const& p) {
    for (auto const& grid : drivers) {
      auto const data =
          std::visit([&](auto const& driver) { return driver.get(p); }, grid);
      if (data != NO_ELEVATION_DATA) {
        return data;
      }
    }
    return NO_ELEVATION_DATA;
  }

  std::vector<raster_driver> drivers;
};

provider::provider(std::filesystem::path const& p)
    : impl_(std::make_unique<impl>()) {
  if (std::filesystem::is_directory(p)) {
    auto hgt_tiles = std::vector<hgt_raster::hgt_tile>{};
    for (auto const& file : std::filesystem::recursive_directory_iterator(p)) {
      if (file.is_regular_file()) {
        auto const& path = file.path();
        auto const ext = path.extension().string();
        if (ext == ".hdr") {
          impl_->add_grid_file(path);
        } else if (ext == ".hgt") {
          auto tile = hgt_raster::open(path);
          if (tile.has_value()) {
            hgt_tiles.push_back(std::move(tile).value());
          }
        }
      }
    }
    if (!hgt_tiles.empty()) {
      impl_->add_driver(hgt_raster(std::move(hgt_tiles)));
    }
  }
}

provider::~provider() = default;

::osr::elevation_t provider::get(::osr::point const& p) const {
  return impl_->get(p);
}

std::size_t provider::driver_count() const { return impl_->drivers.size(); }

step_size provider::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& driver : impl_->drivers) {
    auto const s =
        std::visit([](auto const& d) { return d.get_step_size(); }, driver);
    if (std::isnan(steps.x_) || s.x_ < steps.x_) {
      steps.x_ = s.x_;
    }
    if (std::isnan(steps.y_) || s.y_ < steps.y_) {
      steps.y_ = s.y_;
    }
  }
  return steps;
}

}  // namespace osr::preprocessing::elevation
