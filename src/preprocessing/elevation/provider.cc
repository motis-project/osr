#include "osr/preprocessing/elevation/provider.h"

#include <filesystem>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "utl/enumerate.h"

#include "cista/strong.h"

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/dem_driver.h"
#include "osr/preprocessing/elevation/hgt_driver.h"
#include "osr/preprocessing/elevation/shared.h"

namespace osr::preprocessing::elevation {

static_assert(IsDriver<dem_driver>);
static_assert(IsDriver<hgt_driver>);

using raster_driver = std::variant<dem_driver, hgt_driver>;

struct provider::impl {
  impl() = default;

  void add_driver(IsDriver auto&& driver) {
    auto const old_count = cusum_drivers_.empty() ? elevation_bucket_idx_t{0}
                                                  : cusum_drivers_.back();
    cusum_drivers_.emplace_back(
        old_count + elevation_bucket_idx_t{driver.n_tiles() * kSubTileFactor});
    drivers_.emplace_back(std::move(driver));
  }

  elevation_t get(::osr::point const& p) {
    for (auto const& grid : drivers_) {
      auto const data = std::visit(
          [&](IsDriver auto const& driver) { return driver.get(p); }, grid);
      if (data != NO_ELEVATION_DATA) {
        return data;
      }
    }
    return NO_ELEVATION_DATA;
  }

  tile_idx_t tile_idx(::osr::point const& p) const {
    for (auto const [driver_idx, driver] : std::views::enumerate(drivers_)) {
      auto idx = std::visit(
          [&](IsDriver auto const& d) { return d.tile_idx(p); }, driver);
      if (idx != tile_idx_t::invalid()) {
        idx.driver_idx_ = static_cast<tile_idx_t::data_t>(driver_idx);
        return idx;
      }
    }
    return tile_idx_t::invalid();
  }

  std::vector<raster_driver> drivers_;
  std::vector<elevation_bucket_idx_t> cusum_drivers_;
};

provider::provider(std::filesystem::path const& p)
    : impl_(std::make_unique<impl>()) {
  if (std::filesystem::is_directory(p)) {
    auto dem = dem_driver{};
    auto hgt = hgt_driver{};
    for (auto const& file : std::filesystem::recursive_directory_iterator(p)) {
      [&]() {
        if (!file.is_regular_file()) {
          return;
        }
        auto const& path = file.path();
        if (dem.add_tile(path)) {
          return;
        }
        if (hgt.add_tile(path)) {
          return;
        }
      }();
    }
    if (dem.n_tiles() > 0U) {
      impl_->add_driver(std::move(dem));
    }
    if (hgt.n_tiles() > 0U) {
      impl_->add_driver(std::move(hgt));
    }
  }
}

provider::~provider() = default;

::osr::elevation_t provider::get(::osr::point const& p) const {
  return impl_->get(p);
}

tile_idx_t provider::tile_idx(::osr::point const& p) const {
  return impl_->tile_idx(p);
}

std::size_t provider::driver_count() const { return impl_->drivers_.size(); }

step_size provider::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& driver : impl_->drivers_) {
    auto const s = std::visit(
        [](IsDriver auto const& d) { return d.get_step_size(); }, driver);
    if (std::isnan(steps.x_) || s.x_ < steps.x_) {
      steps.x_ = s.x_;
    }
    if (std::isnan(steps.y_) || s.y_ < steps.y_) {
      steps.y_ = s.y_;
    }
  }
  return steps;
}

provider::point_idx provider::get_point_idx(::osr::point const& point) const {
  for (auto const [driver_idx, driver] : utl::enumerate(impl_->drivers_)) {
    auto const tile_idx = std::visit(
        [&](IsDriver auto const& d) { return d.get_tile_idx(point); }, driver);
    if (tile_idx != elevation_tile_idx_t::invalid()) {
      return {
          .driver_idx_ =
              elevation_driver_idx_t{static_cast<std::uint8_t>(driver_idx)},
          .tile_idx_ = tile_idx,
      };
    }
  }
  return {
      .driver_idx_ = elevation_driver_idx_t::invalid(),
      .tile_idx_ = elevation_tile_idx_t::invalid(),
  };
}

unsigned int provider::get_bucket_count() const {
  return impl_->cusum_drivers_.empty()
             ? 0U
             : cista::to_idx(impl_->cusum_drivers_.back());
}

elevation_bucket_idx_t provider::get_bucket_idx(
    ::osr::point const& point) const {
  for (auto const [driver_idx, driver] : utl::enumerate(impl_->drivers_)) {
    auto const sub_tile_idx = std::visit(
        [&](IsDriver auto const& d) { return d.get_sub_tile_idx(point); },
        driver);
    if (sub_tile_idx != elevation_tile_idx_t::invalid()) {
      return driver_idx == 0U
                 ? elevation_bucket_idx_t{cista::to_idx(sub_tile_idx)}
                 : impl_->cusum_drivers_[driver_idx - 1U] +
                       cista::to_idx(sub_tile_idx);
    }
  }
  return elevation_bucket_idx_t::invalid();
}

}  // namespace osr::preprocessing::elevation
