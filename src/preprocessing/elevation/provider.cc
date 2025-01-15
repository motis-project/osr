#include "osr/preprocessing/elevation/provider.h"

#include <filesystem>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

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

  void add_driver(dem_driver&& driver) {
    drivers_.emplace_back(std::move(driver));
  }

  void add_driver(hgt_driver&& driver) {
    drivers_.emplace_back(std::move(driver));
  }

  elevation_t get(::osr::point const& p) {
    for (auto const& grid : drivers_) {
      auto const data =
          std::visit([&](auto const& driver) { return driver.get(p); }, grid);
      if (data != NO_ELEVATION_DATA) {
        return data;
      }
    }
    return NO_ELEVATION_DATA;
  }

  std::vector<raster_driver> drivers_;
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

std::size_t provider::driver_count() const { return impl_->drivers_.size(); }

step_size provider::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& driver : impl_->drivers_) {
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
