#include "osr/preprocessing/elevation/provider.h"

#include <memory>
#include <utility>
#include <vector>

#include "utl/enumerate.h"

#include "osr/elevation_storage.h"
#include "osr/preprocessing/elevation/dem_driver.h"
#include "osr/preprocessing/elevation/hgt_driver.h"

namespace osr::preprocessing::elevation {

static_assert(IsDriver<dem_driver>);
static_assert(IsDriver<hgt_driver>);

using driver_t = std::variant<dem_driver, hgt_driver>;

struct provider::impl {
  impl() = default;

  void add_driver(IsDriver auto&& driver) {
    drivers_.emplace_back(std::move(driver));
  }

  elevation_meters_t get(point const& p) const {
    for (auto const& driver : drivers_) {
      auto const meters =
          std::visit([&](IsDriver auto const& d) { return d.get(p); }, driver);
      if (meters != elevation_meters_t::invalid()) {
        return meters;
      }
    }
    return elevation_meters_t::invalid();
  }

  tile_idx_t tile_idx(point const& p) const {
    for (auto const [driver_idx, driver] : utl::enumerate(drivers_)) {
      auto idx = std::visit(
          [&](IsDriver auto const& d) { return d.tile_idx(p); }, driver);
      if (idx != tile_idx_t::invalid()) {
        idx.driver_idx_ = static_cast<tile_idx_t::data_t>(driver_idx);
        return idx;
      }
    }
    return tile_idx_t::invalid();
  }

  std::vector<driver_t> drivers_;
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

elevation_meters_t provider::get(point const& p) const { return impl_->get(p); }

tile_idx_t provider::tile_idx(point const& p) const {
  return impl_->tile_idx(p);
}

std::size_t provider::driver_count() const { return impl_->drivers_.size(); }

resolution provider::max_resolution() const {
  auto res = resolution{};
  for (auto const& driver : impl_->drivers_) {
    auto const r = std::visit(
        [](IsDriver auto const& d) { return d.max_resolution(); }, driver);
    res.update(r);
  }
  return res;
}

}  // namespace osr::preprocessing::elevation
