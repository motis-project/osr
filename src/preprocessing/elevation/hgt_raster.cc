#include "osr/preprocessing/elevation/hgt_raster.h"
#include <cstddef>

#include <algorithm>
#include <ranges>

namespace osr::preprocessing::elevation {

hgt_raster::hgt_raster(std::vector<hgt_tile>&& tiles) : tiles_{} {
  if (tiles.empty()) {
    return;
  }

  auto const get_lat = [](hgt_tile const& tile) {
    return std::visit([](auto const& t) { return t.lat(); }, tile);
  };
  auto const get_lng = [](hgt_tile const& tile) {
    return std::visit([](auto const& t) { return t.lng(); }, tile);
  };

  auto const lat_range = std::ranges::minmax(
      tiles | std::views::transform(
                  [&](hgt_tile const& tile) { return get_lat(tile); }));
  auto const lng_range = std::ranges::minmax(
      tiles | std::views::transform(
                  [&](hgt_tile const& tile) { return get_lng(tile); }));

  sw_lat_ = lat_range.min;
  sw_lng_ = lng_range.min;
  ne_lat_ = lat_range.max + 1;
  ne_lng_ = lng_range.max + 1;
  width_ = ne_lng_ - sw_lng_;

  tiles_.resize(static_cast<std::size_t>(ne_lat_ - sw_lat_) *
                static_cast<std::size_t>(ne_lng_ - sw_lng_));

  for (auto&& tile : std::move(tiles)) {
    tiles_[get_tile_offset(get_lat(tile), get_lng(tile))].emplace(
        std::move(tile));
  }
}

::osr::elevation_t hgt_raster::get(::osr::point const& point) {
  auto const offset = get_tile_offset(point.lat(), point.lng());
  assert(offset < tiles_.size());
  return tiles_[offset].has_value()
             ? std::visit(
                   [&](auto const& t) -> ::osr::elevation_t {
                     return t.get(point);
                   },
                   *tiles_[offset])
             : ::osr::elevation_t{};
}

step_size hgt_raster::get_step_size() const {
  for (auto const& tile : tiles_) {
    if (tile.has_value()) {
      return std::visit([](auto const& t) { return t.get_step_size(); }, *tile);
    }
  }
  return {};
}

std::optional<hgt_raster::hgt_tile> hgt_raster::open(fs::path const&) {
  return {};
}

std::size_t hgt_raster::get_tile_offset(int lat, int lng) const {
  return static_cast<std::size_t>((lat - sw_lat_) * width_ + (lng - sw_lng_));
}

}  // namespace osr::preprocessing::elevation
