#include "osr/preprocessing/elevation/hgt_raster.h"

#include <cstddef>
#include <sstream>

#include "utl/verify.h"

#include <algorithm>
#include <ranges>

namespace osr::preprocessing::elevation {

struct grid_point {
  explicit grid_point(std::string filename) {
    auto lat_dir = char{};
    auto lng_dir = char{};
    auto lat = int{};
    auto lng = int{};

    auto s = std::stringstream{std::move(filename)};
    s >> lat_dir >> lat >> lng_dir >> lng;

    utl::verify(lat_dir == 'S' || lat_dir == 'N', "Invalid direction '{}'",
                lat_dir);
    utl::verify(lng_dir == 'W' || lng_dir == 'E', "Invalid direction '{}'",
                lng_dir);
    utl::verify(-180 <= lng && lng < 180, "Invalid longitude '{}'", lng);
    utl::verify(-90 <= lat && lat < 90, "Invalid latitude '{}'", lat);

    lat_ = static_cast<std::int8_t>((lat_dir == 'N') ? lat : -lat);
    lng_ = static_cast<std::int16_t>((lng_dir == 'E') ? lng : -lng);
  }

  std::int8_t lat_;
  std::int16_t lng_;
};

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

::osr::elevation_t hgt_raster::get(::osr::point const& point) const {
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

std::optional<hgt_raster::hgt_tile> hgt_raster::open(fs::path const& path) {
  auto const filename = path.filename();
  auto sw = grid_point{path.filename()};
  auto const file_size = fs::file_size(path);
  switch (file_size) {
    case hgt<3601>::file_size():
      return hgt<3601>{path.string(), sw.lat_, sw.lng_};
    case hgt<1201>::file_size():
      return hgt<1201>{path.string(), sw.lat_, sw.lng_};
    default: return {};
  }
}

std::size_t hgt_raster::get_tile_offset(int lat, int lng) const {
  return static_cast<std::size_t>((lat - sw_lat_) * width_ + (lng - sw_lng_));
}

}  // namespace osr::preprocessing::elevation
