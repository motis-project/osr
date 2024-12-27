#include "osr/preprocessing/elevation/hgt_raster.h"

#include <cstddef>
#include <algorithm>
#include <ranges>
#include <sstream>

#include "utl/verify.h"

namespace osr::preprocessing::elevation {

struct grid_point {
  explicit grid_point(std::string const& filename) {
    auto lat_dir = char{};
    auto lng_dir = char{};
    auto lat = int{};
    auto lng = int{};

    auto s = std::stringstream{filename};
    s >> lat_dir >> lat >> lng_dir >> lng;

    utl::verify(lat_dir == 'S' || lat_dir == 'N', "Invalid direction '{}'",
                lat_dir);
    utl::verify(lng_dir == 'W' || lng_dir == 'E', "Invalid direction '{}'",
                lng_dir);
    utl::verify(-180 <= lng && lng <= 180, "Invalid longitude '{}'", lng);
    utl::verify(-90 <= lat && lat <= 90, "Invalid latitude '{}'", lat);

    lat_ = static_cast<std::int8_t>((lat_dir == 'N') ? lat : -lat);
    lng_ = static_cast<std::int16_t>((lng_dir == 'E') ? lng : -lng);

    utl::verify(-180 <= lng_ && lng_ < 180, "Invalid longitude '{}'", lng);
    utl::verify(-90 <= lat_ && lat_ < 90, "Invalid latitude '{}'", lat);
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
  width_ = static_cast<std::size_t>(lng_range.max - sw_lng_ + 1);
  height_ = static_cast<std::size_t>(lat_range.max - sw_lat_ + 1);

  tiles_.resize(static_cast<std::size_t>(width_ * height_));

  for (auto&& tile : std::move(tiles)) {
    tiles_[get_tile_offset(get_lat(tile), get_lng(tile))].emplace(
        std::move(tile));
  }
}

::osr::elevation_t hgt_raster::get(::osr::point const& point) const {
  auto const offset = get_tile_offset(point.lat(), point.lng());
  return tiles_[offset].has_value()
             ? std::visit(
                   [&](auto const& t) -> ::osr::elevation_t {
                     return t.get(point);
                   },
                   *tiles_[offset])
             : ::osr::elevation_t{};
}

step_size hgt_raster::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& tile : tiles_) {
    if (!tile.has_value()) {
      continue;
    }
    auto const s =
        std::visit([](auto const& t) { return t.get_step_size(); }, *tile);
    if (std::isnan(steps.x_) || s.x_ < steps.x_) {
      steps.x_ = s.x_;
    }
    if (std::isnan(steps.y_) || s.y_ < steps.y_) {
      steps.y_ = s.y_;
    }
  }
  return steps;
}

std::optional<hgt_raster::hgt_tile> hgt_raster::open(fs::path const& path) {
  auto const filename = path.filename();
  auto sw = grid_point{path.filename().string()};
  auto const file_size = fs::file_size(path);
  switch (file_size) {
    case hgt<3601>::file_size():
      return hgt<3601>{path.string(), sw.lat_, sw.lng_};
    case hgt<1201>::file_size():
      return hgt<1201>{path.string(), sw.lat_, sw.lng_};
    default: return {};
  }
}

std::size_t hgt_raster::get_tile_offset(double lat, double lng) const {
  auto const row = std::clamp(
      static_cast<std::size_t>(static_cast<int>(std::floor(lat)) - sw_lat_),
      std::size_t{0U}, height_ - 1);
  auto const column = std::clamp(
      static_cast<std::size_t>(static_cast<int>(std::floor(lng)) - sw_lng_),
      std::size_t{0U}, width_ - 1);
  return static_cast<std::size_t>(row * width_ + column);
}

}  // namespace osr::preprocessing::elevation
