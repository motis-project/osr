#include "osr/preprocessing/elevation/hgt_raster.h"

#include <cstddef>
#include <ranges>
#include <sstream>

#include "osr/elevation_storage.h"

#include "utl/verify.h"

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

hgt_raster::hgt_raster(std::vector<hgt_tile>&& tiles)
    : rtree_{}, tiles_{std::move(tiles)} {
  auto const get_lat = [](hgt_tile const& tile) {
    return std::visit([](auto const& t) { return t.lat(); }, tile);
  };
  auto const get_lng = [](hgt_tile const& tile) {
    return std::visit([](auto const& t) { return t.lng(); }, tile);
  };

  for (auto [idx, tile] : std::views::enumerate(tiles_)) {
    auto const lat = static_cast<float>(get_lat(tile));
    auto const lng = static_cast<float>(get_lng(tile));
    auto const min = decltype(rtree_)::coord_t{lat, lng};
    auto const max = decltype(rtree_)::coord_t{lat + 1.F, lng + 1.F};
    rtree_.insert(min, max, static_cast<std::size_t>(idx));
  }
}

::osr::elevation_t hgt_raster::get(::osr::point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto elevation = NO_ELEVATION_DATA;
  rtree_.search(
      p, p, [&](auto const&, auto const&, std::size_t const& tile_idx) {
        elevation = std::visit(
            [&](auto const& t) -> ::osr::elevation_t { return t.get(point); },
            tiles_[tile_idx]);
        return true;
      });
  return elevation;
}

step_size hgt_raster::get_step_size() const {
  return tiles_.empty()
             ? step_size{}
             : std::visit([](auto const& t) { return t.get_step_size(); },
                          tiles_.front());
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

}  // namespace osr::preprocessing::elevation
