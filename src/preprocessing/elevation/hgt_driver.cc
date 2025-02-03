#include "osr/preprocessing/elevation/hgt_driver.h"

#include <cstddef>
#include <sstream>

#include "utl/verify.h"

namespace fs = std::filesystem;

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

bool hgt_driver::add_tile(fs::path const& path) {
  auto const ext = path.extension().string();
  if (ext != ".hgt") {
    return false;
  }
  auto tile = open(path);
  if (tile.has_value()) {
    auto const box = std::visit(
        [](IsTile auto const& t) { return t.get_coord_box(); }, *tile);
    auto const min = decltype(rtree_)::coord_t{box.min_lat_, box.min_lng_};
    auto const max = decltype(rtree_)::coord_t{box.max_lat_, box.max_lng_};
    rtree_.insert(min, max, static_cast<std::size_t>(tiles_.size()));
    tiles_.emplace_back(std::move(tile.value()));
    return true;
  } else {
    return false;
  }
}

elevation_meters_t hgt_driver::get(point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto elevation = elevation_meters_t::invalid();
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  elevation = std::visit(
                      [&](IsTile auto const& t) -> elevation_meters_t {
                        return t.get(point);
                      },
                      tiles_[tile_idx]);
                  return elevation != elevation_meters_t::invalid();
                });
  return elevation;
}

tile_idx_t hgt_driver::tile_idx(point const& point) const {
  auto const p = decltype(rtree_)::coord_t{static_cast<float>(point.lat()),
                                           static_cast<float>(point.lng())};
  auto idx = tile_idx_t::invalid();
  rtree_.search(p, p,
                [&](auto const&, auto const&, std::size_t const& tile_idx) {
                  idx = std::visit(
                      [&](IsTile auto const& t) -> tile_idx_t {
                        return t.tile_idx(point);
                      },
                      tiles_[tile_idx]);
                  if (idx == tile_idx_t::invalid()) {
                    return false;
                  }
                  idx.tile_idx_ = static_cast<tile_idx_t::data_t>(tile_idx);
                  return true;
                });
  return idx;
}

step_size hgt_driver::get_step_size() const {
  auto steps = step_size{.x_ = std::numeric_limits<double>::quiet_NaN(),
                         .y_ = std::numeric_limits<double>::quiet_NaN()};
  for (auto const& tile : tiles_) {
    auto const s = std::visit(
        [](IsTile auto const& t) { return t.get_step_size(); }, tile);
    if (std::isnan(steps.x_) || s.x_ < steps.x_) {
      steps.x_ = s.x_;
    }
    if (std::isnan(steps.y_) || s.y_ < steps.y_) {
      steps.y_ = s.y_;
    }
  }
  return steps;
}

std::size_t hgt_driver::n_tiles() const { return tiles_.size(); }

std::optional<hgt_driver::hgt_tile_t> hgt_driver::open(fs::path const& path) {
  auto const filename = path.filename();
  auto sw = grid_point{path.filename().string()};
  auto const file_size = fs::file_size(path);
  switch (file_size) {
    case hgt_tile<3601>::file_size():
      return hgt_tile<3601>{path.string(), sw.lat_, sw.lng_};
    case hgt_tile<1201>::file_size():
      return hgt_tile<1201>{path.string(), sw.lat_, sw.lng_};
    default: return {};
  }
}

}  // namespace osr::preprocessing::elevation
