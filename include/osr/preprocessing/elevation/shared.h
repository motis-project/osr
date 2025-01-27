#pragma once

#include <concepts>
#include <limits>
#include <utility>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

struct coord_box {
  float min_lat_;
  float min_lng_;
  float max_lat_;
  float max_lng_;
};

using sub_tile_idx_t = std::uint8_t;
constexpr auto const kSubTileFactor =
    std::numeric_limits<sub_tile_idx_t>::max() + 1U;

template <typename ElevationProvider>
concept IsProvider = requires(ElevationProvider provider) {
  {
    std::as_const(provider).get(std::declval<::osr::point>())
  } -> std::same_as<::osr::elevation_t>;
  { std::as_const(provider).get_step_size() } -> std::same_as<step_size>;
};

template <typename Tile>
concept IsTile = IsProvider<Tile> && requires(Tile tile) {
  { std::as_const(tile).get_coord_box() } -> std::same_as<coord_box>;
  {
    std::as_const(tile).get_sub_tile_idx(std::declval<point const&>())
  } -> std::same_as<sub_tile_idx_t>;
};

template <typename Driver>
concept IsDriver = IsProvider<Driver> && requires(Driver driver) {
  {
    std::as_const(driver).get_tile_idx(std::declval<point const&>())
  } -> std::same_as<elevation_tile_idx_t>;
  {
    std::as_const(driver).get_sub_tile_idx(std::declval<point const&>())
  } -> std::same_as<elevation_tile_idx_t>;
  { std::as_const(driver).n_tiles() } -> std::same_as<std::size_t>;
};

}  // namespace osr::preprocessing::elevation
