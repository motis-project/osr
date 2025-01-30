#pragma once

#include <compare>
#include <concepts>
#include <bit>
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

struct tile_idx_t {
  using data_t = std::uint32_t;
  constexpr static auto const kDriverIdxSize = 4;
  constexpr static auto const kTileIdxSize = 16;
  constexpr static auto const kSubTileIdxSize =
      sizeof(data_t) * CHAR_BIT - kDriverIdxSize - kTileIdxSize;

  constexpr static tile_idx_t invalid() {
    return {
        .driver_idx_ = (1 << kDriverIdxSize) - 1,
        .tile_idx_ = (1 << kTileIdxSize) - 1,
        .sub_tile_idx_ = (1 << kSubTileIdxSize) - 1,
    };
  }
  static tile_idx_t from_sub_tile(data_t const& idx) {
    auto t = invalid();
    t.sub_tile_idx_ = idx;
    return t;
  }
  // std::strong_ordering operator<=>(tile_idx_t const& o) const {
  //   return std::bit_cast<data_t>(*this) <=> std::bit_cast<data_t>(o);
  // }
  bool operator==(tile_idx_t const&) const = default;
  bool operator<(tile_idx_t const& o) const {
    return (driver_idx_ < o.driver_idx_) ||
           (driver_idx_ == o.driver_idx_ &&
            ((tile_idx_ < o.tile_idx_) ||
             (tile_idx_ == o.tile_idx_ && sub_tile_idx_ < o.sub_tile_idx_)));
    // return std::bit_cast<data_t>(*this) < std::bit_cast<data_t>(o);
  }

  data_t driver_idx_ : kDriverIdxSize;
  data_t tile_idx_ : kTileIdxSize;
  data_t sub_tile_idx_ : kSubTileIdxSize;
};
static_assert(sizeof(tile_idx_t) == sizeof(tile_idx_t::data_t));

using sub_tile_idx_t = std::uint8_t;
constexpr auto const kSubTileFactor =
    std::numeric_limits<sub_tile_idx_t>::max() + 1U;

template <typename ElevationProvider>
concept IsProvider = requires(ElevationProvider provider) {
  {
    std::as_const(provider).get(std::declval<::osr::point>())
  } -> std::same_as<::osr::elevation_t>;
  {
    std::as_const(provider).tile_idx(std::declval<point const&>())
  } -> std::same_as<tile_idx_t>;
  { std::as_const(provider).get_step_size() } -> std::same_as<step_size>;
};

template <typename Tile>
concept IsTile = IsProvider<Tile> && requires(Tile tile) {
  { std::as_const(tile).get_coord_box() } -> std::same_as<coord_box>;
};

template <typename Driver>
concept IsDriver = IsProvider<Driver> && requires(Driver driver) {
  { std::as_const(driver).n_tiles() } -> std::same_as<std::size_t>;
};

}  // namespace osr::preprocessing::elevation
