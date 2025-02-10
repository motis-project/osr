#pragma once

#include <compare>
#include <concepts>

#include "cista/strong.h"

#include "geo/box.h"

#include "osr/point.h"
#include "osr/preprocessing/elevation/resolution.h"

namespace osr::preprocessing::elevation {

using elevation_meters_t =
    cista::strong<std::int16_t, struct elevation_meters_>;

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
  bool operator==(tile_idx_t const&) const = default;
  std::strong_ordering operator<=>(tile_idx_t const& o) const {
    return std::tie(driver_idx_, tile_idx_, sub_tile_idx_) <=>
           std::tie(o.driver_idx_, o.tile_idx_, o.sub_tile_idx_);
  }

  data_t driver_idx_ : kDriverIdxSize;
  data_t tile_idx_ : kTileIdxSize;
  data_t sub_tile_idx_ : kSubTileIdxSize;
};
static_assert(sizeof(tile_idx_t) == sizeof(tile_idx_t::data_t));

template <typename ElevationProvider>
concept IsProvider =
    requires(ElevationProvider const& provider, point const& p) {
      { provider.get(p) } -> std::same_as<elevation_meters_t>;
      { provider.tile_idx(p) } -> std::same_as<tile_idx_t>;
      { provider.max_resolution() } -> std::same_as<resolution>;
    };

template <typename Tile>
concept IsTile = IsProvider<Tile> && requires(Tile const& tile) {
  { tile.get_box() } -> std::same_as<geo::box>;
};

template <typename Driver>
concept IsDriver = IsProvider<Driver> && requires(Driver const& driver) {
  { driver.n_tiles() } -> std::same_as<std::size_t>;
};

}  // namespace osr::preprocessing::elevation
