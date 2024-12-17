#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

constexpr elevation_t NO_ELEVATION_DATA = -32767;

namespace preprocessing::elevation {
struct provider;
}

struct elevation_storage {
  struct elevation {
    elevation_t up_;
    elevation_t down_;
  };
  // TODO Rename
  struct compressed_elevation {
    using coding = std::uint8_t;
    static coding encode(elevation_t const);
    static elevation_t decode(coding const);
    bool is_flat() const;
    coding up_ : 4;
    coding down_ : 4;
  };
  elevation_storage(std::filesystem::path const&,
                    cista::mmap::protection const mode);
  static std::unique_ptr<elevation_storage> try_open(
      std::filesystem::path const&);
  void set_elevations(ways&, preprocessing::elevation::provider const&);
  elevation get_elevations(way_idx_t const way,
                           std::uint16_t const from,
                           std::uint16_t const to) const;

  mm_vecvec<way_idx_t, compressed_elevation> elevations_;
};

elevation_storage::elevation get_elevations(elevation_storage const*,
                                            way_idx_t const way,
                                            std::uint16_t const from,
                                            std::uint16_t const to);

}  // namespace osr