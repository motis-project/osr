#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"

#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace preprocessing::elevation {
struct provider;
}

struct elevation_storage {
  struct elevation {
    elevation& operator+=(elevation const&);
    elevation swap() const;
    elevation_monotonic_t up_{0U};
    elevation_monotonic_t down_{0U};
  };
  struct encoding {
    using coding = std::uint8_t;
    explicit encoding(elevation const&);
    encoding() = default;
    elevation decode() const;
    explicit operator bool() const;
    coding up_ : 4 {};
    coding down_ : 4 {};
  };
  elevation_storage(std::filesystem::path const&,
                    cista::mmap::protection const mode);
  static std::unique_ptr<elevation_storage> try_open(
      std::filesystem::path const&);
  void set_elevations(ways const&, preprocessing::elevation::provider const&);
  elevation get_elevations(way_idx_t const way,
                           std::uint16_t const segment) const;

  mm_paged_vecvec<way_idx_t, encoding> elevations_;
};

elevation_storage::elevation get_elevations(elevation_storage const*,
                                            way_idx_t const way,
                                            std::uint16_t const segment);

}  // namespace osr