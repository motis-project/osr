#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include "cista/mmap.h"
#include "cista/strong.h"

#include "osr/preprocessing/elevation/shared.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

namespace preprocessing::elevation {
struct provider;
}

struct elevation_storage {
  struct elevation {
    elevation& operator+=(elevation const&);
    elevation swapped() const;

    cista::base_t<way_idx_t> idx_{0};
    std::uint8_t n_{0};
    elevation_monotonic_t up_{0};
    elevation_monotonic_t down_{0};
  };

  struct encoding {
    using coding = std::uint8_t;
    encoding(cista::base_t<way_idx_t> idx,
             std::uint8_t n,
             coding up,
             coding down)
        : idx_{idx}, n_{n}, up_{up}, down_{down} {}
    explicit encoding(elevation const&);
    encoding() = default;
    elevation decode() const;
    explicit operator bool() const;
    bool valid() const { return n_ != 0; }

    cista::base_t<way_idx_t> idx_{0};
    std::uint8_t n_{0};
    coding up_ : 4 {};
    coding down_ : 4 {};
  };

  elevation_storage(std::filesystem::path const&,
                    cista::mmap::protection const mode);
  static std::unique_ptr<elevation_storage> try_open(
      std::filesystem::path const&);
  void set_elevations(
      ways const&,
      preprocessing::elevation::provider const&,
      preprocessing::elevation::elevation_meters_t min_elevation,
      double max_incline);
  elevation get_elevations(way_idx_t const way,
                           std::uint16_t const segment) const;

  // (way_idx, internal-idx) -> single elevation sample
  // mm_vecvec<way_idx_t, encoding> elevations_;
  using data_vec_t = mm_vec32<preprocessing::elevation::elevation_meters_t>;
  data_vec_t elevations_;

  // (way_idx, segment_idx) -> internal-idx, number of samples
  mm_vecvec<way_idx_t, encoding> offsets_;
};

elevation_storage::elevation get_elevations(elevation_storage const*,
                                            way_idx_t const way,
                                            std::uint16_t const segment);

}  // namespace osr