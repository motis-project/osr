#pragma once

#include <memory>
#include <string>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

template <std::size_t RasterSize>
struct hgt {
  explicit hgt(std::string const& filename);
  ~hgt();
  hgt(hgt&& grid) noexcept;
  hgt(hgt const&) = delete;
  hgt& operator=(hgt const&) = delete;
  hgt& operator=(hgt&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;

  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

extern template struct hgt<3601U>;
extern template struct hgt<1201U>;

}  // namespace osr::preprocessing::elevation
