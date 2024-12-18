#pragma once

#include <concepts>
#include <filesystem>

#include "osr/point.h"
#include "osr/preprocessing/elevation/step_size.h"
#include "osr/types.h"

namespace osr::preprocessing::elevation {

template <typename Driver>
concept IsRasterDriver = requires(Driver driver) {
  {
    std::as_const(driver).get(std::declval<::osr::point>())
  } -> std::same_as<::osr::elevation_t>;
  { std::as_const(driver).get_step_size() } -> std::same_as<step_size>;
};

struct provider {
  provider(std::filesystem::path const&);
  ~provider();
  provider(provider const&) = delete;
  provider& operator=(provider const&) = delete;
  provider(provider&&) = delete;
  provider& operator=(provider&&) = delete;

  ::osr::elevation_t get(::osr::point const&) const;
  std::size_t driver_count() const;
  step_size get_step_size() const;

private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace osr::preprocessing::elevation
