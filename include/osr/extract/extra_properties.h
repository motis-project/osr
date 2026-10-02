#include "osr/extract/tags.h"

namespace osr {

struct way_extra_properties {
  // Properties only required for extract
  explicit way_extra_properties(tags const&);

  constexpr bool is_foot_usable() const { return is_foot_usable_; }
  constexpr bool is_car_usable() const { return is_car_usable_; }
  constexpr bool is_parking_aisle() const { return is_parking_aisle_; }
  constexpr bool is_preferred_footpath() const {
    return is_preferred_footpath_;
  }

  std::uint8_t is_foot_usable_ : 1 = 0U;
  std::uint8_t is_car_usable_ : 1 = 0U;
  std::uint8_t is_parking_aisle_ : 1 = 0U;
  std::uint8_t is_preferred_footpath_ : 1 = 0U;
};

}  // namespace osr
