#include "osr/extract/extra_properties.h"

#include <string_view>

namespace osr {

using namespace std::string_view_literals;

way_extra_properties::way_extra_properties(tags const& t)
    :  // Preferred ways
      is_parking_aisle_{static_cast<std::uint8_t>(
          t.highway_ == "service"sv && t.service_ == "parking_aisle"sv)},
      is_preferred_footpath_{static_cast<std::uint8_t>(
          t.highway_ == "living_street"sv || t.highway_ == "track"sv ||
          t.highway_ == "pedestrian"sv || t.highway_ == "path"sv ||
          t.highway_ == "footway"sv || t.sidewalk_)} {}

// Maybe relevant in the future:
// Access restrictions:
// https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access_restrictions#Worldwide

}  // namespace osr
