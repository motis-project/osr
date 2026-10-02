#include "osr/extract/extra_properties.h"

#include <string_view>

namespace osr {

using namespace std::string_view_literals;

way_extra_properties::way_extra_properties(tags const& t)
    // Defaults according to
    // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access_restrictions#Worldwide
    : is_foot_usable_{static_cast<std::uint8_t>(
          t.highway_ == "trunk"sv || t.highway_ == "trunk_link"sv ||
          t.highway_ == "primary"sv || t.highway_ == "primary_link"sv ||
          t.highway_ == "secondary"sv || t.highway_ == "secondary_link"sv ||
          t.highway_ == "tertiary"sv || t.highway_ == "tertiary_link"sv ||
          t.highway_ == "unclassified"sv || t.highway_ == "residential"sv ||
          t.highway_ == "living_street"sv || t.highway_ == "road"sv ||
          t.highway_ == "track"sv || t.highway_ == "pedestrian"sv ||
          t.highway_ == "path"sv || t.highway_ == "footway"sv)},
      is_car_usable_{static_cast<std::uint8_t>(
          t.highway_ == "motorway"sv || t.highway_ == "motorway_link"sv ||
          t.highway_ == "trunk"sv || t.highway_ == "trunk_link"sv ||
          t.highway_ == "primary"sv || t.highway_ == "primary_link"sv ||
          t.highway_ == "secondary"sv || t.highway_ == "secondary_link"sv ||
          t.highway_ == "tertiary"sv || t.highway_ == "tertiary_link"sv ||
          t.highway_ == "unclassified"sv || t.highway_ == "residential"sv ||
          t.highway_ == "living_street"sv || t.highway_ == "road"sv ||
          t.highway_ == "track"sv)},
      // For preferences
      is_parking_aisle_{static_cast<std::uint8_t>(
          t.highway_ == "service"sv && t.service_ == "parking_aisle"sv)},
      is_preferred_footpath_{static_cast<std::uint8_t>(
          t.highway_ == "living_street"sv || t.highway_ == "track"sv ||
          t.highway_ == "pedestrian"sv || t.highway_ == "path"sv ||
          t.highway_ == "footway"sv || t.sidewalk_)} {
  // No worldwide default, but usable for each country listing 'service'
  if (t.highway_ == "service"sv) {
    is_foot_usable_ = true;
    is_car_usable_ = true;
  }
}

}  // namespace osr
