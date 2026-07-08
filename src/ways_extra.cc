#include "osr/ways_extra.h"

using std::literals::string_view_literals::operator""sv;

namespace osr {

way_extra_properties::way_extra_properties(tags const& t)
    // Defaults according to
    // https://wiki.openstreetmap.org/wiki/OSM_tags_for_routing/Access_restrictions#Worldwide
    : is_foot_usable_{
          t.highway_ == "trunk"sv || t.highway_ == "trunk_link"sv ||
          t.highway_ == "primaray"sv || t.highway_ == "primary_link"sv ||
          t.highway_ == "secondary"sv || t.highway_ == "secondary_link"sv ||
          t.highway_ == "tertiary"sv || t.highway_ == "tertiary_link"sv ||
          t.highway_ == "unclassified"sv || t.highway_ == "residential"sv ||
          t.highway_ == "living_street"sv || t.highway_ == "road"sv ||
          t.highway_ == "track"sv || t.highway_ == "pedestrian"sv ||
          t.highway_ == "path"sv || t.highway_ == "footway"sv},
      is_car_usable_{
          t.highway_ == "motorway"sv || t.highway_ == "motorway_link"sv ||
          t.highway_ == "trunk"sv || t.highway_ == "trunk_link"sv ||
          t.highway_ == "primaray"sv || t.highway_ == "primary_link"sv ||
          t.highway_ == "secondary"sv || t.highway_ == "secondary_link"sv ||
          t.highway_ == "tertiary"sv || t.highway_ == "tertiary_link"sv ||
          t.highway_ == "unclassified"sv || t.highway_ == "residential"sv ||
          t.highway_ == "living_street"sv || t.highway_ == "road"sv ||
          t.highway_ == "track"sv},
      is_parking_aisle_{t.highway_ == "service"sv &&
                        t.service_ == "parking_aisle"sv} {
  //    switch (t.highway_) {
  //      case "motorway"sv:
  //      case "motorway"sv:
  //      case "motorway"sv:
  // }
}
}
