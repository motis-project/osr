#pragma once

#include <optional>

#include "osr/routing/path.h"
#include "osr/location.h"
#include "osr/geojson.h"
#include "osr/lookup.h"
#include "osr/routing/route.h"

namespace osr::test {

std::optional<path> route(ways const& w, lookup const& l, location const& from,
                          location const& to, profile_parameters const& params,
                          search_profile sp, direction dir,
                          routing_algorithm algo);

std::optional<path> extract_and_route(std::string_view path,
                                      location const& from, location const& to,
                                      profile_parameters const& params,
                                      search_profile sp, direction dir,
                                      routing_algorithm algo);

std::string extract_route_to_feature(std::string_view path,
                                     location const& from, location const& to,
                                     profile_parameters const& params,
                                     search_profile sp, direction dir,
                                     routing_algorithm algo);
}  // namespace osr::test