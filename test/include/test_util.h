#pragma once

#include <string>
#include <string_view>
#include "osr/location.h"
#include "osr/routing/path.h"
#include "osr/lookup.h"

std::optional<osr::path> route(osr::ways const& w,
                               osr::lookup const& l,
                               osr::location const& from,
                               osr::location const& to,
                               osr::profile_parameters const& params,
                               osr::search_profile sp);

std::optional<osr::path> extract_and_route(std::string_view path,
                                           osr::location const& from,
                                           osr::location const& to,
                                           osr::profile_parameters const& params,
                                           osr::search_profile sp);

std::string extract_route_to_feature(std::string_view path,
                                     osr::location const& from,
                                     osr::location const& to,
                                     osr::profile_parameters const& params  =
                                      osr::foot<false, osr::elevator_tracking>::parameters{},
                                     osr::search_profile sp = osr::search_profile::kFoot);