#pragma once

#include <string>
#include <string_view>
#include "osr/location.h"
#include "osr/routing/path.h"
#include "osr/lookup.h"

std::optional<osr::path> route(osr::ways const& w,
                               osr::lookup const& l,
                               osr::location const& from,
                               osr::location const& to);

std::optional<osr::path> extract_and_route(std::string_view path,
                                           osr::location const& from,
                                           osr::location const& to);

std::string extract_route_to_feature(std::string_view path,
                                     osr::location const& from,
                                     osr::location const& to);