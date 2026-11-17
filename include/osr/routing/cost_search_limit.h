#pragma once

#include "osr/routing/parameters.h"
#include "osr/types.h"

namespace osr {

inline constexpr auto kMaxDurationSearchCost = cost_t{1'000'000U};

cost_t cost_search_limit(profile_parameters const&, duration_t);

}  // namespace osr
