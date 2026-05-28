#pragma once

#include <optional>
#include <string_view>

#include "osr/ways.h"

namespace osr {

struct conditional_storage_builder {
  ways::routing& routing_;
  way_conditional_restrictions way_{};
};

bool parse_conditional_restriction_tag(std::string_view,
                                       std::string_view,
                                       conditional_storage_builder&);

std::optional<conditional_condition_set_idx_t> parse_conditional_condition_set(
    std::string_view, conditional_storage_builder&);

}  // namespace osr