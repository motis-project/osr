#pragma once

#include <cinttypes>
#include <string_view>

namespace osr {

enum class routing_algorithm : std::uint8_t { kDijkstra, kAStarBi };

routing_algorithm to_algorithm(std::string_view);

}  // namespace osr
