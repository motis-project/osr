#pragma once
#include "boost/filesystem.hpp"

#include <filesystem>
#include "node_order_strategy.h"

namespace osr::ch {

void process_ch(std::filesystem::path const& in, std::filesystem::path const& out, std::unique_ptr<OrderStrategy>& order_strategy, size_t stall);

}// namespace osr::ch