#pragma once
#include "boost/filesystem.hpp"

#include <filesystem>

namespace osr::ch {
struct OrderStrategy;

void process_ch(std::filesystem::path const& in, std::filesystem::path const& out, std::unique_ptr<OrderStrategy> const& order_strategy);

}// namespace osr::ch