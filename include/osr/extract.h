#include "osr/types.h"

#include <filesystem>

namespace osr {

void extract(std::filesystem::path const& in,
             std::filesystem::path const& graph_out,
             std::filesystem::path const& db_out,
             std::filesystem::path const& tmp);

}  // namespace osr