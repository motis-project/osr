#include <filesystem>

namespace osr {

void extract(bool with_platforms,
             std::filesystem::path const& in,
             std::filesystem::path const& out,
             std::filesystem::path const& elevation_dir,
             bool with_cch = true);

}  // namespace osr
