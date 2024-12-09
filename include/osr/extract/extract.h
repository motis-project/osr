#include <filesystem>
#include <optional>

namespace osr {

void extract(
    bool with_platforms,
    std::filesystem::path const& in,
    std::filesystem::path const& out,
    std::optional<std::filesystem::path> const& elevation_dir = std::nullopt);

}  // namespace osr