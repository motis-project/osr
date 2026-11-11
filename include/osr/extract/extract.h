#include <filesystem>

namespace osr {
namespace preprocessing::elevation {
struct elevation_params;
}

void extract(bool with_platforms,
             std::filesystem::path const& in,
             std::filesystem::path const& out,
             std::filesystem::path const& elevation_dir,
             preprocessing::elevation::elevation_params params);

}  // namespace osr