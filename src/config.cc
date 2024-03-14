#include "osr/config.h"

namespace osr {

config::config(std::filesystem::path in, std::filesystem::path const& out)
    : configuration{"Options"}, in_{std::move(in)}, out_{out} {
  param(in_, "in,i", "OpenStreetMap .osm.pbf input path");
  param(out_, "out,o", "output directory");
}

config::~config() = default;

}  // namespace osr