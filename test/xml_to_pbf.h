#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include "osmium/io/pbf_output.hpp"
#include "osmium/io/xml_input.hpp"

namespace osr::test {

// The extract only reads .osm.pbf (via the osm library). Tests define their
// input as readable inline XML - this helper converts it to a temporary
// .osm.pbf file using libosmium (XML input requires expat, which is linked
// into the test binary only).
inline std::filesystem::path write_osm_pbf(std::string_view const name,
                                           std::string_view const xml) {
  auto const xml_path =
      std::filesystem::temp_directory_path() / (std::string{name} + ".osm");
  {
    auto out = std::ofstream{xml_path};
    out << xml;
  }

  auto const pbf_path =
      std::filesystem::temp_directory_path() / (std::string{name} + ".osm.pbf");
  {
    auto reader = osmium::io::Reader{xml_path.generic_string(),
                                     osmium::io::read_meta::no};
    auto header = reader.header();
    auto writer = osmium::io::Writer{pbf_path.generic_string(), header,
                                     osmium::io::overwrite::allow};
    while (auto buf = reader.read()) {
      writer(std::move(buf));
    }
    writer.close();
    reader.close();
  }
  return pbf_path;
}

}  // namespace osr::test
