#pragma once

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>

#include "osmium/io/pbf_output.hpp"
#include "osmium/io/xml_input.hpp"

namespace osr::test {

// The extract only reads .osm.pbf (via the osm library). Tests keep their input
// as reviewable XML - either an inline string (write_osm_pbf) or a checked-in
// .osm file (osm_to_pbf) - and convert it to a temporary .osm.pbf here using
// libosmium (XML input requires expat, which is linked into the test binary
// only). This avoids checking in binary .osm.pbf fixtures that merely duplicate
// an XML source.
inline std::filesystem::path osm_to_pbf(
    std::filesystem::path const& osm_xml_path) {
  auto const pbf_path = std::filesystem::temp_directory_path() /
                        (osm_xml_path.stem().string() + ".osm.pbf");
  auto reader = osmium::io::Reader{osm_xml_path.generic_string(),
                                   osmium::io::read_meta::no};
  auto header = reader.header();
  auto writer = osmium::io::Writer{pbf_path.generic_string(), header,
                                   osmium::io::overwrite::allow};
  while (auto buf = reader.read()) {
    writer(std::move(buf));
  }
  writer.close();
  reader.close();
  return pbf_path;
}

// Write inline OSM XML to a temporary .osm file and convert it to .osm.pbf.
inline std::filesystem::path write_osm_pbf(std::string_view const name,
                                           std::string_view const xml) {
  auto const xml_path =
      std::filesystem::temp_directory_path() / (std::string{name} + ".osm");
  {
    auto out = std::ofstream{xml_path};
    out << xml;
  }
  return osm_to_pbf(xml_path);
}

}  // namespace osr::test
