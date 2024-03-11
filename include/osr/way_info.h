#pragma once

#include <cinttypes>
#include <vector>

#include "osr/buf_io.h"
#include "osr/types.h"

namespace osr {

struct way_info {
  void write(std::vector<std::uint8_t>& buf) const {
    write_buf(buf, nodes_, edge_flags_);
  }

  way_info& read(std::string_view s) {
    read_buf(span(s), nodes_, edge_flags_);
    return *this;
  }

  edge_flags_t edge_flags_;
  std::vector<osm_node_idx_t> nodes_;
};

}  // namespace osr