#pragma once

#include "osr/routing/path.h"
#include "osr/routing/instructions/meta_data.h"

namespace osr {

struct ways;

struct instruction_module {
  virtual ~instruction_module() = default;
  virtual bool process(ways const& w, path& p, std::size_t segment_idx, instruction_meta_data const& meta) = 0;
};

} // namespace osr
