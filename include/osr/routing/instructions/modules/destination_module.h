#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct destination_module : instruction_module {
  bool process(ways const& w, path& p, std::size_t segment_idx, instruction_meta_data const& meta) override;
};

} // namespace osr
