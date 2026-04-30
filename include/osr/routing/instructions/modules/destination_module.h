#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct destination_module : instruction_module {
  bool process(ways const& w, path& p, segment_contexts_window const& window) override;
};

} // namespace osr
