#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct motorway_module : instruction_module {
  void process(ways const& w, path& p, segment_contexts_window const& window) override;
};

}  // namespace osr
