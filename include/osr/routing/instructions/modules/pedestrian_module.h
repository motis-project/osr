#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct pedestrian_module : instruction_module {
  void process(ways const& w, path& p, segment_contexts_window const& window) override;
  void post_process(ways const&, path&, segment_contexts const&) override;
};

} // namespace osr
