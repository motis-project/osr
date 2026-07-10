#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct roundabout_module : instruction_module {
  void process(ways const& w, path& p, segment_contexts_window const& window) override;
  void post_process(ways const& w, path& p, segment_contexts const& contexts) override;
};

} // namespace osr
