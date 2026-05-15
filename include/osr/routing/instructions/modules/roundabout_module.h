#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct roundabout_module : instruction_module {
  bool can_process(ways const& way, path& p, segment_contexts_window const& meta) override;
  void process(ways const& w, path& p, segment_contexts_window const& window) override;
};

} // namespace osr
