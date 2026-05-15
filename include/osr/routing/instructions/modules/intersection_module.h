#pragma once

#include "osr/routing/instructions/module.h"

namespace osr {

struct intersection_module : instruction_module {
  bool can_process(ways const& way, path& p, segment_contexts_window const& meta) override;
  void process(ways const& w, path& p, segment_contexts_window const& meta) override;

};

} // namespace osr
