#pragma once

#include "osr/routing/instructions/meta_data.h"
#include "osr/routing/path.h"
#include "osr/util/sliding_window.h"

namespace osr {

struct ways;

using segment_contexts = std::vector<segment_context>;
using segment_contexts_window = window<1, segment_contexts::iterator>;

struct instruction_module {
  virtual ~instruction_module() = default;
  virtual bool process(ways const& w, path& p, segment_contexts_window const& window) = 0;
};

} // namespace osr
