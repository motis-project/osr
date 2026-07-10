#pragma once

#include "osr/routing/instructions/meta_data.h"
#include "osr/routing/path.h"
#include "osr/util/sliding_window.h"
#include "osr/ways.h"

namespace osr {

using segment_contexts = std::vector<segment_context>;
using segment_contexts_window = window<1, segment_contexts::iterator>;

class instruction_module {
public:
  virtual ~instruction_module() = default;

  virtual void process(ways const& w, path& p,
                       segment_contexts_window const& window) {
    if (this->next_module_) {
      this->next_module_->process(w, p, window);
    }
  }

  virtual std::shared_ptr<instruction_module> set_next(
      std::shared_ptr<instruction_module> next_module) {
    this->next_module_ = next_module;
    return next_module;
  }

  virtual void post_process(ways const&, path&, segment_contexts const&) {}

private:
  std::shared_ptr<instruction_module> next_module_;
};

} // namespace osr
