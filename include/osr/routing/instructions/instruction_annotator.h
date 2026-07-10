#pragma once

#include <vector>
#include <memory>

#include "osr/routing/path.h"
#include "osr/routing/instructions/module.h"
#include "osr/routing/instructions/meta_data.h"

namespace osr {

struct ways;

class instruction_annotator {
public:
  explicit instruction_annotator(ways const& w);
  void annotate(path& path);

private:
  void preprocess(path& path, std::vector<segment_context>& meta);
  void init_chain();
  void process_chain();

  ways const& ways_;
  std::vector<std::shared_ptr<instruction_module>> modules_;
  std::shared_ptr<instruction_module> head_module_;
};

}