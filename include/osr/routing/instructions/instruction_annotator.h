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
  void add_module(std::unique_ptr<instruction_module> m);

private:
  void preprocess(path const& path, instruction_meta_data& meta);

  ways const& ways_;
  std::vector<std::unique_ptr<instruction_module>> modules_;
};

}