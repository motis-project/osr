#pragma once

#include "osr/lookup.h"
#include "osr/routing/path.h"

namespace osr {

class instruction_annotator {
public:
  void annotate(path& path);

private:
  static void set_relative_direction(path& path, std::size_t towards_idx);
};

}