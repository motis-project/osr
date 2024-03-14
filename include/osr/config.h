#pragma once

#include <filesystem>

#include "conf/configuration.h"

namespace osr {

struct config : public conf::configuration {
  config(std::filesystem::path in, std::filesystem::path const& out);
  ~config() override;

  std::filesystem::path in_, out_;
};

}  // namespace osr