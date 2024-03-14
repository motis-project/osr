#pragma once

#include <filesystem>

#include "conf/configuration.h"

namespace osr {

struct config : public conf::configuration {
  config(std::filesystem::path in, std::filesystem::path const& out);
  ~config() override;

  std::filesystem::path in_;
  std::filesystem::path graph_;
  std::filesystem::path node_map_;
  std::filesystem::path edge_map_;
  std::filesystem::path db_;
  std::filesystem::path tmp_;
  std::size_t db_max_size_;
};

}  // namespace osr