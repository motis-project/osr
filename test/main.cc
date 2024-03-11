#include <filesystem>
#include <iostream>

#include "gtest/gtest.h"

#include "utl/progress_tracker.h"

#include "test_dir.h"

#ifdef PROTOBUF_LINKED
#include "google/protobuf/stubs/common.h"
#endif

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  std::clog.rdbuf(std::cout.rdbuf());

  utl::get_active_progress_tracker_or_activate("test");

  try {
    fs::current_path(OSR_TEST_EXECUTION_DIR);
    std::cout << "executing tests in " << fs::current_path() << std::endl;
  } catch (std::exception const& e) {
    std::cout << "could not change directory to " << OSR_TEST_EXECUTION_DIR
              << "\n";
    return 1;
  }

  ::testing::InitGoogleTest(&argc, argv);
  auto test_result = RUN_ALL_TESTS();

#ifdef PROTOBUF_LINKED
  google::protobuf::ShutdownProtobufLibrary();
#endif

  return test_result;
}