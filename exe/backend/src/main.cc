#include <filesystem>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "boost/asio/io_context.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "net/stop_handler.h"

#include "osr/backend/http_server.h"
#include "osr/lookup.h"
#include "osr/ways.h"

namespace fs = std::filesystem;
using namespace osr;
using namespace osr::backend;

class settings : public conf::configuration {
public:
  explicit settings() : configuration("Options") {
    param(data_dir_, "data,d", "Data directory");
    param(http_host_, "host", "HTTP host");
    param(http_port_, "port", "HTTP port");
    param(static_file_path_, "static", "Path to static files (ui/web)");
    param(threads_, "routing-threads", "Number of routing threads");
  }

  fs::path data_dir_{"osr"};
  std::string http_host_{"0.0.0.0"};
  std::string http_port_{"8000"};
  std::string static_file_path_;
  unsigned threads_{std::thread::hardware_concurrency()};
};

auto run(boost::asio::io_context& ioc) {
  return [&ioc]() {
    while (true) {
      try {
        ioc.run();
        break;
      } catch (std::exception const& e) {
        fmt::println("unhandled error: {}", e.what());
      } catch (...) {
        fmt::println("unhandled unknown error");
      }
    }
  };
}

int main(int argc, char const* argv[]) {
  auto opt = settings{};
  auto parser = conf::options_parser({&opt});
  parser.read_command_line_args(argc, argv);

  if (parser.help()) {
    parser.print_help(std::cout);
    return 0;
  } else if (parser.version()) {
    return 0;
  }

  parser.read_configuration_file();
  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::exists(opt.data_dir_)) {
    fmt::println("file not found: {}", opt.data_dir_);
    return 1;
  }

  auto const w = ways{opt.data_dir_, cista::mmap::protection::READ};
  auto const l = lookup{w};

  auto ioc = boost::asio::io_context{};
  auto pool = boost::asio::io_context{};
  auto server = http_server{ioc, pool, w, l, opt.static_file_path_};

  auto work_guard = boost::asio::make_work_guard(pool);
  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  for (auto& t : threads) {
    t = std::thread(run(pool));
  }

  server.listen(opt.http_host_, opt.http_port_);

  auto const stop = net::stop_handler(ioc, [&]() {
    server.stop();
    ioc.stop();
  });

  ioc.run();
  pool.stop();
  for (auto& t : threads) {
    t.join();
  }

  return 0;
}
