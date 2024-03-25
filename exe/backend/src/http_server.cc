#include "osr/backend/http_server.h"

#include "boost/algorithm/string.hpp"
#include "boost/asio/post.hpp"
#include "boost/beast/version.hpp"
#include "boost/json.hpp"
#include "boost/thread/tss.hpp"

#include "rapidjson/error/en.h"

#include "fmt/core.h"

#include "utl/enumerate.h"

#include "net/web_server/responses.h"
#include "net/web_server/serve_static.h"
#include "net/web_server/web_server.h"

#include "osr/geojson.h"
#include "osr/lookup.h"
#include "osr/route.h"

using namespace net;
using net::web_server;

namespace http = boost::beast::http;
namespace fs = std::filesystem;
namespace json = boost::json;

namespace osr::backend {

template <typename Body>
void set_cors_headers(http::response<Body>& res) {
  using namespace boost::beast::http;
  res.set(field::access_control_allow_origin, "*");
  res.set(field::access_control_allow_headers,
          "X-Requested-With, Content-Type, Accept, Authorization");
  res.set(field::access_control_allow_methods, "GET, POST, OPTIONS");
  res.set(field::access_control_max_age, "3600");
}

web_server::string_res_t json_response(
    web_server::http_req_t const& req,
    std::string const& content,
    http::status const status = http::status::ok) {
  auto res = net::string_response(req, content, status, "application/json");
  set_cors_headers(res);
  return res;
}

geo::latlng parse_latlng(json::value const& v) {
  auto const obj = v.as_object();
  return {obj.at("lat").as_double(), obj.at("lng").as_double()};
}

json::value to_json(std::vector<geo::latlng> const& polyline) {
  auto a = json::array{};
  for (auto const& p : polyline) {
    a.emplace_back(json::array{p.lng(), p.lat()});
  }
  return a;
}

std::string to_json(std::optional<path> const& p) {
  return json::serialize(boost::json::value{
      {"routes",
       p.has_value() ? json::array{json::value{{"distance", 0U},  // TODO
                                               {"duration", p->time_},
                                               {"path", to_json(p->polyline_)},
                                               {"steps", json::array{}}}}
                     : json::array{}}});
}

std::string get_graph(ways const& w,
                      lookup const& l,
                      geo::latlng const& min,
                      geo::latlng const& max) {
  auto gj = geojson_writer{.w_ = w};
  l.find(min, max, [&](way_idx_t const w) { gj.write_way(w); });
  return gj.finish();
}

struct http_server::impl {
  impl(boost::asio::io_context& ios,
       boost::asio::io_context& thread_pool,
       ways const& g,
       lookup const& l,
       std::string const& static_file_path)
      : ioc_{ios}, thread_pool_{thread_pool}, w_{g}, l_{l}, server_{ioc_} {
    try {
      if (!static_file_path.empty() && fs::is_directory(static_file_path)) {
        static_file_path_ = fs::canonical(static_file_path).string();
        serve_static_files_ = true;
      }
    } catch (fs::filesystem_error const& e) {
      throw utl::fail("static file directory not found: {}", e.what());
    }
  }

  void handle_route(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto s = boost::thread_specific_ptr<routing_state>{};
    if (s.get() == nullptr) {
      s.reset(new routing_state{});
    }
    auto const query = boost::json::parse(req.body()).as_object();
    auto const from = parse_latlng(query.at("start"));
    auto const to = parse_latlng(query.at("destination"));
    auto const max_it = query.find("max");
    auto const max = max_it == query.end() ? 7200 : max_it->value().as_int64();
    auto const profile_it = query.find("profile");
    auto const profile =
        profile_it == query.end() || !profile_it->value().is_string()
            ? to_str(search_profile::kFoot)
            : profile_it->value().as_string();
    auto const res = json_response(
        req, to_json(route(w_, l_, from, to, max, *s, read_profile(profile))));
    return cb(res);
  }

  void handle_graph(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto const query = boost::json::parse(req.body()).as_object();
    auto const waypoints = query.at("waypoints").as_array();
    auto const min = point::from_latlng(
        {waypoints[1].as_double(), waypoints[0].as_double()});
    auto const max = point::from_latlng(
        {waypoints[3].as_double(), waypoints[2].as_double()});
    cb(json_response(req, get_graph(w_, l_, min, max)));
  }

  void handle_static(web_server::http_req_t&& req,
                     web_server::http_res_cb_t&& cb) {
    if (!serve_static_files_ ||
        !net::serve_static_file(static_file_path_, req, cb)) {
      return cb(net::not_found_response(req));
    }
  }

  void handle_request(web_server::http_req_t&& req,
                      web_server::http_res_cb_t&& cb) {
    std::cout << "[" << req.method_string() << "] " << req.target()
              << std::endl;
    switch (req.method()) {
      case http::verb::options: return cb(json_response(req, {}));
      case http::verb::post: {
        auto const& target = req.target();
        if (target.starts_with("/api/route")) {
          return run_parallel(
              [this](web_server::http_req_t const& req1,
                     web_server::http_res_cb_t const& cb1) {
                handle_route(req1, cb1);
              },
              req, cb);
        } else if (target.starts_with("/api/graph")) {
          return run_parallel(
              [this](web_server::http_req_t const& req1,
                     web_server::http_res_cb_t const& cb1) {
                handle_graph(req1, cb1);
              },
              req, cb);
        } else {
          return cb(json_response(req, R"({"error": "Not found"})",
                                  http::status::not_found));
        }
      }
      case http::verb::get:
      case http::verb::head:
        return handle_static(std::move(req), std::move(cb));
      default:
        return cb(json_response(req,
                                R"({"error": "HTTP method not supported"})",
                                http::status::bad_request));
    }
  }

  template <typename Fn>
  void run_parallel(Fn&& handler,
                    web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    boost::asio::post(thread_pool_, [req, cb, handler, this]() {
      handler(req, [req, cb, this](web_server::http_res_t&& res) {
        boost::asio::post(ioc_, [cb, req, res{std::move(res)}]() mutable {
          try {
            cb(std::move(res));
          } catch (std::exception const& e) {
            return cb(json_response(
                req, fmt::format(R"({{"error": "{}"}})", e.what()),
                http::status::internal_server_error));
          }
        });
      });
    });
  }

  void listen(std::string const& host, std::string const& port) {
    server_.on_http_request([this](web_server::http_req_t req,
                                   web_server::http_res_cb_t cb, bool /*ssl*/) {
      return handle_request(std::move(req), std::move(cb));
    });

    boost::system::error_code ec;
    server_.init(host, port, ec);
    if (ec) {
      std::cerr << "server init error: " << ec.message() << "\n";
      return;
    }

    std::cout << "Listening on http://" << host << ":" << port
              << "/ and https://" << host << ":" << port << "/" << std::endl;
    if (host == "0.0.0.0") {
      std::cout << "Local link: http://127.0.0.1:" << port << "/" << std::endl;
    }
    server_.run();
  }

  void stop() { server_.stop(); }

private:
  boost::asio::io_context& ioc_;
  boost::asio::io_context& thread_pool_;
  ways const& w_;
  lookup const& l_;
  web_server server_;
  bool serve_static_files_{false};
  std::string static_file_path_;
};

http_server::http_server(boost::asio::io_context& ioc,
                         boost::asio::io_context& thread_pool,
                         ways const& w,
                         lookup const& l,
                         std::string const& static_file_path)
    : impl_(new impl(ioc, thread_pool, w, l, static_file_path)) {}

http_server::~http_server() = default;

void http_server::listen(std::string const& host, std::string const& port) {
  impl_->listen(host, port);
}

void http_server::stop() { impl_->stop(); }

}  // namespace osr::backend
