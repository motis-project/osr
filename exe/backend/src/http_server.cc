#include "osr/backend/http_server.h"

#include "boost/algorithm/string.hpp"
#include "boost/asio/post.hpp"
#include "boost/beast/version.hpp"

#include "boost/filesystem.hpp"

#include "rapidjson/error/en.h"

#include "fmt/core.h"

#include "utl/enumerate.h"

#include "net/web_server/responses.h"
#include "net/web_server/serve_static.h"
#include "net/web_server/web_server.h"

#include "osr/backend/json.h"
#include "osr/lookup.h"
#include "osr/route.h"

using namespace net;
using net::web_server;

namespace http = boost::beast::http;
namespace fs = boost::filesystem;

namespace osr::backend {

search_profile read_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("pedestrian"): return search_profile::kPedestrian;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("car"): return search_profile::kCar;
  }
  throw utl::fail("{} is not a valid profile", s);
}

struct route_request {
  point start_, destination_;
  search_profile profile_;
};

struct graph_request {
  std::vector<point> waypoints_;
};

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

rapidjson::Document parse_json(std::string const& s) {
  auto doc = rapidjson::Document{};
  doc.Parse<rapidjson::kParseDefaultFlags>(s.c_str());
  if (doc.HasParseError()) {
    throw utl::fail("parser error: invalid json at {}: {}",
                    doc.GetErrorOffset(),
                    rapidjson::GetParseError_En(doc.GetParseError()));
  }
  return doc;
}

void get_input_location(point& loc,
                        rapidjson::Value const& doc,
                        char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsObject()) {
      auto const& lng = val.FindMember("lng");
      auto const& lat = val.FindMember("lat");
      if (lng != val.MemberEnd() && lat != val.MemberEnd() &&
          lng->value.IsNumber() && lat->value.IsNumber()) {
        loc = point::from_latlng(
            {lat->value.GetDouble(), lng->value.GetDouble()});
      } else {
        throw utl::fail("json: {} did not contain lng and lat", key);
      }
    } else {
      throw utl::fail("json: {} is not an object", key);
    }
  } else {
    throw utl::fail("json: {} not found", key);
  }
}

void get_waypoints(std::vector<point>& waypoints,
                   rapidjson::Value const& doc,
                   char const* key) {
  if (doc.HasMember(key)) {
    auto const& val = doc[key];
    if (val.IsArray() && val.Size() == 4U) {
      for (auto i = 0U; i < val.Size() - 2U; i += 2) {
        waypoints.emplace_back(
            point::from_latlng({val[i + 1].GetDouble(), val[i].GetDouble()}));
      }
    }
  }
}

void get_profile(search_profile& profile,
                 rapidjson::Value const& doc,
                 char const* key) {
  if (doc.HasMember(key)) {
    auto const& str = doc[key];
    if (str.IsString()) {
      profile = read_profile({str.GetString(), str.GetStringLength()});
    }
  }
}

route_request parse_route_request(web_server::http_req_t const& req) {
  route_request r{};
  auto doc = parse_json(req.body());
  get_input_location(r.start_, doc, "start");
  get_input_location(r.destination_, doc, "destination");
  get_profile(r.profile_, doc, "profile");
  return r;
}

graph_request parse_graph_request(web_server::http_req_t const& req) {
  graph_request r{};
  auto doc = parse_json(req.body());
  get_waypoints(r.waypoints_, doc, "waypoints");
  return r;
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

    if (serve_static_files_) {
      std::cout << "Serving static files from " << static_file_path_
                << std::endl;
    } else {
      std::cout << "Not serving static files" << std::endl;
    }
  }

  void handle_route(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto res =
        json_response(req, to_json(route(w_, l_, parse_route_request(req))));
    return cb(res);
  }

  void handle_graph(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    cb(json_response(req,
                     to_json(get_graph(w_, l_, parse_graph_request(req)))));
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
