#include "osr/backend/http_server.h"

#include <utility>

#include "boost/algorithm/string.hpp"
#include "boost/asio/post.hpp"
#include "boost/beast/version.hpp"
#include "boost/json.hpp"
#include "boost/thread/tss.hpp"

#include "rapidjson/error/en.h"

#include "fmt/core.h"

#include "utl/enumerate.h"
#include "utl/pipes.h"
#include "utl/to_vec.h"

#include "net/web_server/responses.h"
#include "net/web_server/serve_static.h"
#include "net/web_server/web_server.h"

#include "osr/geojson.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"

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
          "X-Content-Type-Options, X-Requested-With, Content-Type, Accept, "
          "Authorization");
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

location parse_location(json::value const& v) {
  auto const& obj = v.as_object();
  return {obj.at("lat").as_double(), obj.at("lng").as_double(),
          obj.contains("level") ? to_level(obj.at("level").to_number<float>())
                                : level_t::invalid()};
}

json::value to_json(std::vector<geo::latlng> const& polyline) {
  auto a = json::array{};
  for (auto const& p : polyline) {
    a.emplace_back(json::array{p.lng(), p.lat()});
  }
  return a;
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

  template <typename Profile>
  dijkstra<Profile>& get_dijkstra() {
    static auto s = boost::thread_specific_ptr<dijkstra<Profile>>{};
    if (s.get() == nullptr) {
      s.reset(new dijkstra<Profile>{});
    }
    return *s.get();
  }

  search_profile get_search_profile_from_request(boost::json::object const& q) {
    auto const profile_it = q.find("profile");
    return profile_it == q.end() || !profile_it->value().is_string()
               ? search_profile::kFoot
               : to_profile(profile_it->value().as_string());
  }

  void handle_route(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto const q = boost::json::parse(req.body()).as_object();
    auto const profile = get_search_profile_from_request(q);
    auto const direction_it = q.find("direction");
    auto const direction = to_direction(
        direction_it == q.end() || !direction_it->value().is_string()
            ? to_str(direction::kForward)
            : direction_it->value().as_string());
    auto const from = parse_location(q.at("start"));
    auto const to = parse_location(q.at("destination"));
    auto const max_it = q.find("max");
    auto const max = static_cast<cost_t>(
        max_it == q.end() ? 3600 : max_it->value().as_int64());

    auto p = std::optional<path>{};

    switch (profile) {
      case search_profile::kFoot:
        handle_routing<foot<false>>(req, cb, from, to, max, direction);
        break;
      case search_profile::kWheelchair:
        handle_routing<foot<true>>(req, cb, from, to, max, direction);
        break;
      case search_profile::kBike:
        handle_routing<bike>(req, cb, from, to, max, direction);
        break;
      case search_profile::kCar:
        handle_routing<car>(req, cb, from, to, max, direction);
        break;
      case search_profile::kCarParking:
        handle_routing<car_parking<false>>(req, cb, from, to, max, direction);
        break;
      case search_profile::kCarParkingWheelchair:
        handle_routing<car_parking<true>>(req, cb, from, to, max, direction);
        break;
      default: throw utl::fail("not implemented");
    }
  }

  template <typename Profile>
  void handle_routing(web_server::http_req_t const& req,
                      web_server::http_res_cb_t const& cb,
                      location const from,
                      location const to,
                      cost_t const max,
                      direction const dir) {
    auto p = route(w_, l_, get_dijkstra<Profile>(), from, to, max, dir);

    if (!p.has_value()) {
      cb(json_response(req, "could not find a valid path",
                       http::status::not_found));
      return;
    }

    auto to_feature = [&](const path::segment& s) {
      return json::object{
          {"type", "Feature"},
          {
              "properties",
              {{"level", to_float(s.from_level_)},
               {"osm_way_id", s.way_ == way_idx_t::invalid()
                                  ? 0U
                                  : to_idx(w_.way_osm_idx_[s.way_])},
               {"cost", s.cost_},
               {"distance", s.dist_},
               {"from_node", s.from_node_properties_},
               {"to_node", s.to_node_properties_}},
          },
          {"geometry", to_line_string(s.polyline_)}};
    };

    auto feature_collection_meta_data =
        json::value{{"duration", p->cost_}, {"distance", p->dist_}};
    auto features = utl::all(p->segments_) | utl::transform(to_feature) |
                    utl::emplace_back_to<json::array>();

    auto feature_collection =
        json::object{{"type", "FeatureCollection"},
                     {"metadata", feature_collection_meta_data},
                     {"features", features}};

    cb(json_response(req, json::serialize(feature_collection)));
  }

  void handle_levels(web_server::http_req_t const& req,
                     web_server::http_res_cb_t const& cb) {
    auto const query = boost::json::parse(req.body()).as_object();
    auto const waypoints = query.at("waypoints").as_array();
    auto const min = point::from_latlng(
        {waypoints[1].as_double(), waypoints[0].as_double()});
    auto const max = point::from_latlng(
        {waypoints[3].as_double(), waypoints[2].as_double()});
    auto levels = hash_set<level_t>{};
    l_.find(min, max, [&](way_idx_t const x) {
      auto const p = w_.r_->way_properties_[x];
      levels.emplace(p.from_level());
      if (p.from_level() != p.to_level()) {
        levels.emplace(p.to_level());
      }
    });
    auto levels_sorted =
        utl::to_vec(levels, [](level_t const l) { return to_float(l); });
    utl::sort(levels_sorted, [](auto&& a, auto&& b) { return a > b; });
    cb(json_response(req,
                     json::serialize(utl::all(levels_sorted)  //
                                     | utl::emplace_back_to<json::array>())));
  }

  void handle_graph(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto const query = boost::json::parse(req.body()).as_object();
    auto const waypoints = query.at("waypoints").as_array();
    auto const profile = get_search_profile_from_request(query);
    auto const min = point::from_latlng(
        {waypoints[1].as_double(), waypoints[0].as_double()});
    auto const max = point::from_latlng(
        {waypoints[3].as_double(), waypoints[2].as_double()});

    auto gj = geojson_writer{.w_ = w_};
    l_.find(min, max, [&](way_idx_t const w) { gj.write_way(w); });

    switch (profile) {
      case search_profile::kFoot:
        send_graph_response<foot<false>>(req, cb, gj);
        break;
      case search_profile::kWheelchair:
        send_graph_response<foot<true>>(req, cb, gj);
        break;
      case search_profile::kBike: send_graph_response<bike>(req, cb, gj); break;
      case search_profile::kCar: send_graph_response<car>(req, cb, gj); break;
      case search_profile::kCarParking:
        send_graph_response<car_parking<false>>(req, cb, gj);
        break;
      case search_profile::kCarParkingWheelchair:
        send_graph_response<car_parking<true>>(req, cb, gj);
        break;
      default: throw utl::fail("not implemented");
    }
  }

  template <typename Profile>
  void send_graph_response(web_server::http_req_t const& req,
                           web_server::http_res_cb_t const& cb,
                           geojson_writer& gj) {
    cb(json_response(req, gj.finish(get_dijkstra<Profile>())));
  }

  void handle_static(web_server::http_req_t const& req,
                     web_server::http_res_cb_t const& cb) {
    if (!serve_static_files_ ||
        !net::serve_static_file(static_file_path_, req, cb)) {
      return cb(net::not_found_response(req));
    }
  }

  void handle_request(web_server::http_req_t const& req,
                      web_server::http_res_cb_t const& cb) {
    std::cout << "[" << req.method_string() << "] " << req.target() << '\n';
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
        } else if (target.starts_with("/api/levels")) {
          return run_parallel(
              [this](web_server::http_req_t const& req1,
                     web_server::http_res_cb_t const& cb1) {
                handle_levels(req1, cb1);
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
      case http::verb::head: return handle_static(req, cb);
      default:
        return cb(json_response(req,
                                R"({"error": "HTTP method not supported"})",
                                http::status::bad_request));
    }
  }

  template <typename Fn>
  void run_parallel(
      Fn&& handler,  // NOLINT(cppcoreguidelines-missing-std-forward)
      web_server::http_req_t const& req,
      web_server::http_res_cb_t const& cb) {
    boost::asio::post(
        thread_pool_, [req, cb, h = std::forward<Fn>(handler), this]() {
          try {
            h(req, [req, cb, this](web_server::http_res_t&& res) {
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
          } catch (std::exception const& e) {
            return cb(json_response(
                req, fmt::format(R"({{"error": "{}"}})", e.what()),
                http::status::internal_server_error));
          }
        });
  }

  void listen(std::string const& host, std::string const& port) {
    server_.on_http_request(
        [this](web_server::http_req_t const& req,
               web_server::http_res_cb_t const& cb,
               bool /*ssl*/) { return handle_request(req, cb); });

    boost::system::error_code ec;
    server_.init(host, port, ec);
    if (ec) {
      std::cerr << "server init error: " << ec.message() << "\n";
      return;
    }

    std::cout << "Listening on http://" << host << ":" << port
              << "/ and https://" << host << ":" << port << "/" << '\n';
    if (host == "0.0.0.0") {
      std::cout << "Local link: http://127.0.0.1:" << port << "/" << '\n';
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
