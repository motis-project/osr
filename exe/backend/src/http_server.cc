#include "osr/backend/http_server.h"

#include <utility>

#include "boost/algorithm/string.hpp"
#include "boost/asio/post.hpp"
#include "boost/beast/core/string.hpp"
#include "boost/beast/version.hpp"
#include "boost/json.hpp"
#include "boost/url/parse.hpp"

#include "fmt/core.h"

#include "utl/enumerate.h"
#include "utl/pipes.h"
#include "utl/to_vec.h"

#include "net/web_server/responses.h"
#include "net/web_server/serve_static.h"
#include "net/web_server/web_server.h"

#include "osr/geojson.h"
#include "osr/lookup.h"
#include "osr/routing/algorithms.h"
#include "osr/routing/instructions/instruction_annotator.h"
#include "osr/routing/parameters.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/routing/with_profile.h"

using namespace net;
using net::web_server;

namespace http = boost::beast::http;
namespace fs = std::filesystem;
namespace json = boost::json;

namespace osr::backend {

const std::filesystem::path test_data_dir = TEST_DATA_DIR;
const std::filesystem::path test_cases_path = test_data_dir / "test-cases.json";

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
          obj.contains("level") ? level_t{obj.at("level").to_number<float>()}
                                : kNoLevel};
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
       platforms const* pl,
       elevation_storage const* elevations,
       std::string const& static_file_path,
       bool const provide_test_features)
      : ioc_{ios},
        thread_pool_{thread_pool},
        w_{g},
        l_{l},
        pl_{pl},
        elevations_{elevations},
        server_{ioc_},
        provide_test_features_(provide_test_features) {
    try {
      if (!static_file_path.empty() && fs::is_directory(static_file_path)) {
        static_file_path_ = fs::canonical(static_file_path).string();
        serve_static_files_ = true;
      }
    } catch (fs::filesystem_error const& e) {
      throw utl::fail("static file directory not found: {}", e.what());
    }
  }

  static search_profile get_search_profile_from_request(
      boost::json::object const& q) {
    auto const profile_it = q.find("profile");
    return profile_it == q.end() || !profile_it->value().is_string()
               ? search_profile::kFoot
               : to_profile(profile_it->value().as_string());
  }

  static routing_algorithm get_routing_algorithm_from_request(
      boost::json::object const& q) {
    auto const routing_it = q.find("routing");
    return routing_it == q.end() || !routing_it->value().is_string()
               ? routing_algorithm::kDijkstra
               : to_algorithm(routing_it->value().as_string());
  }

  void handle_route(web_server::http_req_t const& req,
                    web_server::http_res_cb_t const& cb) {
    auto const q = boost::json::parse(req.body()).as_object();
    auto const profile = get_search_profile_from_request(q);
    auto const direction_it = q.find("direction");
    auto const routing_algo = get_routing_algorithm_from_request(q);
    auto const dir = to_direction(direction_it == q.end() ||
                                          !direction_it->value().is_string()
                                      ? to_str(direction::kForward)
                                      : direction_it->value().as_string());
    auto const from = parse_location(q.at("start"));
    auto const to = parse_location(q.at("destination"));
    auto const max_it = q.find("max");
    auto const max = static_cast<cost_t>(
        max_it == q.end() ? 3600 : max_it->value().as_int64());
    auto const foot_speed_result =
        q.try_at("footSpeed")->try_to_number<float>();
    auto const params =
        profile == search_profile::kFoot && foot_speed_result.has_value()
            ? foot<false,
                   elevator_tracking>::parameters{.speed_meters_per_second_ =
                                                      foot_speed_result.value()}
            : get_parameters(profile);

    auto p = route(params, w_, l_, profile, from, to, max, dir, 100,
                   nullptr, nullptr, elevations_, routing_algo);

    auto const p1 = route(params, w_, l_, profile, from, std::vector{to}, max,
                          dir, 100, nullptr, nullptr, elevations_);

    auto const print = [](char const* name, std::optional<path> const& p) {
      if (p.has_value()) {
        std::cout << name << " cost: " << p->cost_ << "\n";
      } else {
        std::cout << name << ": not found\n";
      }
    };
    print("p", p);
    print("p1", p1.at(0));

    if (!p.has_value()) {
      cb(json_response(req, "could not find a valid path",
                       http::status::not_found));
      return;
    }

    auto annotator = instruction_annotator{w_};
    annotator.annotate(*p);

    cb(json_response(req, to_featurecollection(w_, p, true, true, true)));
  }

  static void handle_post_test_case(web_server::http_req_t const& req,
                                    web_server::http_res_cb_t const& cb) {
    try {
      auto const test_case = json::parse(req.body()).as_object();
      json::array all_tests;
      auto const new_name = test_case.at("name").as_string();

      std::ifstream ifs(test_cases_path);
      if (ifs.is_open()) {
        std::string content((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());
        if (!content.empty()) {
          all_tests = json::parse(content).as_array();
        }
        ifs.close();
      }

      bool found = false;
      for (auto& existing_test : all_tests) {
        if (existing_test.as_object().at("name").as_string() == new_name) {
          existing_test = test_case; // Replace existing entry
          found = true;
          break;
        }
      }
      if (!found) {
        all_tests.push_back(test_case);
      }

      std::ofstream ofs(test_cases_path);
      ofs << boost::json::serialize(all_tests);
      ofs.close();
      return cb(json_response(req, "Saved", http::status::ok));
    } catch (std::exception const& e) {
      return cb(
          json_response(req, e.what(), http::status::internal_server_error));
    }
  }

  static void handle_get_test_cases(web_server::http_req_t const& req,
                                    web_server::http_res_cb_t const& cb) {
    try {
      std::ifstream ifs(test_cases_path);
      if (ifs.is_open()) {
        std::string content((std::istreambuf_iterator<char>(ifs)),
                            std::istreambuf_iterator<char>());
        ifs.close();
        if (!content.empty()) {
          return cb(json_response(req, content, http::status::ok));
        }
      }
    } catch (std::exception const& e) {
      return cb(
          json_response(req, e.what(), http::status::internal_server_error));
    }
  }

  static void handle_get_available_feeds(web_server::http_req_t const& req,
                                         web_server::http_res_cb_t const& cb) {
    try {
      json::array feeds;

      if (fs::exists(test_data_dir) && fs::is_directory(test_data_dir)) {
        for (auto const& entry : fs::directory_iterator(test_data_dir)) {
          if (entry.is_regular_file() && entry.path().extension() == ".pbf") {
            feeds.push_back(entry.path().filename().c_str());
          }
        }
      }

      return cb(
          json_response(req, boost::json::serialize(feeds), http::status::ok));
    } catch (std::exception const& e) {
      return cb(
          json_response(req, e.what(), http::status::internal_server_error));
    }
  }

  void handle_get_traversed_node_hub(web_server::http_req_t const& req,
                                     web_server::http_res_cb_t const& cb) {

    try {
      auto target = req.target();
      auto view = boost::urls::parse_origin_form(target);

      std::array<node_idx_t, 3> nodes_indices;
      nodes_indices.fill(node_idx_t::invalid());
      std::array<way_idx_t, 2> way_indices;
      way_indices.fill(way_idx_t::invalid());

      if (view.has_value()) {
        auto const params = view->params();
        auto const mode_param_iter = params.find("mode");
        if (mode_param_iter == params.end() || !(*mode_param_iter).has_value) {
          return cb(json_response(req, "Parameter mode is missing",
                                  http::status::bad_request));
        }

        const auto m = static_cast<mode>(std::stol((*mode_param_iter).value));

        for (const auto [i, node_param] :
             utl::enumerate<std::vector<std::string>>(
                 {"osm_prev_node", "osm_hub_node", "osm_next_node"})) {
          const auto node_param_iter = params.find(node_param);
          if (node_param_iter != params.end() && (*node_param_iter).has_value) {
            osm_node_idx_t const osm_node_idx(
                std::stoll((*node_param_iter).value));
            if (osm_node_idx != 0U) {
              nodes_indices[i] = w_.get_node_idx(osm_node_idx);
            }
          }
        }

        for (const auto [i, way_param] :
             utl::enumerate<std::vector<std::string>>(
                 {"osm_arrive_way", "osm_exit_way"})) {
          const auto way_param_iter = params.find(way_param);
          if (way_param_iter != params.end() && (*way_param_iter).has_value) {
            osm_way_idx_t const osm_way_idx(
                std::stoll((*way_param_iter).value));
            if (osm_way_idx != 0U) {
              if (const auto opt_way_idx = w_.find_way(osm_way_idx);
                  opt_way_idx.has_value()) {
                way_indices[i] = *opt_way_idx;
              }
            }
          }
        }

        if (std::ranges::any_of(nodes_indices, [](node_idx_t const idx) {
              return idx == node_idx_t::invalid();
            })) {
          return cb(json_response(req, "At least one node idx is invalid",
                                  http::status::bad_request));
        }

        if (std::ranges::any_of(way_indices, [](way_idx_t const idx) {
              return idx == way_idx_t::invalid();
            })) {
          return cb(json_response(req, "At least one way idx is invalid",
                                  http::status::bad_request));
        }

        const auto hub = traversed_node_hub::from(
            w_, nodes_indices[0], way_indices[0], nodes_indices[1],
            way_indices[1], nodes_indices[2], m);

        return cb(json_response(req, hub.to_json(w_), http::status::ok));
      }

      return cb(
          json_response(req, "Parameters missing", http::status::bad_request));
    } catch (std::exception const& e) {
      return cb(
          json_response(req, e.what(), http::status::internal_server_error));
    }
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
    l_.find({min, max}, [&](way_idx_t const x) {
      auto const p = w_.r_->way_properties_[x];
      levels.emplace(p.from_level());
      if (p.from_level() != p.to_level()) {
        levels.emplace(p.to_level());
      }
    });
    auto levels_sorted =
        utl::to_vec(levels, [](level_t const l) { return l.to_float(); });
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
    auto const min =
        geo::latlng{waypoints[1].as_double(), waypoints[0].as_double()};
    auto const max =
        geo::latlng{waypoints[3].as_double(), waypoints[2].as_double()};

    auto gj = geojson_writer{.w_ = w_};
    l_.find({min, max}, [&](way_idx_t const w) { gj.write_way(w); });

    with_profile(profile,
                 [&]<Profile P>(P&&) { send_graph_response<P>(req, cb, gj); });
  }

  template <Profile P>
  void send_graph_response(web_server::http_req_t const& req,
                           web_server::http_res_cb_t const& cb,
                           geojson_writer& gj) {
    gj.finish(&get_dijkstra<P>());
    cb(json_response(req, gj.string()));
  }

  void handle_static(web_server::http_req_t const& req,
                     web_server::http_res_cb_t const& cb) {
    if (auto res = net::serve_static_file(
            boost::beast::string_view{static_file_path_}, req);
        res.has_value()) {
      cb(std::move(*res));
    } else {
      namespace http = boost::beast::http;
      cb(net::web_server::string_res_t{http::status::not_found, req.version()});
    }
  }

  void handle_platforms(web_server::http_req_t const& req,
                        web_server::http_res_cb_t const& cb) {
    utl::verify(pl_ != nullptr, "no platforms");

    auto const query = boost::json::parse(req.body()).as_object();
    auto const level = query.contains("level")
                           ? level_t{query.at("level").to_number<float>()}
                           : kNoLevel;
    auto const waypoints = query.at("waypoints").as_array();
    auto const min = point::from_latlng(
        {waypoints[1].as_double(), waypoints[0].as_double()});
    auto const max = point::from_latlng(
        {waypoints[3].as_double(), waypoints[2].as_double()});

    auto gj = geojson_writer{.w_ = w_, .platforms_ = pl_};
    pl_->find(min, max, [&](platform_idx_t const i) {
      if (level == kNoLevel || pl_->get_level(w_, i) == level) {
        gj.write_platform(i);
      }
    });

    cb(json_response(req, gj.string()));
  }

  void handle_request(web_server::http_req_t const& req,
                      web_server::http_res_cb_t const& cb) {
    std::cout << "[" << req.method_string() << "] " << req.target() << '\n';
    switch (req.method()) {
      case http::verb::options: return cb(json_response(req, {}));
      case http::verb::post: {
        auto const& target = req.target();
        if (provide_test_features_ &&
            target.starts_with("/api/test-framework/test-case")) {
          return handle_post_test_case(req, cb);
        }
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
        } else if (target.starts_with("/api/platforms")) {
          return run_parallel(
              [this](web_server::http_req_t const& req1,
                     web_server::http_res_cb_t const& cb1) {
                handle_platforms(req1, cb1);
              },
              req, cb);
        } else {
          return cb(json_response(req, R"({"error": "Not found"})",
                                  http::status::not_found));
        }
      }
      case http::verb::get: {
        auto const& target = req.target();
        if (provide_test_features_) {
          if (target.starts_with("/api/test-framework/test-cases")) {
            return run_parallel(
                [this](web_server::http_req_t const& req1,
                       web_server::http_res_cb_t const& cb1) {
                  handle_get_test_cases(req1, cb1);
                },
                req, cb);
          } else if (target.starts_with("/api/test-framework/feeds")) {
            return handle_get_available_feeds(req, cb);
          } else if (target.starts_with("/api/test-framework/hub")) {
            return run_parallel(
                [this](web_server::http_req_t const& req1,
                       web_server::http_res_cb_t const& cb1) {
                  handle_get_traversed_node_hub(req1, cb1);
                },
                req, cb);
          }
        }
      }
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
  platforms const* pl_;
  elevation_storage const* elevations_;
  web_server server_;
  bool serve_static_files_{false};
  std::string static_file_path_;
  bool provide_test_features_{false};
};

http_server::http_server(boost::asio::io_context& ioc,
                         boost::asio::io_context& thread_pool,
                         ways const& w,
                         lookup const& l,
                         platforms const* pl,
                         elevation_storage const* elevation,
                         std::string const& static_file_path,
                         bool const provide_test_features)
    : impl_{new impl(ioc, thread_pool, w, l, pl, elevation, static_file_path, provide_test_features)} {
}

http_server::~http_server() = default;

void http_server::listen(std::string const& host, std::string const& port) {
  impl_->listen(host, port);
}

void http_server::stop() { impl_->stop(); }

}  // namespace osr::backend
