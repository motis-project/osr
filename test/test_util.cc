#include "test_util.h"

#include "osr/geojson.h"
#include "osr/extract/extract.h"
#include "osr/lookup.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

namespace fs = std::filesystem;

std::optional<osr::path> route(osr::ways const& w,
                               osr::lookup const& l,
                               osr::location const& from,
                               osr::location const& to,
                               osr::profile_parameters const& params,
                               osr::search_profile const sp) {
  return osr::route(
    params, w, l,
    sp, from, {to}, 900, osr::direction::kForward,
    250.0, nullptr, nullptr, nullptr, osr::routing_algorithm::kDijkstra);
}

std::optional<osr::path> extract_and_route(std::string_view path,
                              osr::location const& from,
                              osr::location const& to,
                              osr::profile_parameters const& params,
                              osr::search_profile const sp) {
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const p = route(w, l, from, to, params, sp);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return p;
}

std::string extract_route_to_feature(std::string_view path,
                                     osr::location const& from,
                                     osr::location const& to,
                                     osr::profile_parameters const& params,
                                     osr::search_profile const sp) {
  auto const dir = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);

  osr::extract(false, path, dir, {});

  const auto w = osr::ways{dir, cista::mmap::protection::READ};
  const auto l = osr::lookup{w, dir, cista::mmap::protection::READ};

  auto const p = route(w, l, from, to, params, sp);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return osr::to_featurecollection(w, *p, false);
}



