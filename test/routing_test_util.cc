#include "osr/routing/routing_test_util.h"

#include "osr/extract/extract.h"

namespace osr::test {

std::optional<path> route(ways const& w,
                          lookup const& l,
                          location const& from,
                          location const& to,
                          profile_parameters const& params,
                          search_profile const sp,
                          direction const dir,
                          routing_algorithm const algo) {
  return osr::route(params, w, l, sp, from, {to}, 900, dir,
                    250.0, nullptr, nullptr, nullptr, algo);
}

std::optional<path> extract_and_route(std::string_view path,
                                      location const& from,
                                      location const& to,
                                      profile_parameters const& params,
                                      search_profile const sp,
                                      direction const dir,
                                      routing_algorithm const algo) {
  auto const directory = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  std::filesystem::remove_all(directory, ec);
  std::filesystem::create_directories(directory, ec);

  extract(false, path, directory, {});

  const auto w = ways{directory, cista::mmap::protection::READ};
  const auto l = lookup{w, directory, cista::mmap::protection::READ};

  auto const p = route(w, l, from, to, params, sp, dir, algo);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return p;
}

std::string extract_route_to_feature(std::string_view path,
                                     location const& from,
                                     location const& to,
                                     profile_parameters const& params,
                                     search_profile const sp,
                                     direction const dir,
                                     routing_algorithm const algo) {
  auto const directory = fmt::format("/tmp/{}", path);
  auto ec = std::error_code{};
  std::filesystem::remove_all(directory, ec);
  std::filesystem::create_directories(directory, ec);

  extract(false, path, directory, {});

  const auto w = ways{directory, cista::mmap::protection::READ};
  const auto l = lookup{w, directory, cista::mmap::protection::READ};

  auto const p = route(w, l, from, to, params, sp, dir, algo);
  utl::verify(p.has_value(), "{}: from={} to={} -> no route", path,
              fmt::streamed(from), fmt::streamed(to));
  return to_featurecollection(w, *p, false);
}

}
