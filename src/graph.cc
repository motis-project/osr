#include "osr/graph.h"

#include "cista/mmap.h"
#include "cista/serialization.h"

#include "utl/overloaded.h"

namespace osr {

constexpr auto const kMode =
    cista::mode::WITH_INTEGRITY | cista::mode::WITH_STATIC_VERSION;

edge_idx_t graph::add_edge(node_idx_t const from,
                           node_idx_t const to,
                           edge_flags_t const f,
                           distance_t const distance) {
  auto const idx = edge_idx_t{edge_flags_.size()};
  edge_flags_.emplace_back(f);
  edge_distance_.emplace_back(distance);
  edge_from_.emplace_back(from);
  edge_to_.emplace_back(to);
  return idx;
}

cista::wrapped<graph> graph::read(cista::memory_holder&& mem) {
  return std::visit(
      utl::overloaded{[&](cista::buf<cista::mmap>& b) {
                        auto const ptr = reinterpret_cast<graph*>(
                            &b[cista::data_start(kMode)]);
                        return cista::wrapped{std::move(mem), ptr};
                      },
                      [&](cista::buffer& b) {
                        auto const ptr = cista::deserialize<graph, kMode>(b);
                        return cista::wrapped{std::move(mem), ptr};
                      },
                      [&](cista::byte_buf& b) {
                        auto const ptr = cista::deserialize<graph, kMode>(b);
                        return cista::wrapped{std::move(mem), ptr};
                      }},
      mem);
}

void graph::write(std::filesystem::path const& p) const {
  auto mmap = cista::mmap{p.string().c_str(), cista::mmap::protection::WRITE};
  auto writer = cista::buf<cista::mmap>(std::move(mmap));

  {
    //    auto const timer = scoped_timer{"graph.write"};
    cista::serialize<kMode>(writer, *this);
  }
}

void graph::write(cista::memory_holder& mem) const {
  std::visit(utl::overloaded{[&](cista::buf<cista::mmap>& writer) {
                               cista::serialize<kMode>(writer, *this);
                             },
                             [&](cista::buffer&) {
                               throw std::runtime_error{"not supported"};
                             },
                             [&](cista::byte_buf& b) {
                               auto writer = cista::buf{std::move(b)};
                               cista::serialize<kMode>(writer, *this);
                               b = std::move(writer.buf_);
                             }},
             mem);
}

}  // namespace osr