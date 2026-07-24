#include "osr/bike_parkings.h"

#include "cista/io.h"

namespace fs = std::filesystem;

namespace osr {

cista::wrapped<bike_parking_data> bike_parking_data::read(
    fs::path const& p) {
  return cista::read<bike_parking_data>(p / "bike_parkings.bin");
}

void bike_parking_data::write(fs::path const& p) const {
  cista::write(p / "bike_parkings.bin", *this);
}

bike_parkings::bike_parkings(cista::wrapped<bike_parking_data>&& data)
    : data_{std::move(data)} {
  additional_node_coordinates_.assign(
      begin(data_->additional_node_coordinates_),
      end(data_->additional_node_coordinates_));
  for (auto const& e : data_->edges_) {
    additional_edges_[e.from_].push_back(
        additional_edge{.to_ = e.to_,
                        .distance_ = e.distance_,
                        .underlying_way_ = e.underlying_way_,
                        .reverse_ = e.reverse_});
  }
}

std::unique_ptr<bike_parkings> bike_parkings::try_open(fs::path const& path) {
  auto const file = path / "bike_parkings.bin";
  if (!fs::exists(file)) {
    return nullptr;
  }
  return std::make_unique<bike_parkings>(bike_parking_data::read(path));
}

sharing_data bike_parkings::get_sharing_data(
    node_idx_t::value_t const additional_node_offset) const {
  return {.start_allowed_ = &data_->start_allowed_,
          .end_allowed_ = &data_->end_allowed_,
          .through_allowed_ = nullptr,
          .additional_node_offset_ = additional_node_offset,
          .additional_node_coordinates_ = additional_node_coordinates_,
          .additional_edges_ = additional_edges_};
}

}  // namespace osr
