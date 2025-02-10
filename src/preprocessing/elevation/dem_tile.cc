#include "osr/preprocessing/elevation/dem_tile.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "boost/algorithm/string/case_conv.hpp"

#include "cista/mmap.h"
#include "cista/strong.h"

#include "osr/preprocessing/elevation/shared.h"

// EHdr / BIL File Format:
// http://www.gdal.org/frmt_various.html#EHdr
// http://downloads.esri.com/support/whitepapers/other_/eximgav.pdf
// http://desktop.arcgis.com/en/arcmap/10.3/manage-data/raster-and-images/bil-bip-and-bsq-raster-files.htm

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

constexpr auto const kNoData = std::int16_t{-32768};

using str_map = std::unordered_map<std::string, std::string>;
using dem_exception = std::runtime_error;

str_map read_hdr_file(fs::path const& path) {
  str_map map;

  auto f = std::ifstream{path};
  while (f.good()) {
    std::string key, value;
    f >> key >> value;
    if (f) {
      boost::to_upper(key);
      boost::to_upper(value);
      map[key] = value;
    }
  }

  return map;
}

std::string get_string(str_map const& map,
                       std::string const& key,
                       std::string const& def = "") {
  auto const r = map.find(key);
  if (r != end(map)) {
    auto const& val = r->second;
    return val.empty() ? def : val;
  } else {
    return def;
  }
}

int get_int(str_map const& map, std::string const& key, int def) {
  auto const str = get_string(map, key);
  return str.empty() ? def : std::stoi(str);
}

unsigned get_uint(str_map const& map, std::string const& key, unsigned def) {
  auto const str = get_string(map, key);
  return str.empty() ? def : static_cast<unsigned>(std::stoul(str));
}

double get_double(str_map const& map, std::string const& key, double def) {
  auto const str = get_string(map, key);
  return str.empty() ? def : std::stod(str);
}

struct bil_header {
  explicit bil_header(fs::path const& bil_path) {
    auto const hdr_path = fs::path{bil_path.parent_path() /
                                   fs::path(bil_path.stem().string() + ".hdr")};
    if (!fs::exists(hdr_path)) {
      throw dem_exception("Missing hdr file: " + hdr_path.string());
    }
    auto const hdr = read_hdr_file(hdr_path);
    init_hdr(hdr);
  }

  void init_hdr(str_map const& hdr) {
    rows_ = get_uint(hdr, "NROWS", 0);
    cols_ = get_uint(hdr, "NCOLS", 0);
    if (rows_ == 0 || cols_ == 0) {
      throw dem_exception("Missing nrows/ncols");
    }

    if (get_uint(hdr, "NBANDS", 1) != 1) {
      throw dem_exception("Unsupported nbands value");
    }
    if (get_string(hdr, "BYTEORDER", "I") != "I") {
      throw dem_exception("Unsupported byte order");
    }
    if (get_uint(hdr, "SKIPBYTES", 0) != 0) {
      throw dem_exception("Unsupported skipbytes");
    }

    auto const nbits = get_uint(hdr, "NBITS", 8);
    auto const pixeltype = get_string(hdr, "PIXELTYPE", "UNSIGNEDINT");
    if (nbits == 16 && pixeltype[0] == 'S') {
      pixel_type_ = pixel_type::int16;
    } else if (nbits == 32 && pixeltype[0] == 'F') {
      pixel_type_ = pixel_type::float32;
    } else {
      throw dem_exception("Unsupported pixeltype");
    }
    pixel_size_ = nbits / 8;
    row_size_ = pixel_size_ * cols_;

    ulx_ = get_double(hdr, "ULXMAP", std::numeric_limits<double>::min());
    uly_ = get_double(hdr, "ULYMAP", std::numeric_limits<double>::min());
    if (std::equal_to<>()(ulx_, std::numeric_limits<double>::min()) ||
        std::equal_to<>()(uly_, std::numeric_limits<double>::min())) {
      throw dem_exception("Missing ulxmap/ulymap");
    }

    xdim_ = get_double(hdr, "XDIM", 0);
    ydim_ = get_double(hdr, "YDIM", 0);
    if (std::equal_to<>()(xdim_, 0.0) || std::equal_to<>()(ydim_, 0.0)) {
      throw dem_exception("Missing xdim/ydim");
    }
    brx_ = ulx_ + cols_ * xdim_;
    bry_ = uly_ - rows_ * ydim_;

    switch (pixel_type_) {
      case pixel_type::int16:
        nodata_.int16_ = static_cast<std::int16_t>(get_int(hdr, "NODATA", 0));
        break;
      case pixel_type::float32:
        nodata_.float32_ = static_cast<float>(get_double(hdr, "NODATA", 0));
        break;
    }

    auto const bandrowbytes = get_uint(hdr, "BANDROWBYTES", row_size_);
    auto const totalrowbytes = get_uint(hdr, "TOTALROWBYTES", row_size_);
    if (bandrowbytes != row_size_ || totalrowbytes != row_size_) {
      throw dem_exception("Unsupported bandrowbytes/totalrowbytes");
    }
  }

  unsigned rows_{0};
  unsigned cols_{0};
  double ulx_{0};  // upper left lon
  double uly_{0};  // upper left lat
  double brx_{0};  // bottom right lon
  double bry_{0};  // bottom right lat
  double xdim_{0};  // x pixel dimension, degrees
  double ydim_{0};  // y pixel dimension, degrees
  unsigned pixel_size_{0};  // bytes per pixel
  unsigned row_size_{0};  // bytes per row
  pixel_type pixel_type_{pixel_type::int16};
  pixel_value nodata_{};
};

fs::path get_bil_path(fs::path const& path) {
  auto const bil_path =
      fs::path{path.parent_path() / fs::path(path.stem().string() + ".bil")};
  if (!fs::exists(bil_path)) {
    throw dem_exception("Missing bil file: " + bil_path.string());
  }
  return bil_path;
}

struct dem_tile::impl {
  explicit impl(fs::path const& path)
      : data_file_{get_bil_path(path)},
        hdr_{data_file_},
        mapped_file_{cista::mmap{data_file_.string().data(),
                                 cista::mmap::protection::READ}} {}

  pixel_value get(point const p) const {
    auto const lng = p.lng();
    auto const lat = p.lat();

    if (lng < hdr_.ulx_ || lat > hdr_.uly_ || lng >= hdr_.brx_ ||
        lat <= hdr_.bry_) {
      return hdr_.nodata_;
    }

    auto const pix_x = static_cast<unsigned>((lng - hdr_.ulx_) / hdr_.xdim_);
    auto const pix_y = static_cast<unsigned>((hdr_.uly_ - lat) / hdr_.ydim_);
    assert(pix_x < hdr_.cols_);
    assert(pix_y < hdr_.rows_);
    auto const byte_pos = hdr_.row_size_ * pix_y + hdr_.pixel_size_ * pix_x;

    assert(byte_pos < mapped_file_.size());
    auto const* byte_ptr = mapped_file_.data() + byte_pos;

    pixel_value val{};

    switch (hdr_.pixel_type_) {
      case pixel_type::int16:
        val.int16_ = *reinterpret_cast<std::int16_t const*>(byte_ptr);
        break;
      case pixel_type::float32:
        val.float32_ = *reinterpret_cast<float const*>(byte_ptr);
        break;
    }

    return val;
  }

  fs::path data_file_;
  bil_header hdr_;
  cista::mmap mapped_file_;
};

dem_tile::dem_tile(fs::path const& path)
    : impl_(std::make_unique<impl>(path)) {}

dem_tile::dem_tile(dem_tile&& grid) noexcept : impl_(std::move(grid.impl_)) {}

dem_tile::~dem_tile() = default;

elevation_meters_t dem_tile::get(point const p) const {
  auto const val = get_raw(p);
  switch (impl_->hdr_.pixel_type_) {
    case pixel_type::int16:
      if (val.int16_ == impl_->hdr_.nodata_.int16_) {
        return elevation_meters_t::invalid();
      } else {
        auto const value = val.int16_;
        return value != kNoData ? elevation_meters_t{value}
                                : elevation_meters_t::invalid();
      }
    case pixel_type::float32:
      if (std::equal_to<>()(val.float32_, impl_->hdr_.nodata_.float32_)) {
        return elevation_meters_t::invalid();
      } else {
        auto const value = static_cast<cista::base_t<elevation_meters_t>>(
            std::round(val.float32_));
        return value != kNoData ? elevation_meters_t{value}
                                : elevation_meters_t::invalid();
      }
  }
  throw std::runtime_error{"dem_grid: invalid pixel type"};
}

tile_idx_t dem_tile::tile_idx(point const) const {
  // TODO Return non trivial sub tile index
  return tile_idx_t::from_sub_tile(0U);
}

geo::box dem_tile::get_box() const {
  return {
      {
          impl_->hdr_.bry_,
          impl_->hdr_.ulx_,
      },
      {
          impl_->hdr_.uly_,
          impl_->hdr_.brx_,
      },
  };
}

pixel_value dem_tile::get_raw(point const p) const { return impl_->get(p); }

pixel_type dem_tile::get_pixel_type() const { return impl_->hdr_.pixel_type_; }

resolution dem_tile::max_resolution() const {
  return {.x_ = impl_->hdr_.xdim_, .y_ = impl_->hdr_.ydim_};
}

}  // namespace osr::preprocessing::elevation
