#include <cmath>
#include <fstream>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "boost/algorithm/string/case_conv.hpp"

#include "cista/mmap.h"

#include "osr/preprocessing/elevation/dem_grid.h"

// EHdr / BIL File Format:
// http://www.gdal.org/frmt_various.html#EHdr
// http://downloads.esri.com/support/whitepapers/other_/eximgav.pdf
// http://desktop.arcgis.com/en/arcmap/10.3/manage-data/raster-and-images/bil-bip-and-bsq-raster-files.htm

namespace fs = std::filesystem;

namespace osr::preprocessing::elevation {

using str_map = std::unordered_map<std::string, std::string>;
using dem_exception = std::runtime_error;

str_map read_hdr_file(std::string const& filename) {
  str_map map;

  std::ifstream f(filename);
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
  if (str.empty()) {
    return def;
  } else {
    return std::stoi(str);
  }
}

unsigned get_uint(str_map const& map, std::string const& key, unsigned def) {
  auto const str = get_string(map, key);
  if (str.empty()) {
    return def;
  } else {
    return static_cast<unsigned>(std::stoul(str));
  }
}

double get_double(str_map const& map, std::string const& key, double def) {
  auto const str = get_string(map, key);
  if (str.empty()) {
    return def;
  } else {
    return std::stod(str);
  }
}

struct dem_grid::impl {
  explicit impl(std::string const& filename) : mapped_file_{} {
    auto const path = fs::path{filename};
    auto const hdr_path =
        fs::path{path.parent_path() / fs::path(path.stem().string() + ".hdr")};
    auto const bil_path =
        fs::path{path.parent_path() / fs::path(path.stem().string() + ".bil")};
    if (!fs::exists(hdr_path)) {
      throw dem_exception("Missing hdr file: " + hdr_path.string());
    }
    if (!fs::exists(bil_path)) {
      throw dem_exception("Missing bil file: " + bil_path.string());
    }
    auto const hdr = read_hdr_file(hdr_path.string());
    init_hdr(hdr);
    data_file_ = bil_path.string();
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
        nodata_.int16_ = static_cast<int16_t>(get_int(hdr, "NODATA", 0));
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

  pixel_value get(location const& loc) {
    auto const lon = loc.lon();
    auto const lat = loc.lat();

    if (lon < ulx_ || lat > uly_ || lon > brx_ || lat < bry_) {
      return nodata_;
    }

    auto const pix_x = static_cast<unsigned>((lon - ulx_) / xdim_);
    auto const pix_y = static_cast<unsigned>((uly_ - lat) / ydim_);
    assert(pix_x < cols_);
    assert(pix_y < rows_);
    auto const byte_pos = row_size_ * pix_y + pixel_size_ * pix_x;

    ensure_file_mapped();
    assert(byte_pos < mapped_file_->size());
    auto const* byte_ptr = mapped_file_->data() + byte_pos;

    pixel_value val{};

    switch (pixel_type_) {
      case pixel_type::int16:
        val.int16_ = *reinterpret_cast<int16_t const*>(byte_ptr);
        break;
      case pixel_type::float32:
        val.float32_ = *reinterpret_cast<float const*>(byte_ptr);
        break;
    }

    return val;
  }

  void ensure_file_mapped() {
    if (!mapped_file_.has_value()) {
      std::clog << "Using DEM grid file: " << data_file_ << std::endl;
      mapped_file_ = cista::mmap{data_file_.data(), cista::mmap::protection::READ};
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

  std::string data_file_;
  std::optional<cista::mmap> mapped_file_;
};

dem_grid::dem_grid(std::string const& filename)
    : impl_(std::make_unique<impl>(filename)) {}

dem_grid::dem_grid(dem_grid&& grid) noexcept : impl_(std::move(grid.impl_)) {}

dem_grid::~dem_grid() = default;

elevation_t dem_grid::get(location const& loc) const {
  auto const val = get_raw(loc);
  switch (impl_->pixel_type_) {
    case pixel_type::int16:
      if (val.int16_ == impl_->nodata_.int16_) {
        return NO_ELEVATION_DATA;
      } else {
        return static_cast<elevation_t>(val.int16_);
      }
    case pixel_type::float32:
      if (std::equal_to<>()(val.float32_, impl_->nodata_.float32_)) {
        return NO_ELEVATION_DATA;
      } else {
        return static_cast<elevation_t>(std::round(val.float32_));
      }
  }
  throw std::runtime_error{"dem_grid: invalid pixel type"};
}

pixel_value dem_grid::get_raw(location const& loc) const {
  return impl_->get(loc);
}

pixel_type dem_grid::get_pixel_type() const { return impl_->pixel_type_; }

}  // namespace osr::preprocessing::elevation
