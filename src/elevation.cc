#include "osr/elevation.h"
#include "osr/types.h"

namespace osr {

elevation::elevation(std::filesystem::path const& p,
            cista::mmap::protection const mode):
    p_{p},
    mode_{mode},
        elevation_up_m_{mm_vec<int>{mm("elevation_up_data.bin")}, mm_vec<unsigned>(mm("elevation_up_idx.bin"))},
        elevation_down_m_{mm_vec<int>{mm("elevation_down_data.bin")}, mm_vec<unsigned>(mm("elevation_down_idx.bin"))} {}
//   mm_vecvec<way_idx_t, int> elevation_up_m_;
//   mm_vecvec<way_idx_t, int> elevation_down_m_;

}