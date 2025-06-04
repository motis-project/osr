#include "osr/preprocessing/elevation/hgt_tile.h"
#include "osr/preprocessing/elevation/hgt_tile_def.h"

namespace osr::preprocessing::elevation {

template struct hgt_tile<3601U>;
template struct hgt_tile<1201U>;

}  // namespace osr::preprocessing::elevation
