#include "osr/preprocessing/elevation/hgt.h"
#include "osr/preprocessing/elevation/hgt_def.h"

namespace osr::preprocessing::elevation {

template struct hgt<3601U>;
template struct hgt<1201U>;

}  // namespace osr::preprocessing::elevation
