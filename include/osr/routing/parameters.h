#pragma once

#include <variant>

#include "osr/routing/profile.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/bike_sharing.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/car_sharing.h"
#include "osr/routing/profiles/ferry.h"
#include "osr/routing/profiles/foot.h"
#include "osr/routing/profiles/railway.h"

namespace osr {

using profile_parameters =
    std::variant<foot<false, noop_tracking>::parameters,
                 foot<true, noop_tracking>::parameters,
                 foot<false, elevator_tracking>::parameters,
                 foot<true, elevator_tracking>::parameters,
                 bike<bike_costing::kSafe, kElevationNoCost>::parameters,
                 bike<bike_costing::kFast, kElevationNoCost>::parameters,
                 bike<bike_costing::kSafe, kElevationLowCost>::parameters,
                 bike<bike_costing::kSafe, kElevationHighCost>::parameters,
                 car::parameters,
                 car_parking<false, false>::parameters,
                 car_parking<true, false>::parameters,
                 car_parking<false, true>::parameters,
                 car_parking<true, true>::parameters,
                 bike_sharing::parameters,
                 car_sharing<track_node_tracking>::parameters,
                 bus::parameters,
                 railway::parameters,
                 ferry::parameters>;

profile_parameters get_parameters(search_profile const p);

}  // namespace osr
