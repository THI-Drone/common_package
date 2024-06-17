#pragma once

namespace common_lib {
// Global Defines
constexpr uint16_t MAX_FLIGHT_HEIGHT_CM =
    120 /* [m] */ * 100;  //!< Maximum allowed flight height in cm
constexpr uint16_t MIN_CRUISE_HEIGHT_CM =
    5 /* [m] */ * 100;  //!< Minimum required cruise height in cm
constexpr float MAX_HORIZONTAL_SPEED_MPS =
    12.0 /* [m/s] */;  //!< Maximum allowed horizontal speed in m/s
constexpr float MAX_VERTICAL_SPEED_MPS =
    3.0 /* [m/s] */;  //!< Maximum allowed vertical speed in m/s
constexpr float MIN_SOC_PERCENT =
    20.0 /* [%] */;  //!< Minimum SOC after which a RTH is triggered
}  // namespace common_lib
