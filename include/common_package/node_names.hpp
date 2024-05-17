//
// Created by Johan <job8197@thi.de> on 17.05.2024.
//

#ifndef THI_DRONE_WS_NODE_NAMES_HPP
#define THI_DRONE_WS_NODE_NAMES_HPP

namespace common_lib::node_names {

constexpr char const *const MISSION_CONTROL =
    "mission_control"; /**< Node name for the mission control node */

constexpr char const *const WAYPOINT =
    "waypoint"; /**< Node name for the navigate to waypoint node */

constexpr char const *const QRCODE_DETECTION =
    "qrcode_detection"; /**< Node name for the qr code detection node */

constexpr char const *const FCC_BRIDGE =
    "fcc_bridge"; /**< Node name for the FCC bridge node */

constexpr char const *const TELEMETRY =
    "telemetry"; /**< Node name for the telemetry node */

}  // namespace common_lib::node_names

#endif  // THI_DRONE_WS_NODE_NAMES_HPP
