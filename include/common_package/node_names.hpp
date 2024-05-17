//
// Created by Johan <job8197@thi.de> on 17.05.2024.
//

#ifndef THI_DRONE_WS_NODE_NAMES_HPP
#define THI_DRONE_WS_NODE_NAMES_HPP

#include <set>

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

const std::set<std::string> VALID_CONTROL_NODE_NAMES{
    MISSION_CONTROL, WAYPOINT,
    QRCODE_DETECTION}; /**< Names of nodes that are allowed to be passed control
                        */

}  // namespace common_lib::node_names

#endif  // THI_DRONE_WS_NODE_NAMES_HPP
