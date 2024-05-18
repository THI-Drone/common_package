//
// Created by Johan <job8197@thi.de> on 17.05.2024.
//

#pragma once

#include <set>
#include <string>

namespace common_lib::node_names {

constexpr char const *const MISSION_CONTROL =
    "mission_control";  //!< Node name for the mission control node

constexpr char const *const WAYPOINT =
    "waypoint_node";  //!< Node name for the navigate to waypoint node

constexpr char const *const QRCODE_SCANNER =
    "qr_code_scanner_node";  //!< Node name for the qr code scanner node

constexpr char const *const FCC_BRIDGE =
    "fcc_bridge";  //!< Node name for the FCC bridge node

const std::set<std::string> VALID_CONTROL_NODE_NAMES{
    MISSION_CONTROL, WAYPOINT,
    QRCODE_SCANNER};  //!< Names of nodes that are allowed to be passed
                      //!< control

}  // namespace common_lib::node_names
