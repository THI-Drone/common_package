#pragma once

namespace common_lib {

namespace topic_names {
// Topic names by their message names

// FCC Bridge
constexpr const char* const BatteryState = "uav_battery_state";
constexpr const char* const GPSPosition = "uav_gps_position";
constexpr const char* const MissionProgress = "uav_mission_progress";
constexpr const char* const Pose = "uav_pose";
constexpr const char* const RCState = "uav_rc_state";
constexpr const char* const SafetyLimits = "uav_safety_limits";
constexpr const char* const UAVCommand = "uav_command";
constexpr const char* const UAVHealth = "uav_health";
constexpr const char* const UAVWaypointCommand = "uav_waypoint_command";
constexpr const char* const FlightState = "uav_flight_state";

// Mission Control
constexpr const char* const Control = "control";
constexpr const char* const Heartbeat = "heartbeat";
constexpr const char* const JobFinished = "job_finished";
constexpr const char* const MissionFinished = "mission_finished";
constexpr const char* const MissionStart = "mission_start";

// Marker Detection
constexpr const char* const QRCodeInfo = "qr_code_info";

}  // namespace topic_names

}  // namespace common_lib