#pragma once

#include <chrono>
#include <cinttypes>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "rclcpp/duration.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "topic_names.hpp"

// Message includes
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/job_finished.hpp"

using namespace std::chrono_literals;

namespace common_lib {

namespace {

/**
 * @brief rosout QoS to explicitly set the rosout for Nodes subclassing
 * CommonNode
 *
 * See:
 * 1.
 * https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1QoS.html#_CPPv4N6rclcpp3QoSE
 * 2.
 * https://docs.ros.org/en/humble/p/rclcpp/generated/structrclcpp_1_1QoSInitialization.html#_CPPv4N6rclcpp17QoSInitializationE
 * 3.
 * https://docs.ros.org/en/humble/p/rmw/generated/structrmw__qos__profile__s.html#_CPPv417rmw_qos_profile_s
 * 4. https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html
 */
const rclcpp::QoS ROSOUT_QOS{
    rclcpp::KeepLast(1), /**< Ignored */
    rmw_qos_profile_t{
        rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        1,
        rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE,
        rmw_time_t{0, 0},
        rmw_time_t{1, 0},
        rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        RMW_DURATION_INFINITE,
        false,
    } /**< This struct holds the default values for the QoS */
};

}  // namespace

/**
 * @class CommonNode
 * @brief A class representing a common node.
 *
 * This class inherits from rclcpp::Node and provides functionality for creating
 * a node that sends heartbeat messages.
 */
class CommonNode : public rclcpp::Node {
   public:
    /**
     * @brief Constructor for creating a new CommonNode.
     * @param id Unique name of the node.
     * @param options Optional NodeOptions
     *
     * @warning This constructor overrides the rosout QoS!
     */
    CommonNode(const std::string &id,
               rclcpp::NodeOptions options = rclcpp::NodeOptions())
        : Node(id, options.rosout_qos(ROSOUT_QOS)) {
        /// Create a publisher for the "heartbeat" topic
        heartbeat_publisher =
            this->create_publisher<interfaces::msg::Heartbeat>(
                topic_names::Heartbeat, 1);

        /// Create a timer that sends a heartbeat message every 500ms
        heartbeat_timer = this->create_wall_timer(
            std::chrono::milliseconds(heartbeat_period_ms),
            std::bind(&CommonNode::heartbeat_timer_callback, this));

        /// Create a publisher for the "job_finished" topic
        job_finished_publisher =
            this->create_publisher<interfaces::msg::JobFinished>(
                topic_names::JobFinished, 10);
    }

    /**
     * @brief Get the active status of the node.
     *
     * @return true if the node is active, false otherwise.
     */
    bool get_active() const { return node_active; }

   protected:
    /**
     * @brief Activates the node.
     *
     * This function sets the `node_active` flag to true, indicating that the
     * node is active.
     */
    void activate() {
        node_active = true;
        RCLCPP_DEBUG(this->get_logger(),
                     "CommonNode::activate: Activated node");
    }

    /**
     * @brief Deactivates the node.
     *
     * This function sets the `node_active` flag to false, indicating that the
     * node is no longer active.
     */
    void deactivate() {
        node_active = false;
        RCLCPP_DEBUG(this->get_logger(),
                     "CommonNode::deactivate: Deactivated node");
    }

    /**
     * @brief Handles the completion of a job.
     *
     * This function sends a job_finished message with the given error code and
     * payload. Additionally, deactivates the node.
     *
     * @param error_code The error code associated with the job completion
     * (EXIT_SUCCESS == 0 if no error).
     * @param payload The payload data associated with the job completion. Set
     * the "error_msg" key to your error message string if you want to have it
     * appear on the mission control output log.
     */
    void job_finished(const uint8_t error_code, const nlohmann::json &payload);

    /**
     * @brief Handles the completion of a job with the given error message. Can
     * be used if your job fails and you just want to return an error message.
     *
     * This function formats the error message to JSON and sends a job_finished
     * message with the error message. Additionally, deactivates the node. This
     * function should only be used if your job failed.
     *
     * @param error_message The error message associated with the job
     * completion.
     */
    void job_finished(const std::string &error_message);

    /**
     * @brief Sends a job_finished message with error code 0 and no error
     * message.
     *
     * This function should be executed only if your job was completed
     * successfully. Additionally, deactivates the node.
     */
    void job_finished();

   private:
    /**
     * @brief Callback function for the timer.
     */
    void heartbeat_timer_callback();

    /// Indicating if node is currently active and sending commands to interface
    /// node
    bool node_active = false;
    /// Heartbeat period in ms
    static constexpr uint16_t heartbeat_period_ms = 500;
    /// Timer for sending heartbeat messages
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    /// Publisher for the "heartbeat" topic
    rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr
        heartbeat_publisher;
    /// Tick counting upwards with every heartbeat
    interfaces::msg::Heartbeat::_tick_type heartbeat_tick = 0;

    /// Publisher for the "job_finished" topic
    rclcpp::Publisher<interfaces::msg::JobFinished>::SharedPtr
        job_finished_publisher;
};
}  // namespace common_lib
