#pragma once

#include <cinttypes>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"

// Message includes
#include "interfaces/msg/heartbeat.hpp"
#include "interfaces/msg/job_finished.hpp"

using namespace std::chrono_literals;

namespace common_lib
{
    /**
     * @class CommonNode
     * @brief A class representing a common node.
     *
     * This class inherits from rclcpp::Node and provides functionality for creating a node that sends heartbeat messages.
     */
    class CommonNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for creating a new CommonNode.
         * @param id Unique name of the node.
         */
        CommonNode(const std::string &id) : Node(id)
        {
            // Create a publisher for the "heartbeat" topic
            heartbeat_publisher = this->create_publisher<interfaces::msg::Heartbeat>("heartbeat", 1);

            // Create a timer that sends a heartbeat message every 500ms
            heartbeat_timer = this->create_wall_timer(std::chrono::milliseconds(heartbeat_period_ms), std::bind(&CommonNode::heartbeat_timer_callback, this));

            // Create a publisher for the "job_finished" topic
            job_finished_publisher = this->create_publisher<interfaces::msg::JobFinished>("job_finished", 10);
        }

        /**
         * @brief Get the active status of the node.
         *
         * @return true if the node is active, false otherwise.
         */
        bool get_active() const
        {
            return node_active;
        }

    protected:
        /**
         * @brief Activates the node.
         *
         * This function sets the `node_active` flag to true, indicating that the node is active.
         */
        void activate()
        {
            node_active = true;
            RCLCPP_DEBUG(this->get_logger(), "CommonNode::activate: Activated node");
        }

        /**
         * @brief Deactivates the node.
         *
         * This function sets the `node_active` flag to false, indicating that the node is no longer active.
         */
        void deactivate()
        {
            node_active = false;
            RCLCPP_DEBUG(this->get_logger(), "CommonNode::deactivate: Deactivated node");
        }

        /**
         * @brief Handles the completion of a job.
         *
         * This function sends a job_finished message with the given error code and payload.
         * Additionally, deactivates the node.
         *
         * @param error_code The error code associated with the job completion (EXIT_SUCCESS == 0 if no error).
         * @param payload The payload data associated with the job completion. Set the "error_msg" key to your error message string if you want to have it appear on the mission control output log.
         */
        void job_finished(const uint8_t error_code, const nlohmann::json &payload);

        /**
         * @brief Handles the completion of a job with the given error message. Can be used if your job fails and you just want to return an error message.
         *
         * This function formats the error message to JSON and sends a job_finished message with the error message.
         * Additionally, deactivates the node.
         * This function should only be used if your job failed.
         *
         * @param error_message The error message associated with the job completion.
         */
        void job_finished(const std::string &error_message);

        /**
         * @brief Sends a job_finished message with error code 0 and no error message.
         *
         * This function should be executed only if your job was completed successfully.
         * Additionally, deactivates the node.
         */
        void job_finished();

    private:
        /**
         * @brief Callback function for the timer.
         */
        void heartbeat_timer_callback();

        bool node_active = false;                                                     ///< Indicating if node is currently active and sending commands to interface node
        static constexpr uint16_t heartbeat_period_ms = 500;                          ///< Heartbeat period in ms
        rclcpp::TimerBase::SharedPtr heartbeat_timer;                                 ///< Timer for sending heartbeat messages
        rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr heartbeat_publisher; ///< Publisher for the "heartbeat" topic
        interfaces::msg::Heartbeat::_tick_type heartbeat_tick = 0;                    ///< Tick counting upwards with every heartbeat

        rclcpp::Publisher<interfaces::msg::JobFinished>::SharedPtr job_finished_publisher; ///< Publisher for the "job_finished" topic
    };
}
