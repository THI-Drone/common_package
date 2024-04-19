#pragma once

#include <cinttypes>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/heartbeat.hpp"

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
    };
}
