#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cinttypes>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"

#include "interfaces/msg/heartbeat.hpp"

/*
 * @class CommonNode
 * @brief A class representing a common node.
 *
 * This class inherits from rclcpp::Node and provides functionality for creating a node that sends heartbeat messages.
 */
class CommonNode : public rclcpp::Node {
public:
    /*
     * @brief Constructor for creating a new CommonNode.
     * @param id Unique name of the node.
     * @param options Optional options for the Node
     */
    CommonNode(const std::string &id, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node(id, options) {
        const uint32_t heartbeat_rate = this->declare_parameter("heartbeat_rate", 500);
        RCLCPP_DEBUG(this->get_logger(), "Got heartbeat rate: %" PRIu32, heartbeat_rate);

        // Create a publisher for the "heartbeat" topic
        publisher_ = this->create_publisher<interfaces::msg::Heartbeat>("heartbeat", 10);
        RCLCPP_DEBUG(this->get_logger(), "Create heartbeat publisher");

        // Create a timer that sends a heartbeat message every 500ms
        timer_ = this->create_wall_timer(std::chrono::milliseconds(heartbeat_rate), std::bind(&CommonNode::timer_callback, this));
        RCLCPP_DEBUG(this->get_logger(), "Created heartbeat timer");
    }

protected:
    /*
     * @brief Setter for active
     */
    void activate();

    /*
     * @brief Setter for active
     */
    void deactivate();

private:
    /*
     * @brief Callback function for the timer.
     */
    void timer_callback();

    bool active = false;  ///< Indicating if node is currently active and sending commands to interface node

    rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for sending heartbeat messages
    rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr publisher_;  ///< Publisher for the "heartbeat" topic
    interfaces::msg::Heartbeat::_seq_type tick_ = 0;  ///< Tick counting upwards with every heartbeat
};
