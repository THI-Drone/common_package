#include "common_package/common_node.hpp"


/**
 * @brief Callback function for the timer.
 * 
 * This function is called when the timer expires. It creates a Heartbeat message,
 * sets the necessary fields, and publishes the message using the publisher.
 * 
 * @return void
 */
void CommonNode::timer_callback() {
    RCLCPP_DEBUG(this->get_logger(), "Heartbeat timer callback called");
    interfaces::msg::Heartbeat message;
    message.time_stamp = this->now();
    message.sender_id = this->get_fully_qualified_name();
    message.active = this->node_active;
    message.seq = this->heartbeat_tick++;
    this->heartbeat_publisher->publish(message);
    RCLCPP_INFO(this->get_logger(), "Send heartbeat with seq: %d and marked as: %s", message.seq, message.active ? "active" : "inactive");
}

/**
 * @brief Setter for active flag.
 *
 * This functions activates the node
 *
 * @return void
 */
void CommonNode::activate() {
    this->node_active = true;
    RCLCPP_DEBUG(this->get_logger(), "Activated node");
}

/**
 * @brief Setter for active flag.
 *
 * This functions deactivates the node
 *
 * @return void
 */
void CommonNode::deactivate() {
    this->node_active = false;
    RCLCPP_DEBUG(this->get_logger(), "Deactivated node");
}
