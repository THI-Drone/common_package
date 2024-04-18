#include "common_package/common_node.hpp"


/**
 * @brief Callback function for the timer.
 * 
 * This function is called when the timer expires. It creates a Heartbeat message,
 * sets the necessary fields, and publishes the message using the publisher.
 * 
 * @return void
 */
void CommonNode::heartbeat_timer_callback() {
    interfaces::msg::Heartbeat message;
    message.sender_id = this->get_fully_qualified_name();
    message.active = node_active;
    message.tick = ++heartbeat_tick;
    message.time_stamp = this->now();
    heartbeat_publisher->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "CommonNode::heartbeat_timer_callback: Published heartbeat message with sender_id: %s, tick: %u, active: %d", this->get_fully_qualified_name(), message.tick, message.active);
}
