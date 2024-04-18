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
    auto message = interfaces::msg::Heartbeat();
    message.sender_id = this->get_fully_qualified_name();
    message.active = node_active;
    message.tick = ++heartbeat_tick;
    message.time_stamp = this->now();
    heartbeat_publisher->publish(message);
}
