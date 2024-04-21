#include "common_package/common_node.hpp"

using namespace common_lib;

/**
 * @brief Callback function for the timer.
 *
 * This function is called when the timer expires. It creates a Heartbeat message,
 * sets the necessary fields, and publishes the message using the publisher.
 *
 * @return void
 */
void CommonNode::heartbeat_timer_callback()
{
    interfaces::msg::Heartbeat message;
    message.sender_id = this->get_fully_qualified_name();
    message.active = node_active;
    message.tick = ++heartbeat_tick;
    message.time_stamp = this->now();
    heartbeat_publisher->publish(message);
    RCLCPP_DEBUG(this->get_logger(), "CommonNode::heartbeat_timer_callback: Published heartbeat message with sender_id: %s, tick: %u, active: %d", this->get_fully_qualified_name(), message.tick, message.active);
}

/**
 * @brief Handles the completion of a job.
 *
 * This function sends a job_finished message with the given error code and payload.
 * Additionally, deactivates the node.
 *
 * @param error_code The error code associated with the job completion (EXIT_SUCCESS == 0 if no error).
 * @param payload The payload data associated with the job completion.
 */
void CommonNode::job_finished(const uint8_t error_code, const nlohmann::json &payload)
{
    interfaces::msg::JobFinished msg;

    msg.sender_id = this->get_fully_qualified_name();
    msg.error_code = error_code;
    msg.payload = payload.dump();

    job_finished_publisher->publish(msg);

    // Deactivate node
    deactivate();
}

/**
 * @brief Handles the completion of a job with the given error message. Can be used if your job fails and you just want to return an error message.
 *
 * This function formats the error message to JSON and sends a job_finished message with the error message.
 * Additionally, deactivates the node.
 * This function should only be used if your job failed.
 *
 * @param error_message The error message associated with the job completion.
 */
void CommonNode::job_finished(const std::string &error_message)
{
    // format error_message to JSON
    nlohmann::json payload;
    payload["error_msg"] = error_message;

    interfaces::msg::JobFinished msg;

    msg.sender_id = this->get_fully_qualified_name();
    msg.error_code = EXIT_FAILURE;
    msg.payload = payload.dump();

    job_finished_publisher->publish(msg);

    // Deactivate node
    deactivate();
}

/**
 * @brief Sends a job_finished message with error code 0 and no error message.
 *
 * This function should be executed only if your job was completed successfully.
 * Additionally, deactivates the node.
 */
void CommonNode::job_finished()
{
    interfaces::msg::JobFinished msg;

    msg.sender_id = this->get_fully_qualified_name();
    msg.error_code = EXIT_SUCCESS; // set error code to 0 to indicate success
    msg.payload = "";              // payload is empty

    job_finished_publisher->publish(msg);

    // Deactivate node
    deactivate();
}
