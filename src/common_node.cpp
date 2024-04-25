#include "common_package/common_node.hpp"

using namespace common_lib;

/**
 * @brief Callback function for the timer.
 *
 * This function is called when the timer expires. It creates a Heartbeat
 * message, sets the necessary fields, and publishes the message using the
 * publisher.
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
  RCLCPP_DEBUG(this->get_logger(),
               "CommonNode::heartbeat_timer_callback: Published heartbeat "
               "message with sender_id: %s, tick: %u, active: %d",
               this->get_fully_qualified_name(), message.tick, message.active);
}

/**
 * @brief Handles the completion of a job.
 *
 * This function sends a job_finished message with the given error code and
 * payload. Additionally, deactivates the node.
 *
 * @param error_code The error code associated with the job completion
 * (EXIT_SUCCESS == 0 if no error).
 * @param payload The payload data associated with the job completion. Set the
 * "error_msg" key to your error message string if you want to have it appear on
 * the mission control output log.
 */
void CommonNode::job_finished(const uint8_t error_code,
                              const nlohmann::json &payload) {
  interfaces::msg::JobFinished msg;

  msg.sender_id = this->get_fully_qualified_name();
  msg.error_code = error_code;

  try {
    msg.payload = payload.dump();
  } catch (const nlohmann::json::parse_error &e) {
    RCLCPP_FATAL(this->get_logger(), "CommonNode::job_finished: Payload is not "
                                     "a valid JSON. Stopping node.");
    exit(EXIT_FAILURE);
  }

  job_finished_publisher->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
               "CommonNode::job_finished: Sent job_finished message with "
               "custom error_code: %" PRIu8 " and payload",
               error_code);

  // Deactivate node
  deactivate();
}

/**
 * @brief Handles the completion of a job with the given error message. Can be
 * used if your job fails and you just want to return an error message.
 *
 * This function formats the error message to JSON and sends a job_finished
 * message with the error message. Additionally, deactivates the node. This
 * function should only be used if your job failed.
 *
 * @param error_message The error message associated with the job completion.
 */
void CommonNode::job_finished(const std::string &error_message) {
  // format error_message to JSON
  nlohmann::json payload;
  payload["error_msg"] = error_message;

  interfaces::msg::JobFinished msg;

  msg.sender_id = this->get_fully_qualified_name();
  msg.error_code = EXIT_FAILURE;

  try {
    msg.payload = payload.dump();
  } catch (const nlohmann::json::parse_error &e) {
    RCLCPP_FATAL(this->get_logger(), "CommonNode::job_finished: Payload is not "
                                     "a valid JSON. Stopping node.");
    exit(EXIT_FAILURE);
  }

  job_finished_publisher->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
               "CommonNode::job_finished: Sent job_finished message with error "
               "message: '%s' and EXIT_FAILURE error code",
               error_message.c_str());

  // Deactivate node
  deactivate();
}

/**
 * @brief Sends a job_finished message with error code 0 and no error message.
 *
 * This function should be executed only if your job was completed successfully.
 * Additionally, deactivates the node.
 */
void CommonNode::job_finished() {
  interfaces::msg::JobFinished msg;

  msg.sender_id = this->get_fully_qualified_name();
  msg.error_code =
      EXIT_SUCCESS;   // set error code to EXIT_SUCCESS to indicate success
  msg.payload = "{}"; // payload is empty JSON

  job_finished_publisher->publish(msg);
  RCLCPP_DEBUG(
      this->get_logger(),
      "CommonNode::job_finished: Sent job_finished message indicating success");

  // Deactivate node
  deactivate();
}
