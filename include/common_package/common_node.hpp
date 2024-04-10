#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/Heartbeat.hpp"

using namespace std::chrono_literals;

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
  CommonNode(char* id) : Node(id) {
    this->id = id;

    // Create a publisher for the "heartbeat" topic
    publisher_ = this->create_publisher<interfaces::msg::Heartbeat>("heartbeat", 10);

    // Create a timer that sends a heartbeat message every 500ms
    timer_ = this->create_wall_timer(500ms, std::bind(&CommonNode::timer_callback, this));
  }
  
  /// @brief Getter for the node id
  /// @return Node id as char*
  char* get_id() const {
    return id;
  }

protected:
  bool active = false;  ///< Indicating if node is currently active and sending commands to interface node

private:
  /**
   * @brief Callback function for the timer.
   */
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for sending heartbeat messages
  rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr publisher_;  ///< Publisher for the "heartbeat" topic
  uint32_t tick_ = 0;  ///< Tick counting upwards with every heartbeat
  char* id;  ///< Unique name of the node
};
