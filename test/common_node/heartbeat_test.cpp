#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

#include "common_package/common_node.hpp"

#include "interfaces/msg/heartbeat.hpp"

using namespace common_lib;

TEST(common_package, heartbeat_rate) {
  constexpr uint16_t heartbeat_period = 500;
  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<CommonNode> heartbeat_node =
      std::make_shared<CommonNode>("heartbeat");
  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

  interfaces::msg::Heartbeat last_msg;
  last_msg.tick = 0;
  last_msg.time_stamp = test_node->now();

  rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub =
      test_node->create_subscription<interfaces::msg::Heartbeat>(
          "heartbeat", 1,
          [&last_msg,
           test_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
            RCLCPP_DEBUG(test_node->get_logger(),
                         "Got message with tick: %" PRIu32, msg->tick);
            ASSERT_TRUE(
                rclcpp::Time(msg->time_stamp) - last_msg.time_stamp <=
                rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
                    heartbeat_period + 10)));
            ASSERT_EQ(last_msg.tick + 1, msg->tick);
            ASSERT_FALSE(msg->active);
            ASSERT_EQ(msg->sender_id, "/heartbeat");
            last_msg = *msg;
          });

  rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(
      std::chrono::milliseconds(heartbeat_period * 10),
      [&executor, test_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        executor.cancel();
      });

  executor.add_node(heartbeat_node);
  executor.add_node(test_node);

  executor.spin();
  ASSERT_EQ(last_msg.tick, 10);
}

TEST(common_package, heartbeat_activate_deactivate) {
  constexpr uint16_t heartbeat_period = 500;

  class OpenCommonNode : public CommonNode {
  public:
    OpenCommonNode(const std::string &name) : CommonNode(name) {}

    inline void activate() { CommonNode::activate(); }

    inline void deactivate() { CommonNode::deactivate(); }
  };

  rclcpp::executors::SingleThreadedExecutor executor;

  std::shared_ptr<OpenCommonNode> heartbeat_node =
      std::make_shared<OpenCommonNode>("heartbeat");
  executor.add_node(heartbeat_node);

  rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

  interfaces::msg::Heartbeat last_msg;
  last_msg.tick = 0;
  last_msg.time_stamp = test_node->now();
  last_msg.active = true;

  rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub =
      test_node->create_subscription<interfaces::msg::Heartbeat>(
          "heartbeat", 1,
          [&last_msg, test_node,
           heartbeat_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
            RCLCPP_DEBUG(test_node->get_logger(),
                         "Got message with tick: %" PRIu32, msg->tick);
            const rclcpp::Duration time_delta =
                rclcpp::Time(msg->time_stamp) - last_msg.time_stamp;
            ASSERT_TRUE(
                time_delta <=
                rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
                    heartbeat_period + 10)))
                << "Delta: " << time_delta.seconds() << "s "
                << time_delta.nanoseconds() << "ns";
            ASSERT_EQ(last_msg.tick + 1, msg->tick);
            ASSERT_EQ(msg->sender_id, "/heartbeat");
            ASSERT_EQ(msg->active, heartbeat_node->get_active());

            if (msg->active) {
              ASSERT_FALSE(last_msg.active)
                  << "Got unexpected active message with tick: " << msg->tick;
              heartbeat_node->deactivate();
            } else {
              ASSERT_TRUE(last_msg.active)
                  << "Got unexpected active message with seq: " << msg->tick;
              heartbeat_node->activate();
            }

            last_msg = *msg;
            ASSERT_EQ(last_msg.tick, msg->tick);
          });

  rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(
      std::chrono::milliseconds(heartbeat_period * 10),
      [&executor, test_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        executor.cancel();
      });

  executor.add_node(test_node);

  executor.spin();
  ASSERT_EQ(last_msg.tick, 10);
}
