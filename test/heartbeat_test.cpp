#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/executors.hpp"

#include "interfaces/msg/heartbeat.hpp"

#include "common_package/common_node.hpp"

TEST(common_package, heartbeat_rate){
    constexpr int64_t heartbeat_period = 50;
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    // Lower heartbeat rate to accelerate test
    options.append_parameter_override("heartbeat_period", heartbeat_period);
    std::shared_ptr<common_package::CommonNode> heartbeat_node = std::make_shared<common_package::CommonNode>("heartbeat", options);
    executor.add_node(heartbeat_node);
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");
    interfaces::msg::Heartbeat last_msg;
    last_msg.seq = 0;
    last_msg.time_stamp = test_node->now();
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>("heartbeat", 1, [&last_msg, test_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
        RCLCPP_DEBUG(test_node->get_logger(), "Got message with seq: %" PRIu32, msg->seq);
        ASSERT_TRUE(rclcpp::Time(msg->time_stamp) - last_msg.time_stamp <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(heartbeat_period +10)));
        ASSERT_EQ(last_msg.seq + 1, msg->seq);
        ASSERT_FALSE(msg->active);
        ASSERT_EQ(msg->sender_id, "/heartbeat");
        last_msg = *msg;
    });
    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(std::chrono::milliseconds(500), [&executor, test_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        executor.cancel();
    });
    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(last_msg.seq, 10);
}

TEST(common_package, heartbeat_blocked){
    constexpr int64_t heartbeat_period = 50;
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    volatile bool keep_block = true;
    // Lower heartbeat rate to accelerate test
    options.append_parameter_override("heartbeat_period", heartbeat_period);
    std::shared_ptr<common_package::CommonNode> heartbeat_node = std::make_shared<common_package::CommonNode>("heartbeat", options);
    rclcpp::TimerBase::SharedPtr block_timer = heartbeat_node->create_wall_timer(std::chrono::milliseconds(75), [&keep_block, heartbeat_node](){
        RCLCPP_DEBUG(heartbeat_node->get_logger(), "Starting block");
        while(keep_block);
        RCLCPP_DEBUG(heartbeat_node->get_logger(), "Stopped block");
    }, heartbeat_node->critical_callbacks);
    executor.add_node(heartbeat_node);
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");
    uint16_t count = 0;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>("heartbeat", 1, [&count, test_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
        RCLCPP_DEBUG(test_node->get_logger(), "Got message with seq: %" PRIu32, msg->seq);
        count++;
        ASSERT_LE(count, 1) << "Block of heartbeat failed!";
    });
    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(std::chrono::milliseconds(500), [&executor, test_node, &keep_block]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        keep_block = false;
        executor.cancel();
    });
    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(count, 1);
}

TEST(common_package, heartbeat_multiple){
    constexpr int64_t heartbeat_period = 50;
    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::NodeOptions node_options_0;
    node_options_0.append_parameter_override("heartbeat_period", heartbeat_period);
    std::shared_ptr<common_package::CommonNode> heartbeat_node_0 = std::make_shared<common_package::CommonNode>("heartbeat_0", node_options_0);
    executor.add_node(heartbeat_node_0);

    rclcpp::NodeOptions node_options_1;
    node_options_1.append_parameter_override("heartbeat_period", heartbeat_period * 2);
    std::shared_ptr<common_package::CommonNode> heartbeat_node_1 = std::make_shared<common_package::CommonNode>("heartbeat_1", node_options_1);
    executor.add_node(heartbeat_node_1);

    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");
    uint16_t count_0 = 0;
    rclcpp::Time last_msg_0 = test_node->now();
    uint16_t count_1 = 0;
    rclcpp::Time last_msg_1 = test_node->now();
    rclcpp::CallbackGroup::SharedPtr test_callback_group = test_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>> subscription_options;
    subscription_options.callback_group = test_callback_group;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>("heartbeat", 1, [&count_0, &last_msg_0, &count_1, &last_msg_1, test_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
        RCLCPP_DEBUG(test_node->get_logger(), "Got message with seq: %" PRIu32, msg->seq);
        if(msg->sender_id == "/heartbeat_0") {
            ASSERT_EQ(count_0 + 1, msg->seq);
            count_0++;
            ASSERT_TRUE(rclcpp::Time(msg->time_stamp) - last_msg_0 <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(heartbeat_period + 10)));
            last_msg_0 = msg->time_stamp;
        } else if(msg->sender_id == "/heartbeat_1") {
            ASSERT_EQ(count_1 + 1, msg->seq);
            count_1++;
            ASSERT_TRUE(rclcpp::Time(msg->time_stamp) - last_msg_1 <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(2 * heartbeat_period + 10)));
            last_msg_1 = msg->time_stamp;
        } else {
            FAIL() << "Got unexpected sender id: " << msg->sender_id;
        }
    }, subscription_options);
    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(std::chrono::milliseconds(500), [&executor, test_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        executor.cancel();
    });
    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(count_0, 10);
    ASSERT_EQ(count_1, 5);
}

TEST(common_package, heartbeat_activate_deactivate){
    constexpr int64_t heartbeat_period = 50;

    class OpenCommonNode : public common_package::CommonNode {
    public:
        OpenCommonNode(const std::string &name, const rclcpp::NodeOptions options = rclcpp::NodeOptions()) : common_package::CommonNode(name, options) {

        }

        inline void activate() {
            common_package::CommonNode::activate();
        }

        inline void deactivate() {
            common_package::CommonNode::deactivate();
        }
    };

    rclcpp::executors::MultiThreadedExecutor executor;

    rclcpp::NodeOptions options;
    options.append_parameter_override("heartbeat_period", heartbeat_period);
    std::shared_ptr<OpenCommonNode> heartbeat_node = std::make_shared<OpenCommonNode>("heartbeat", options);
    executor.add_node(heartbeat_node);
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");
    interfaces::msg::Heartbeat last_msg;
    last_msg.seq = 0;
    last_msg.time_stamp = test_node->now();
    last_msg.active = true;
    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>("heartbeat", 1, [&last_msg, test_node, heartbeat_node](interfaces::msg::Heartbeat::ConstSharedPtr msg) {
        RCLCPP_DEBUG(test_node->get_logger(), "Got message with seq: %" PRIu32, msg->seq);
        const rclcpp::Duration time_delta = rclcpp::Time(msg->time_stamp) - last_msg.time_stamp;
        ASSERT_TRUE(time_delta <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(heartbeat_period +10))) << "Delta: " << time_delta.seconds() << "s " << time_delta.nanoseconds() << "ns";
        ASSERT_EQ(last_msg.seq + 1, msg->seq);
        ASSERT_EQ(msg->sender_id, "/heartbeat");
        if(msg->active) {
            ASSERT_FALSE(last_msg.active) << "Got unexpected active message with seq: " << msg->seq;
            heartbeat_node->deactivate();
        } else {
            ASSERT_TRUE(last_msg.active) << "Got unexpected active message with seq: " << msg->seq;
            heartbeat_node->activate();
        }
        last_msg = *msg;
        ASSERT_EQ(last_msg.seq, msg->seq);
    });
    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(std::chrono::milliseconds(500), [&executor, test_node]() {
        RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
        executor.cancel();
    });
    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(last_msg.seq, 10);
}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);
    testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}