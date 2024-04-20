#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"

#include "common_package/common_node.hpp"

#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/heartbeat.hpp"

using namespace common_lib;

TEST(common_package, heartbeat_rate)
{
    constexpr uint16_t heartbeat_period = 500;
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<CommonNode> heartbeat_node = std::make_shared<CommonNode>("heartbeat");
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    interfaces::msg::Heartbeat last_msg;
    last_msg.tick = 0;
    last_msg.time_stamp = test_node->now();

    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>(
        "heartbeat",
        1,
        [&last_msg, test_node](interfaces::msg::Heartbeat::ConstSharedPtr msg)
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Got message with tick: %" PRIu32, msg->tick);
            ASSERT_TRUE(rclcpp::Time(msg->time_stamp) - last_msg.time_stamp <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(heartbeat_period + 10)));
            ASSERT_EQ(last_msg.tick + 1, msg->tick);
            ASSERT_FALSE(msg->active);
            ASSERT_EQ(msg->sender_id, "/heartbeat");
            last_msg = *msg; });

    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(heartbeat_period * 10), [&executor, test_node]()
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
            executor.cancel(); });

    executor.add_node(heartbeat_node);
    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(last_msg.tick, 10);
}

TEST(common_package, heartbeat_activate_deactivate)
{
    constexpr uint16_t heartbeat_period = 500;

    class OpenCommonNode : public CommonNode
    {
    public:
        OpenCommonNode(const std::string &name) : CommonNode(name)
        {
        }

        inline void activate()
        {
            CommonNode::activate();
        }

        inline void deactivate()
        {
            CommonNode::deactivate();
        }
    };

    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<OpenCommonNode> heartbeat_node = std::make_shared<OpenCommonNode>("heartbeat");
    executor.add_node(heartbeat_node);

    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    interfaces::msg::Heartbeat last_msg;
    last_msg.tick = 0;
    last_msg.time_stamp = test_node->now();
    last_msg.active = true;

    rclcpp::Subscription<interfaces::msg::Heartbeat>::SharedPtr heartbeat_sub = test_node->create_subscription<interfaces::msg::Heartbeat>(
        "heartbeat",
        1,
        [&last_msg, test_node, heartbeat_node](interfaces::msg::Heartbeat::ConstSharedPtr msg)
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Got message with tick: %" PRIu32, msg->tick);
            const rclcpp::Duration time_delta = rclcpp::Time(msg->time_stamp) - last_msg.time_stamp;
            ASSERT_TRUE(time_delta <= rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(heartbeat_period +10))) << "Delta: " << time_delta.seconds() << "s " << time_delta.nanoseconds() << "ns";
            ASSERT_EQ(last_msg.tick + 1, msg->tick);
            ASSERT_EQ(msg->sender_id, "/heartbeat");
            ASSERT_EQ(msg->active, heartbeat_node->get_active());

            if(msg->active) {
                ASSERT_FALSE(last_msg.active) << "Got unexpected active message with tick: " << msg->tick;
                heartbeat_node->deactivate();
            } else {
                ASSERT_TRUE(last_msg.active) << "Got unexpected active message with seq: " << msg->tick;
                heartbeat_node->activate();
            }

            last_msg = *msg;
            ASSERT_EQ(last_msg.tick, msg->tick); });

    rclcpp::TimerBase::SharedPtr end_timer = test_node->create_wall_timer(
        std::chrono::milliseconds(heartbeat_period * 10),
        [&executor, test_node]()
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Stopping executor");
            executor.cancel(); });

    executor.add_node(test_node);

    executor.spin();
    ASSERT_EQ(last_msg.tick, 10);
}

class JobFinishedCommonNode : public CommonNode
{
public:
    JobFinishedCommonNode(const std::string &name) : CommonNode(name)
    {
    }

    inline void job_finished(const uint8_t error_code, const nlohmann::json payload)
    {
        CommonNode::job_finished(error_code, payload);
    }

    inline void job_finished(const std::string error_message)
    {
        CommonNode::job_finished(error_message);
    }

    inline void job_finished()
    {
        CommonNode::job_finished();
    }
};

TEST(common_package, job_finished_successfull)
{
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node = std::make_shared<JobFinishedCommonNode>("common_node");
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr job_finished_sub = test_node->create_subscription<interfaces::msg::JobFinished>(
        "job_finished",
        10,
        [common_node, test_node, &executor](interfaces::msg::JobFinished::ConstSharedPtr msg)
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Got job_finished message");
            ASSERT_EQ(msg->sender_id, "/common_node");
            ASSERT_EQ(msg->error_code, EXIT_SUCCESS);
            ASSERT_EQ(msg->payload.length(), 0);
            ASSERT_FALSE(common_node->get_active());
            executor.cancel();
        });

    rclcpp::TimerBase::SharedPtr job_finished_timer = common_node->create_wall_timer(
        std::chrono::milliseconds(100), [common_node]()
        {
            RCLCPP_DEBUG(common_node->get_logger(), "Sending job_finished message");
            common_node->job_finished(); });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}

TEST(common_package, job_finished_error_message)
{
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node = std::make_shared<JobFinishedCommonNode>("common_node");
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr job_finished_sub = test_node->create_subscription<interfaces::msg::JobFinished>(
        "job_finished",
        10,
        [common_node, test_node, &executor](interfaces::msg::JobFinished::ConstSharedPtr msg)
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Got job_finished message");
            ASSERT_EQ(msg->sender_id, "/common_node");
            ASSERT_EQ(msg->error_code, EXIT_FAILURE);

            nlohmann::json payload_check;
            payload_check["error_msg"] = "This is my error message";
            ASSERT_EQ(payload_check, nlohmann::json::parse(msg->payload));

            ASSERT_FALSE(common_node->get_active());
            executor.cancel();
        });

    rclcpp::TimerBase::SharedPtr job_finished_timer = common_node->create_wall_timer(
        std::chrono::milliseconds(100), [common_node]()
        {
            RCLCPP_DEBUG(common_node->get_logger(), "Sending job_finished message");
            common_node->job_finished("This is my error message"); });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}

TEST(common_package, job_finished_custom_payload)
{
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node = std::make_shared<JobFinishedCommonNode>("common_node");
    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr job_finished_sub = test_node->create_subscription<interfaces::msg::JobFinished>(
        "job_finished",
        10,
        [common_node, test_node, &executor](interfaces::msg::JobFinished::ConstSharedPtr msg)
        {
            RCLCPP_DEBUG(test_node->get_logger(), "Got job_finished message");
            ASSERT_EQ(msg->sender_id, "/common_node");
            ASSERT_EQ(msg->error_code, EXIT_FAILURE);

            nlohmann::json payload_check;
            payload_check["coord"] = "These are my coordinates";
            payload_check["height_cm"] = 500;
            ASSERT_EQ(payload_check, nlohmann::json::parse(msg->payload));

            ASSERT_FALSE(common_node->get_active());
            executor.cancel();
        });

    rclcpp::TimerBase::SharedPtr job_finished_timer = common_node->create_wall_timer(
        std::chrono::milliseconds(100), [common_node]()
        {
            RCLCPP_DEBUG(common_node->get_logger(), "Sending job_finished message");
            nlohmann::json payload;
            payload["coord"] = "These are my coordinates";
            payload["height_cm"] = 500;
            common_node->job_finished(EXIT_FAILURE, payload); });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}

int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);
    testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
