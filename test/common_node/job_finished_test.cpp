#include "interfaces/msg/job_finished.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cinttypes>
#include <nlohmann/json.hpp>

#include "common_package/common_node.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"

using namespace common_lib;

class JobFinishedCommonNode : public CommonNode {
   public:
    JobFinishedCommonNode(const std::string &name) : CommonNode(name) {}

    inline void activate() { CommonNode::activate(); }

    inline void job_finished(const uint8_t error_code,
                             const nlohmann::json &payload) {
        CommonNode::job_finished(error_code, payload);
    }

    inline void job_finished(const std::string &error_message) {
        CommonNode::job_finished(error_message);
    }

    inline void job_finished() { CommonNode::job_finished(); }
};

TEST(common_package, job_finished_successfull) {
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node =
        std::make_shared<JobFinishedCommonNode>("common_node");
    ASSERT_FALSE(common_node->get_active());
    common_node->activate();
    ASSERT_TRUE(common_node->get_active());

    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr
        job_finished_sub =
            test_node->create_subscription<interfaces::msg::JobFinished>(
                topic_names::JobFinished, 10,
                [common_node, test_node,
                 &executor](interfaces::msg::JobFinished::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Got job_finished message");
                    ASSERT_EQ(msg->sender_id, "common_node");
                    ASSERT_EQ(msg->error_code, EXIT_SUCCESS);
                    ASSERT_EQ(nlohmann::json::parse(msg->payload),
                              nlohmann::json::parse("{}"));
                    ASSERT_FALSE(common_node->get_active());
                    executor.cancel();
                });

    rclcpp::TimerBase::SharedPtr job_finished_timer =
        common_node->create_wall_timer(
            std::chrono::milliseconds(100), [common_node]() {
                RCLCPP_DEBUG(common_node->get_logger(),
                             "Sending job_finished message");
                common_node->job_finished();
            });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}

TEST(common_package, job_finished_error_message) {
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node =
        std::make_shared<JobFinishedCommonNode>("common_node");
    ASSERT_FALSE(common_node->get_active());
    common_node->activate();
    ASSERT_TRUE(common_node->get_active());

    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr
        job_finished_sub =
            test_node->create_subscription<interfaces::msg::JobFinished>(
                topic_names::JobFinished, 10,
                [common_node, test_node,
                 &executor](interfaces::msg::JobFinished::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Got job_finished message");
                    ASSERT_EQ(msg->sender_id, "common_node");
                    ASSERT_EQ(msg->error_code, EXIT_FAILURE);

                    nlohmann::json payload_check;
                    payload_check["error_msg"] = "This is my error message";
                    ASSERT_EQ(payload_check,
                              nlohmann::json::parse(msg->payload));

                    ASSERT_FALSE(common_node->get_active());
                    executor.cancel();
                });

    rclcpp::TimerBase::SharedPtr job_finished_timer =
        common_node->create_wall_timer(
            std::chrono::milliseconds(100), [common_node]() {
                RCLCPP_DEBUG(common_node->get_logger(),
                             "Sending job_finished message");
                common_node->job_finished("This is my error message");
            });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}

TEST(common_package, job_finished_custom_payload) {
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<JobFinishedCommonNode> common_node =
        std::make_shared<JobFinishedCommonNode>("common_node");
    ASSERT_FALSE(common_node->get_active());
    common_node->activate();
    ASSERT_TRUE(common_node->get_active());

    rclcpp::Node::SharedPtr test_node = std::make_shared<rclcpp::Node>("test");

    rclcpp::Subscription<interfaces::msg::JobFinished>::SharedPtr
        job_finished_sub =
            test_node->create_subscription<interfaces::msg::JobFinished>(
                topic_names::JobFinished, 10,
                [common_node, test_node,
                 &executor](interfaces::msg::JobFinished::ConstSharedPtr msg) {
                    RCLCPP_DEBUG(test_node->get_logger(),
                                 "Got job_finished message");
                    ASSERT_EQ(msg->sender_id, "common_node");
                    ASSERT_EQ(msg->error_code, 5);

                    nlohmann::json payload_check;
                    payload_check["coord"] = "These are my coordinates";
                    payload_check["height_cm"] = 500;
                    ASSERT_EQ(payload_check,
                              nlohmann::json::parse(msg->payload));

                    ASSERT_FALSE(common_node->get_active());
                    executor.cancel();
                });

    rclcpp::TimerBase::SharedPtr job_finished_timer =
        common_node->create_wall_timer(
            std::chrono::milliseconds(100), [common_node]() {
                RCLCPP_DEBUG(common_node->get_logger(),
                             "Sending job_finished message");
                nlohmann::json payload;
                payload["coord"] = "These are my coordinates";
                payload["height_cm"] = 500;
                common_node->job_finished(5, payload);
            });

    executor.add_node(common_node);
    executor.add_node(test_node);

    executor.spin();
}
