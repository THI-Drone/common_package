#pragma once

#include <cinttypes>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/duration.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "interfaces/msg/heartbeat.hpp"

namespace common_package {
    /**
     * @class CommonNode
     * @brief A class representing a common node.
     *
     * This class inherits from rclcpp::Node and provides functionality for creating a node that sends heartbeat messages.
     */
    class CommonNode : public rclcpp::Node {
    public:
        /**
         * @brief Constructor for creating a new CommonNode.
         * @param id Unique name of the node.
         * @param options Optional options for the Node
         */
        CommonNode(const std::string &id, const rclcpp::NodeOptions &options = rclcpp::NodeOptions()) : Node(id,
                                                                                                             options) {

            this->critical_callbacks = this->create_callback_group(
                    rclcpp::CallbackGroupType::MutuallyExclusive,
                    true
            );

            const auto heartbeat_period = std::chrono::duration<uint32_t, std::milli>(
                    this->declare_parameter("heartbeat_period", 500));
            RCLCPP_DEBUG(this->get_logger(), "Got heartbeat period: %" PRIu32 "ms", heartbeat_period.count());

            // Create a publisher for the "heartbeat" topic
            this->heartbeat_publisher = this->create_publisher<interfaces::msg::Heartbeat>(
                    "heartbeat",
                    rclcpp::QoS(
                            rclcpp::KeepLast(1),
                            rmw_qos_profile_t{
                                    rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                    1,
                                    rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                    rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                    rclcpp::Duration(heartbeat_period).to_rmw_time(),
                                    rclcpp::Duration(heartbeat_period / 2).to_rmw_time(),
                                    rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
                                    rclcpp::Duration(heartbeat_period * 2).to_rmw_time(),
                                    false,
                            }
                    )
            );
            RCLCPP_DEBUG(this->get_logger(), "Create heartbeat publisher");

            // Create a timer that sends a heartbeat message every 500ms
            this->heartbeat_timer = this->create_wall_timer(heartbeat_period,
                                                            std::bind(&CommonNode::timer_callback, this),
                                                            this->critical_callbacks);
            RCLCPP_DEBUG(this->get_logger(), "Created heartbeat timer");
        }

        rclcpp::CallbackGroup::SharedPtr critical_callbacks;  ///< Callback group for callbacks that should block the heartbeat.

    protected:
        /**
         * @brief Setter for active
         */
        void activate();

        /**
         * @brief Setter for active
         */
        void deactivate();

    private:
        /**
         * @brief Callback function for the timer.
         */
        void timer_callback();

        bool node_active = false;  ///< Indicating if node is currently active and sending commands to interface node

        rclcpp::TimerBase::SharedPtr heartbeat_timer;  ///< Timer for sending heartbeat messages
        rclcpp::Publisher<interfaces::msg::Heartbeat>::SharedPtr heartbeat_publisher;  ///< Publisher for the "heartbeat" topic
        interfaces::msg::Heartbeat::_seq_type heartbeat_tick = 0;  ///< Tick counting upwards with every heartbeat
    };

}