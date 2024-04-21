#include <gtest/gtest.h>

#include "rclcpp/node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(0, nullptr);
    testing::InitGoogleTest(&argc, argv);
    const int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return ret;
}
