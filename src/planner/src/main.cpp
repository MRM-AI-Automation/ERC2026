#include <rclcpp/rclcpp.hpp>
#include "planner/irc_planner.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<planner::SensorCallback>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
