#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <limits>
#include <functional>

class FrontScanFilter : public rclcpp::Node
{
public:
    FrontScanFilter() : Node("front_scan_filter")
    {
        // Input LiDAR scan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &FrontScanFilter::scanCallback,
                this,
                std::placeholders::_1));

        // IMPORTANT:
        // Use RELIABLE QoS for /scan_front because downstream
        // nodes are requesting RELIABLE reliability.
        auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
        output_qos.reliable();
        output_qos.durability_volatile();

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/scan_front",
            output_qos);

        RCLCPP_INFO(
            this->get_logger(),
            "Front scan filter started: publishing /scan_front");
    }

private:
    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered = *msg;

        // Keep only:
        // -90 degrees -> +90 degrees
        const double front_min = -M_PI / 2.0;
        const double front_max =  M_PI / 2.0;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            double angle =
                msg->angle_min +
                static_cast<double>(i) * msg->angle_increment;

            // Normalize angle to [-pi, pi]
            while (angle > M_PI)
                angle -= 2.0 * M_PI;

            while (angle < -M_PI)
                angle += 2.0 * M_PI;

            // Remove points outside the front 180 degrees
            if (angle < front_min || angle > front_max)
            {
                filtered.ranges[i] =
                    std::numeric_limits<float>::infinity();
            }
        }

        scan_pub_->publish(filtered);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FrontScanFilter>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
