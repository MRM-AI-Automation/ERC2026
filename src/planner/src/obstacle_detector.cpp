#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>
#include <string>

class ObstacleDetector : public rclcpp::Node
{
public:
    ObstacleDetector()
        : Node("obstacle_detector")
    {
        // Parameters
        declare_parameter<std::string>("lidar_topic", "/scan_front");
        declare_parameter<std::string>("obstacle_topic", "/obstacle_detected");
        declare_parameter<std::string>("position_topic", "/obstacle_position");

        declare_parameter<double>("min_x", 0.4);
        declare_parameter<double>("max_x", 2.0);
        declare_parameter<double>("half_width", 1.20);

        lidar_topic_ = get_parameter("lidar_topic").as_string();
        obstacle_topic_ = get_parameter("obstacle_topic").as_string();
        position_topic_ = get_parameter("position_topic").as_string();

        min_x_ = get_parameter("min_x").as_double();
        max_x_ = get_parameter("max_x").as_double();
        half_width_ = get_parameter("half_width").as_double();

        // Publisher: obstacle detected / not detected
        obstacle_pub_ =
            create_publisher<std_msgs::msg::Bool>(
                obstacle_topic_, 10);

        // Publisher: closest obstacle position
        position_pub_ =
            create_publisher<geometry_msgs::msg::Point>(
                position_topic_, 10);

        // LiDAR subscriber
        lidar_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                lidar_topic_,
                rclcpp::SensorDataQoS(),
                std::bind(
                    &ObstacleDetector::lidarCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "\033[1;36m[OBS] Obstacle detector started\033[0m");

        RCLCPP_INFO(
            get_logger(),
            "LiDAR topic: %s",
            lidar_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Detection region: X [%.2f, %.2f] m | Y +/- %.2f m",
            min_x_,
            max_x_,
            half_width_);
    }

private:

    void lidarCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        bool found = false;

        float best_x = std::numeric_limits<float>::max();
        float best_y = 0.0f;

        float angle = scan->angle_min;

        for (size_t i = 0;
             i < scan->ranges.size();
             ++i, angle += scan->angle_increment)
        {
            const float r = scan->ranges[i];

            // Reject invalid range
            if (!std::isfinite(r))
                continue;

            // Reject range outside LiDAR limits
            if (r < scan->range_min ||
                r > scan->range_max)
                continue;

            // Convert polar -> Cartesian
            //
            // LiDAR frame:
            // X = forward
            // Y = left
            //
            const float x = r * std::cos(angle);
            const float y = r * std::sin(angle);

            // Detection corridor
            if (x < min_x_)
                continue;

            if (x > max_x_)
                continue;

            if (std::abs(y) > half_width_)
                continue;

            // Keep closest obstacle in X direction
            if (x < best_x)
            {
                best_x = x;
                best_y = y;
                found = true;
            }
        }

        publishResult(found, best_x, best_y);
    }

    void publishResult(
        bool detected,
        float x,
        float y)
    {
        // -------------------------------------------------
        // Publish detection state
        // -------------------------------------------------

        std_msgs::msg::Bool detection_msg;
        detection_msg.data = detected;

        obstacle_pub_->publish(detection_msg);

        // -------------------------------------------------
        // Publish obstacle position
        // -------------------------------------------------

        geometry_msgs::msg::Point position_msg;

        if (detected)
        {
            position_msg.x = x;
            position_msg.y = y;
            position_msg.z = 0.0;

            const double distance =
                std::hypot(x, y);

            const char *side;

            if (y > 0.15)
                side = "left";
            else if (y < -0.15)
                side = "right";
            else
                side = "center";

            // RED = obstacle detected
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                300,
                "\033[1;31m[OBS] DETECTED\033[0m | "
                "x=%.2f m | y=%.2f m | "
                "dist=%.2f m | side=%s",
                x,
                y,
                distance,
                side);
        }
        else
        {
            // Explicitly publish zero when no obstacle
            position_msg.x = 0.0;
            position_msg.y = 0.0;
            position_msg.z = 0.0;

            // GREEN = no obstacle
            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "\033[1;32m[OBS] NO OBSTACLE\033[0m | "
                "corridor clear | "
                "X=[%.2f, %.2f] m | "
                "Y=+/-%.2f m",
                min_x_,
                max_x_,
                half_width_);
        }

        position_pub_->publish(position_msg);
    }

    // -------------------------------------------------
    // Parameters
    // -------------------------------------------------

    std::string lidar_topic_;
    std::string obstacle_topic_;
    std::string position_topic_;

    double min_x_;
    double max_x_;
    double half_width_;

    // -------------------------------------------------
    // ROS
    // -------------------------------------------------

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        lidar_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        obstacle_pub_;

    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
        position_pub_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node =
        std::make_shared<ObstacleDetector>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
