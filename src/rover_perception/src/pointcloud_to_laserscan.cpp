#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudToLaserScan : public rclcpp::Node
{
public:
    PointCloudToLaserScan()
        : Node("pointcloud_to_laserscan")
    {
        input_topic_ = declare_parameter<std::string>(
            "input_topic", "/local_grid_obstacle");

        output_topic_ = declare_parameter<std::string>(
            "output_topic", "/local_grid_scan");

        min_height_ = declare_parameter<double>("min_height", -1.0);
        max_height_ = declare_parameter<double>("max_height", 1.0);

        angle_min_ = declare_parameter<double>(
            "angle_min", -M_PI);

        angle_max_ = declare_parameter<double>(
            "angle_max", M_PI);

        angle_increment_ = declare_parameter<double>(
            "angle_increment", M_PI / 720.0);  // 0.25 degrees

        range_min_ = declare_parameter<double>(
            "range_min", 0.1);

        range_max_ = declare_parameter<double>(
            "range_max", 10.0);

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
            output_topic_, 10);

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &PointCloudToLaserScan::cloudCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "PointCloud -> LaserScan started");

        RCLCPP_INFO(
            get_logger(),
            "Input: %s",
            input_topic_.c_str());

        RCLCPP_INFO(
            get_logger(),
            "Output: %s",
            output_topic_.c_str());
    }

private:

    void cloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();

        scan->header = cloud->header;

        scan->angle_min = angle_min_;
        scan->angle_max = angle_max_;
        scan->angle_increment = angle_increment_;

        scan->time_increment = 0.0;
        scan->scan_time = 0.0;

        scan->range_min = range_min_;
        scan->range_max = range_max_;

        const int num_bins =
            static_cast<int>(
                std::ceil(
                    (angle_max_ - angle_min_) /
                    angle_increment_));

        scan->ranges.assign(
            num_bins,
            std::numeric_limits<float>::infinity());

        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(
                *cloud, "x");

            sensor_msgs::PointCloud2ConstIterator<float> iter_y(
                *cloud, "y");

            sensor_msgs::PointCloud2ConstIterator<float> iter_z(
                *cloud, "z");

            for (; iter_x != iter_x.end();
                 ++iter_x, ++iter_y, ++iter_z)
            {
                const float x = *iter_x;
                const float y = *iter_y;
                const float z = *iter_z;

                // Ignore invalid points
                if (!std::isfinite(x) ||
                    !std::isfinite(y) ||
                    !std::isfinite(z))
                {
                    continue;
                }

                // Height filtering
                if (z < min_height_ ||
                    z > max_height_)
                {
                    continue;
                }

                // Horizontal distance
                const float range =
                    std::sqrt(x * x + y * y);

                if (range < range_min_ ||
                    range > range_max_)
                {
                    continue;
                }

                // Angle around the sensor
                const float angle =
                    std::atan2(y, x);

                if (angle < angle_min_ ||
                    angle > angle_max_)
                {
                    continue;
                }

                const int index =
                    static_cast<int>(
                        (angle - angle_min_) /
                        angle_increment_);

                if (index < 0 ||
                    index >= num_bins)
                {
                    continue;
                }

                // Keep closest obstacle
                if (range < scan->ranges[index])
                {
                    scan->ranges[index] = range;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                get_logger(),
                "PointCloud processing failed: %s",
                e.what());

            return;
        }

        scan_pub_->publish(std::move(scan));
    }

    std::string input_topic_;
    std::string output_topic_;

    double min_height_;
    double max_height_;

    double angle_min_;
    double angle_max_;
    double angle_increment_;

    double range_min_;
    double range_max_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        cloud_sub_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_pub_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<PointCloudToLaserScan>());

    rclcpp::shutdown();

    return 0;
}
