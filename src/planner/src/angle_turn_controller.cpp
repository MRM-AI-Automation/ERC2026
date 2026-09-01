#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <msgs/msg/imu_data.hpp>

#include <cmath>
#include <memory>
#include <string>

class AngleTurnController : public rclcpp::Node
{
public:
    explicit AngleTurnController(double target_angle)
        : Node("angle_turn_controller")
    {
        // ================================
        // SETTINGS
        // ================================
        target_angle_deg_ = normalize360(target_angle);
        tolerance_deg_ = 5.0;

        // ONLY +2.0 OR -2.0
        angular_speed_ = 2.0;

        // Subscribe to IMU
        imu_sub_ = this->create_subscription<msgs::msg::ImuData>(
            "/imu_data",
            10,
            std::bind(
                &AngleTurnController::imuCallback,
                this,
                std::placeholders::_1
            )
        );

        // Publish turning commands
        cmd_vel_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10
            );

        RCLCPP_INFO(this->get_logger(),
                    "========================================");
        RCLCPP_INFO(this->get_logger(),
                    "ANGLE TURN CONTROLLER STARTED");
        RCLCPP_INFO(this->get_logger(),
                    "Target angle: %.2f degrees",
                    target_angle_deg_);
        RCLCPP_INFO(this->get_logger(),
                    "Tolerance: +/- %.2f degrees",
                    tolerance_deg_);
        RCLCPP_INFO(this->get_logger(),
                    "Angular turning speed: %.2f",
                    angular_speed_);
        RCLCPP_INFO(this->get_logger(),
                    "========================================");
    }

    ~AngleTurnController()
    {
        publishStop();
    }

private:

    // Convert angle to [0, 360)
    double normalize360(double angle)
    {
        angle = std::fmod(angle, 360.0);

        if (angle < 0.0)
        {
            angle += 360.0;
        }

        return angle;
    }


    // Calculate shortest signed angular error.
    // Result: (-180, +180]
    double shortestAngleError(double target, double current)
    {
        double error = target - current;

        while (error > 180.0)
        {
            error -= 360.0;
        }

        while (error <= -180.0)
        {
            error += 360.0;
        }

        return error;
    }


    // Publish complete stop command
    void publishStop()
    {
        if (!cmd_vel_pub_)
        {
            return;
        }

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd);
    }


    void imuCallback(
        const msgs::msg::ImuData::SharedPtr msg)
    {
        // orientation.z = yaw
        double current_angle_deg =
            normalize360(msg->orientation.z);

        // Calculate current shortest error
        const double error_deg =
            shortestAngleError(
                target_angle_deg_,
                current_angle_deg
            );

        geometry_msgs::msg::Twist cmd;


        // ==========================================
        // STOP WHEN TARGET IS REACHED
        // ==========================================
        if (std::fabs(error_deg) <= tolerance_deg_)
        {
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                500,
                "[TARGET REACHED] Current: %.2f deg | "
                "Target: %.2f deg | Error: %.2f deg | "
                "cmd_vel.angular.z: %.2f",
                current_angle_deg,
                target_angle_deg_,
                error_deg,
                cmd.angular.z
            );

            return;
        }


        // ==========================================
        // CHOOSE THE SHORTEST DIRECTION ONCE
        //
        // This happens on the FIRST IMU message.
        // After that, keep the direction locked.
        //
        // This prevents flipping at +/-180 degrees.
        // ==========================================
        if (!direction_chosen_)
        {
            if (error_deg >= 0.0)
            {
                turn_direction_ = 1;
            }
            else
            {
                turn_direction_ = -1;
            }

            direction_chosen_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "[SHORTEST DIRECTION SELECTED] %s | "
                "Current: %.2f deg | Target: %.2f deg | "
                "Initial Error: %.2f deg",
                turn_direction_ > 0
                    ? "POSITIVE (+2.0)"
                    : "NEGATIVE (-2.0)",
                current_angle_deg,
                target_angle_deg_,
                error_deg
            );
        }


        // ==========================================
        // TURN AT EXACTLY +2.0 OR -2.0
        // ==========================================
        cmd.angular.z =
            static_cast<double>(turn_direction_)
            * angular_speed_;

        cmd_vel_pub_->publish(cmd);


        // ==========================================
        // LOG CURRENT STATE
        // ==========================================
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            200,
            "[TURNING] Current: %.2f deg | "
            "Target: %.2f deg | Error: %.2f deg | "
            "cmd_vel.angular.z: %.2f",
            current_angle_deg,
            target_angle_deg_,
            error_deg,
            cmd.angular.z
        );
    }


    // ROS interfaces
    rclcpp::Subscription<msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // Controller settings
    double target_angle_deg_;
    double tolerance_deg_;
    double angular_speed_;

    // Turn state
    bool direction_chosen_ = false;
    int turn_direction_ = 1;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (argc < 2)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("angle_turn_controller"),
            "Usage: ros2 run planner angle_turn_controller <target_angle>"
        );

        RCLCPP_ERROR(
            rclcpp::get_logger("angle_turn_controller"),
            "Example: ros2 run planner angle_turn_controller 90"
        );

        rclcpp::shutdown();
        return 1;
    }

    double target_angle;

    try
    {
        target_angle = std::stod(argv[1]);
    }
    catch (const std::exception &)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("angle_turn_controller"),
            "Invalid target angle: %s",
            argv[1]
        );

        rclcpp::shutdown();
        return 1;
    }

    auto node =
        std::make_shared<AngleTurnController>(target_angle);

    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();
    return 0;
}
