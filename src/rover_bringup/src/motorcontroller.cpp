#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define track_width 1.005
#define wheel_diameter 0.30
#define max_wheel_RPM 100

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {
        max_linear_velocity =
            ((M_PI * wheel_diameter) * max_wheel_RPM) / 60.0f;

        subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                1,
                std::bind(
                    &MotorController::vel_callback,
                    this,
                    std::placeholders::_1));

        last_cmd_time_ = this->now();

        init_socket();
    }

    ~MotorController()
    {
        if (sock >= 0)
        {
            close(sock);
        }
    }

    void updateMotor()
    {
        if (!socket_valid_)
        {
            return;
        }

        const double cmd_age =
            (this->now() - last_cmd_time_).seconds();

        if (cmd_age > 0.2)
        {
            if (pwm_start != 0)
            {
                RCLCPP_WARN(this->get_logger(), "cmd_vel timeout — stopping robot");
            }
            pwm_start = 0;
        }

        std::string pwm_data;

        if (pwm_start)
        {
            pwm_data = formatPWMData();
        }
        else
        {
            pwm_data = "L0R0E";
        }

        sendPWMData(pwm_data);
    }

private:
    void vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_time_ = this->now();

        pwm_start = 1;

        linear_vel = msg->linear.x * 1.2f;
        angular_vel = -msg->angular.z * 1.2f;

        RCLCPP_INFO(this->get_logger(),
                    "cmd_vel → linear: %.3f  angular: %.3f",
                    linear_vel, angular_vel);

        if (std::fabs(linear_vel) < EPSILON &&
            std::fabs(angular_vel) < EPSILON)
        {
            pwm_start = 0;
        }

        if (linear_vel > max_linear_velocity)
        {
            linear_vel = max_linear_velocity;
        }
        else if (linear_vel < -max_linear_velocity)
        {
            linear_vel = -max_linear_velocity;
        }

        findMaxAngular();

        if (angular_vel > maxAngularVel)
        {
            angular_vel = maxAngularVel;
        }
        else if (angular_vel < -maxAngularVel)
        {
            angular_vel = -maxAngularVel;
        }

        wheelVelocity();
    }

    void findMaxAngular()
    {
        maxAngularVel =
            (max_linear_velocity - std::fabs(linear_vel))
            * 2.0f / track_width;

        if (maxAngularVel < 0.0f)
        {
            maxAngularVel = 0.0f;
        }
    }

    void wheelVelocity()
    {
        right_vel = std::fabs(linear_vel + angular_vel * track_width / 2.0f);
        left_vel  = std::fabs(linear_vel - angular_vel * track_width / 2.0f);
        setRPM();
        setDirection();
    }

    void setDirection()
    {
        const float right_cmd =
            linear_vel + angular_vel * track_width / 2.0f;

        const float left_cmd =
            linear_vel - angular_vel * track_width / 2.0f;

        if (std::fabs(right_cmd) < EPSILON &&
            std::fabs(left_cmd) < EPSILON)
        {
            direction = 0;
        }
        else if (right_cmd > 0 && left_cmd > 0)
        {
            direction = 1;
        }
        else if (right_cmd < 0 && left_cmd < 0)
        {
            direction = 2;
        }
        else if (right_cmd < 0 && left_cmd > 0)
        {
            direction = 3;
        }
        else if (right_cmd > 0 && left_cmd < 0)
        {
            direction = 4;
        }
        else
        {
            direction = 0;
        }
    }

    void setRPM()
    {
        right_rpm = right_vel * 60.0f / (wheel_diameter * M_PI);
        left_rpm  = left_vel  * 60.0f / (wheel_diameter * M_PI);

        right_dutyCyclePercentage = right_rpm / max_wheel_RPM * 100.0f;
        left_dutyCyclePercentage  = left_rpm  / max_wheel_RPM * 100.0f;

        if (right_dutyCyclePercentage > 100.0f) right_dutyCyclePercentage = 99.0f;
        if (left_dutyCyclePercentage  > 100.0f) left_dutyCyclePercentage  = 99.0f;
    }

    void init_socket()
    {
        sock = socket(AF_INET, SOCK_DGRAM, 0);

        if (sock < 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Socket creation failed");
            return;
        }

        server.sin_family = AF_INET;
        server.sin_port   = htons(5005);

        if (inet_pton(AF_INET, "10.0.0.68", &server.sin_addr) <= 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Invalid IP address");
            close(sock);
            sock = -1;
            return;
        }

        socket_valid_ = true;

        RCLCPP_INFO(this->get_logger(), "UDP socket ready");
    }

    std::string formatPWMData()
    {
        switch (direction)
        {
            case 1:
                return "L"  + std::to_string(int(left_dutyCyclePercentage))
                     + "R"  + std::to_string(int(right_dutyCyclePercentage))
                     + "E";

            case 2:
                return "L-" + std::to_string(int(left_dutyCyclePercentage))
                     + "R-" + std::to_string(int(right_dutyCyclePercentage))
                     + "E";

            case 3:
                return "L"  + std::to_string(int(left_dutyCyclePercentage))
                     + "R-" + std::to_string(int(right_dutyCyclePercentage))
                     + "E";

            case 4:
                return "L-" + std::to_string(int(left_dutyCyclePercentage))
                     + "R"  + std::to_string(int(right_dutyCyclePercentage))
                     + "E";

            default:
                return "L0R0E";
        }
    }

    void sendPWMData(const std::string &data)
    {
        if (!socket_valid_)
        {
            return;
        }

        ssize_t sent_bytes =
            sendto(
                sock,
                data.c_str(),
                data.size(),
                0,
                (struct sockaddr *)&server,
                sizeof(server));

        if (sent_bytes < 0)
        {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Failed to send UDP data");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data.c_str());
        }
    }

    // Member declarations
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    int pwm_start{0};
    int Ena{13};
    int Enb{12};
    int Dira{19};
    int Dirb{6};

    int  sock{-1};
    bool socket_valid_{false};

    float right_dutyCyclePercentage{0.0f};
    float left_dutyCyclePercentage{0.0f};

    int direction{0};

    float linear_vel{0.0f};
    float angular_vel{0.0f};
    float right_vel{0.0f};
    float left_vel{0.0f};
    float right_rpm{0.0f};
    float left_rpm{0.0f};
    float maxAngularVel{0.0f};
    float max_linear_velocity{0.0f};

    rclcpp::Time last_cmd_time_;

    static constexpr float EPSILON = 1e-3f;

    struct sockaddr_in server{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorController>();
    rclcpp::Rate rate(60);

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->updateMotor();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
