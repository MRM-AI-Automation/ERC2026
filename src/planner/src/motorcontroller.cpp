#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "stack/shared_state.hpp"
#include <cmath>
#include <string>

#define TRACK_WIDTH 1.005
#define WHEEL_DIAMETER 0.30
#define MAX_WHEEL_RPM 100

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("motor_controller")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorController::callback, this, std::placeholders::_1));

        max_linear_velocity = ((M_PI * WHEEL_DIAMETER) * MAX_WHEEL_RPM) / 60;
        RCLCPP_INFO(this->get_logger(), "MotorController started.");
    }

    void updateMotor()
    {
        std::string packet;
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            packet = pwm_enabled ? formatPacket() : "L0R0E";
            ai_motor_data = packet;
        }

        if (!pwm_enabled && last_packet != "L0R0E")
        {
            RCLCPP_WARN(this->get_logger(), "STOP command issued.");
        }

        if (packet != last_packet)
        {
            RCLCPP_INFO(this->get_logger(), "Motor command updated: %s", packet.c_str());
            last_packet = packet;
        }
    }

private:

    void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pwm_enabled = true;
        linear = msg->linear.x * 1.2;
        angular = -msg->angular.z * 1.2;

        if (linear == 0 && angular == 0)
        {
            pwm_enabled = false;
            return;
        }

        if (linear > max_linear_velocity) linear = max_linear_velocity;
        if (linear < -max_linear_velocity) linear = -max_linear_velocity;

        compute();

        RCLCPP_DEBUG(this->get_logger(),
            "cmd_vel -> linear: %.2f  angular: %.2f", linear, angular);
    }

    void compute()
    {
        float left = linear - angular * TRACK_WIDTH / 2;
        float right = linear + angular * TRACK_WIDTH / 2;

        left_rpm = fabs(left * 60 / (M_PI * WHEEL_DIAMETER));
        right_rpm = fabs(right * 60 / (M_PI * WHEEL_DIAMETER));

        left_pwm = std::min(left_rpm / MAX_WHEEL_RPM * 100.0f, 99.0f);
        right_pwm = std::min(right_rpm / MAX_WHEEL_RPM * 100.0f, 99.0f);

        direction = (left >= 0 && right >= 0) ? 1 :
                    (left < 0 && right < 0) ? 2 :
                    (left < 0 && right > 0) ? 3 : 4;

        RCLCPP_DEBUG(this->get_logger(),
            "PWM -> L: %.1f R: %.1f  | Dir=%d", left_pwm, right_pwm, direction);
    }

    std::string formatPacket()
    {
        switch (direction)
        {
            case 1: return "L" + std::to_string((int)left_pwm) + "R" + std::to_string((int)right_pwm) + "E";
            case 2: return "L-" + std::to_string((int)left_pwm) + "R-" + std::to_string((int)right_pwm) + "E";
            case 3: return "L-" + std::to_string((int)left_pwm) + "R" + std::to_string((int)right_pwm) + "E";
            case 4: return "L" + std::to_string((int)left_pwm) + "R-" + std::to_string((int)right_pwm) + "E";
        }
        return "L0R0E";
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    float linear{}, angular{};
    float left_rpm{}, right_rpm{};
    float left_pwm{}, right_pwm{};
    int direction{1};

    float max_linear_velocity{};
    bool pwm_enabled{false};

    std::string last_packet{"INIT"};
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
