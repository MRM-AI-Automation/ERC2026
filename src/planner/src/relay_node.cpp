#include <iostream>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <cstring>
#include <cmath>
#include <cerrno>
#include <algorithm>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "aruco_msgs/msg/arm_pwm.hpp"

using namespace std::chrono_literals;

static constexpr float track_width = 1.005f;
static constexpr float wheel_diameter = 0.30f;
static constexpr float max_wheel_RPM = 100.0f;

static constexpr const char* ESP_IP = "10.0.0.7";
static constexpr int ESP_PORT = 5005;
static constexpr int LISTEN_PORT = 5010;

class RelayNode : public rclcpp::Node
{
public:
    RelayNode()
        : Node("relay_node_udp_bridge")
    {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(
                &RelayNode::cmdVelCallback,
                this,
                std::placeholders::_1));

        arm_sub_ = this->create_subscription<aruco_msgs::msg::ArmPwm>(
            "/arm_pwm",
            10,
            std::bind(
                &RelayNode::armCallback,
                this,
                std::placeholders::_1));

        mode_cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/autonomous_mode_cmd",
            10,
            std::bind(
                &RelayNode::modeCmdCallback,
                this,
                std::placeholders::_1));

        mode_state_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/autonomous_mode_state",
                10);

        mode_timer_ = this->create_wall_timer(
            200ms,
            std::bind(
                &RelayNode::publishModeState,
                this));

        toggle_srv_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/toggle_autonomous",
                std::bind(
                    &RelayNode::toggleAutonomousService,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        listener_thread_ =
            std::thread(
                &RelayNode::udpListenerThread,
                this);

        sender_thread_ =
            std::thread(
                &RelayNode::udpSenderThread,
                this);

        RCLCPP_INFO(
            this->get_logger(),
            "Relay node started");
    }

    ~RelayNode()
    {
        running_.store(false);

        if (listener_socket_ >= 0)
        {
            shutdown(listener_socket_, SHUT_RDWR);
            close(listener_socket_);
            listener_socket_ = -1;
        }

        if (sender_socket_ >= 0)
        {
            shutdown(sender_socket_, SHUT_RDWR);
            close(sender_socket_);
            sender_socket_ = -1;
        }

        if (listener_thread_.joinable())
            listener_thread_.join();

        if (sender_thread_.joinable())
            sender_thread_.join();
    }

private:

    void toggleAutonomousService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool new_mode = !autonomous_mode_.load();

        autonomous_mode_.store(new_mode);

        if (!new_mode)
        {
            std::lock_guard<std::mutex> lock(mtx_);

            last_motor_packet_ = "L0R0";
            last_arm_packet_.clear();
        }

        response->success = true;
        response->message =
            new_mode ? "Autonomous ON" : "Autonomous OFF";
    }

    void cmdVelCallback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!autonomous_mode_.load())
            return;

        float linear_vel =
            static_cast<float>(msg->linear.x) * 1.2f;

        float angular_vel =
            -static_cast<float>(msg->angular.z) * 1.2f;

        const float max_linear_velocity =
            static_cast<float>(M_PI) *
            wheel_diameter *
            max_wheel_RPM /
            60.0f;

        linear_vel = std::clamp(
            linear_vel,
            -max_linear_velocity,
            max_linear_velocity);

        float max_angular_velocity =
            (max_linear_velocity -
             std::fabs(linear_vel)) *
            2.0f /
            track_width;

        max_angular_velocity =
            std::max(0.0f, max_angular_velocity);

        angular_vel = std::clamp(
            angular_vel,
            -max_angular_velocity,
            max_angular_velocity);

        float right_vel =
            linear_vel +
            angular_vel * track_width / 2.0f;

        float left_vel =
            linear_vel -
            angular_vel * track_width / 2.0f;

        float right_rpm =
            right_vel *
            60.0f /
            (wheel_diameter *
             static_cast<float>(M_PI));

        float left_rpm =
            left_vel *
            60.0f /
            (wheel_diameter *
             static_cast<float>(M_PI));

        right_rpm = std::clamp(
            right_rpm,
            -max_wheel_RPM,
            max_wheel_RPM);

        left_rpm = std::clamp(
            left_rpm,
            -max_wheel_RPM,
            max_wheel_RPM);

        int right_pct = static_cast<int>(
            std::round(
                std::fabs(right_rpm) /
                max_wheel_RPM *
                100.0f));

        int left_pct = static_cast<int>(
            std::round(
                std::fabs(left_rpm) /
                max_wheel_RPM *
                100.0f));

        right_pct = std::clamp(right_pct, 0, 99);
        left_pct = std::clamp(left_pct, 0, 99);

        std::string packet;

        if (left_rpm >= 0.0f && right_rpm >= 0.0f)
        {
            packet =
                "L" +
                std::to_string(left_pct) +
                "R" +
                std::to_string(right_pct);
        }
        else if (left_rpm < 0.0f && right_rpm < 0.0f)
        {
            packet =
                "L-" +
                std::to_string(left_pct) +
                "R-" +
                std::to_string(right_pct);
        }
        else if (left_rpm < 0.0f && right_rpm >= 0.0f)
        {
            packet =
                "L-" +
                std::to_string(left_pct) +
                "R" +
                std::to_string(right_pct);
        }
        else
        {
            packet =
                "L" +
                std::to_string(left_pct) +
                "R-" +
                std::to_string(right_pct);
        }

        std::lock_guard<std::mutex> lock(mtx_);

        last_motor_packet_ = packet;
    }

    void armCallback(
        const aruco_msgs::msg::ArmPwm::SharedPtr msg)
    {
        if (!autonomous_mode_.load())
            return;

        std::lock_guard<std::mutex> lock(mtx_);

        last_arm_data[0] = msg->link1;
        last_arm_data[1] = msg->link2;
        last_arm_data[2] = msg->gripper;

        last_arm_packet_ =
            "T" +
            std::to_string(
                static_cast<int>(msg->link1)) +
            "U" +
            std::to_string(
                static_cast<int>(msg->link2)) +
            "G" +
            std::to_string(
                static_cast<int>(msg->gripper));
    }

    void modeCmdCallback(
        const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool old_mode = autonomous_mode_.load();
        bool new_mode = msg->data;

        autonomous_mode_.store(new_mode);

        if (old_mode && !new_mode)
        {
            std::lock_guard<std::mutex> lock(mtx_);

            last_motor_packet_ = "L0R0";
            last_arm_packet_.clear();
        }
    }

    void publishModeState()
    {
        std_msgs::msg::Bool msg;

        msg.data = autonomous_mode_.load();

        mode_state_pub_->publish(msg);
    }

    void udpListenerThread()
    {
        listener_socket_ =
            socket(AF_INET, SOCK_DGRAM, 0);

        if (listener_socket_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create UDP listener socket");

            return;
        }

        int reuse = 1;

        setsockopt(
            listener_socket_,
            SOL_SOCKET,
            SO_REUSEADDR,
            &reuse,
            sizeof(reuse));

        sockaddr_in serv{};
        sockaddr_in cli{};

        serv.sin_family = AF_INET;
        serv.sin_addr.s_addr = INADDR_ANY;
        serv.sin_port = htons(LISTEN_PORT);

        if (bind(
                listener_socket_,
                reinterpret_cast<sockaddr*>(&serv),
                sizeof(serv)) < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to bind UDP port %d: %s",
                LISTEN_PORT,
                std::strerror(errno));

            close(listener_socket_);
            listener_socket_ = -1;

            return;
        }

        timeval timeout{};
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000;

        setsockopt(
            listener_socket_,
            SOL_SOCKET,
            SO_RCVTIMEO,
            &timeout,
            sizeof(timeout));

        RCLCPP_INFO(
            this->get_logger(),
            "UDP listener running on port %d",
            LISTEN_PORT);

        char buf[1500];

        while (rclcpp::ok() && running_.load())
        {
            socklen_t len = sizeof(cli);

            ssize_t n =
                recvfrom(
                    listener_socket_,
                    buf,
                    sizeof(buf),
                    0,
                    reinterpret_cast<sockaddr*>(&cli),
                    &len);

            if (n <= 0)
                continue;

            if (!autonomous_mode_.load())
            {
                std::lock_guard<std::mutex> lock(mtx_);

                last_manual_packet_ =
                    std::string(buf, n);
            }
        }

        if (listener_socket_ >= 0)
        {
            close(listener_socket_);
            listener_socket_ = -1;
        }
    }

    void udpSenderThread()
    {
        sender_socket_ =
            socket(AF_INET, SOCK_DGRAM, 0);

        if (sender_socket_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create UDP sender socket");

            return;
        }

        sockaddr_in esp{};

        esp.sin_family = AF_INET;
        esp.sin_port = htons(ESP_PORT);

        if (inet_pton(
                AF_INET,
                ESP_IP,
                &esp.sin_addr) != 1)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid ESP IP: %s",
                ESP_IP);

            close(sender_socket_);
            sender_socket_ = -1;

            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "UDP sender -> %s:%d",
            ESP_IP,
            ESP_PORT);

        auto last_log =
            std::chrono::steady_clock::now();

        while (rclcpp::ok() && running_.load())
        {
            std::string packet;
            bool mode = autonomous_mode_.load();

            {
                std::lock_guard<std::mutex> lock(mtx_);

                if (!mode)
                {
                    std::string manual =
                        last_manual_packet_.empty()
                            ? "M0X0Y0P0Q0A0S0J0DE"
                            : last_manual_packet_;

                    packet =
                        manual +
                        "|L0R0T0U0G0E|Z0";
                }
                else
                {
                    std::string motor =
                        last_motor_packet_.empty()
                            ? "L0R0"
                            : last_motor_packet_;

                    std::string arm =
                        last_arm_packet_.empty()
                            ? "T0U0G0E"
                            : last_arm_packet_ + "E";

                    packet =
                        motor +
                        arm +
                        "|Z1";
                }
            }

            ssize_t sent =
                sendto(
                    sender_socket_,
                    packet.c_str(),
                    packet.size(),
                    0,
                    reinterpret_cast<sockaddr*>(&esp),
                    sizeof(esp));

            if (sent < 0)
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "UDP send failed: %s",
                    std::strerror(errno));
            }

            auto now =
                std::chrono::steady_clock::now();

            if (now - last_log >= 1s)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "TX[%s] -> %s",
                    mode ? "AUTO" : "MANUAL",
                    packet.c_str());

                last_log = now;
            }

            std::this_thread::sleep_for(50ms);
        }

        if (sender_socket_ >= 0)
        {
            close(sender_socket_);
            sender_socket_ = -1;
        }
    }

    std::atomic<bool> running_{true};
    std::atomic<bool> autonomous_mode_{false};

    std::thread listener_thread_;
    std::thread sender_thread_;

    std::mutex mtx_;

    std::string last_manual_packet_ =
        "M0X0Y0P0Q0A0S0J0DE";

    std::string last_motor_packet_ =
        "L0R0";

    std::string last_arm_packet_;

    int last_arm_data[3] = {0, 0, 0};

    int listener_socket_ = -1;
    int sender_socket_ = -1;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_sub_;

    rclcpp::Subscription<aruco_msgs::msg::ArmPwm>::SharedPtr
        arm_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
        mode_cmd_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        mode_state_pub_;

    rclcpp::TimerBase::SharedPtr
        mode_timer_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
        toggle_srv_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<RelayNode>());

    rclcpp::shutdown();

    return 0;
}
