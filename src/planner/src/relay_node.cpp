#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <cerrno>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

#include <aruco_msgs/msg/arm_pwm.hpp>






constexpr double TRACK_WIDTH = 1.005;
constexpr double WHEEL_DIAMETER = 0.30;
constexpr double MAX_WHEEL_RPM = 100.0;






class ArmStateSubscriber : public rclcpp::Node
{
public:

    ArmStateSubscriber()
        : Node("arm_state_subscriber")
    {




        udp_ip_tx_ = "10.0.0.68";
        udp_port_tx_ = 5005;





        sock_tx_ = socket(AF_INET, SOCK_DGRAM, 0);

        if (sock_tx_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create TX socket: %s",
                std::strerror(errno)
            );
        }





        rx_sock_ = socket(AF_INET, SOCK_DGRAM, 0);

        if (rx_sock_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to create RX socket: %s",
                std::strerror(errno)
            );
        }





        int opt = 1;

        setsockopt(
            rx_sock_,
            SOL_SOCKET,
            SO_REUSEADDR,
            &opt,
            sizeof(opt)
        );





        sockaddr_in rx_addr{};

        rx_addr.sin_family = AF_INET;
        rx_addr.sin_addr.s_addr = INADDR_ANY;
        rx_addr.sin_port = htons(5005);

        if (
            bind(
                rx_sock_,
                reinterpret_cast<sockaddr*>(&rx_addr),
                sizeof(rx_addr)
            ) < 0
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to bind RX socket: %s",
                std::strerror(errno)
            );
        }





        int flags = fcntl(
            rx_sock_,
            F_GETFL,
            0
        );

        if (flags >= 0)
        {
            fcntl(
                rx_sock_,
                F_SETFL,
                flags | O_NONBLOCK
            );
        }





        tx_addr_.sin_family = AF_INET;
        tx_addr_.sin_port = htons(udp_port_tx_);

        if (
            inet_pton(
                AF_INET,
                udp_ip_tx_.c_str(),
                &tx_addr_.sin_addr
            ) <= 0
        )
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Invalid TX IP address: %s",
                udp_ip_tx_.c_str()
            );
        }





        ref_string_ = "M0L0R0T0U0S0G0Z0E";
        send_string_ = ref_string_;





        automate_ = false;





        mode_state_pub_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "/autonomous_mode_state",
                10
            );





        automation_service_ =
            this->create_service<std_srvs::srv::Trigger>(
                "/toggle_autonomous",
                std::bind(
                    &ArmStateSubscriber::toggle_autonomous_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2
                )
            );





        RCLCPP_INFO(
            this->get_logger(),
            "[MANUAL] Initial mode"
        );

        publish_mode_state();





        looptime_ = 0.0025;

        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(looptime_)
            ),
            std::bind(
                &ArmStateSubscriber::send_pwm,
                this
            )
        );





        cmd_vel_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &ArmStateSubscriber::cmd_vel_callback,
                    this,
                    std::placeholders::_1
                )
            );





        virtualarm_ =
            this->create_subscription<aruco_msgs::msg::ArmPwm>(
                "/joint_states",
                10,
                std::bind(
                    &ArmStateSubscriber::joint_callback,
                    this,
                    std::placeholders::_1
                )
            );

        realArm_ =
            this->create_subscription<aruco_msgs::msg::ArmPwm>(
                "/joint_angles",
                10,
                std::bind(
                    &ArmStateSubscriber::arm_callback,
                    this,
                    std::placeholders::_1
                )
            );





        real_l1_ = 0.0;
        real_l2_ = 0.0;
        real_swivel_ = 0.0;

        sim_l1_ = 0.0;
        sim_l2_ = 0.0;
        sim_swivel_ = 0.0;

        sim_gripper_ = 0;

        gripperEngage_ = false;





        link1_error_prev_ = 0.0;
        link1_error_int_ = 0.0;

        pid_const_link1_p_ = 25.0;
        pid_const_link1_i_ = 0.1;
        pid_const_link1_d_ = 40.0;

        link2_error_prev_ = 0.0;
        link2_error_int_ = 0.0;

        pid_const_link2_p_ = 25.0;
        pid_const_link2_i_ = 1.0;
        pid_const_link2_d_ = 40.0;

        swivel_error_prev_ = 0.0;
        swivel_error_int_ = 0.0;

        pid_const_swivel_p_ = 100.0;
        pid_const_swivel_i_ = 0.08;
        pid_const_swivel_d_ = 10.0;

        pwm_l1_ = 0;
        pwm_l2_ = 0;
        pwm_swivel_ = 0;





        linear_ = 0.0;
        angular_ = 0.0;

        left_ = 0.0;
        right_ = 0.0;

        left_rpm_ = 0.0;
        right_rpm_ = 0.0;

        left_pwm_ = 0.0;
        right_pwm_ = 0.0;

        pwm_enabled_ = false;





        max_linear_velocity_ =
            (
                M_PI *
                WHEEL_DIAMETER *
                MAX_WHEEL_RPM
            ) / 60.0;

        RCLCPP_INFO(
            this->get_logger(),
            "Combined Arm + Rover Controller started"
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Max rover velocity: %.3f m/s",
            max_linear_velocity_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Service: /toggle_autonomous [std_srvs/srv/Trigger]"
        );
    }






    void publish_mode_state()
    {
        std_msgs::msg::Bool msg;

        msg.data = automate_;

        mode_state_pub_->publish(msg);
    }






    void toggle_autonomous_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;


        automate_ = !automate_;





        if (automate_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[AUTONOMOUS] MODE ENABLED"
            );

            response->success = true;
            response->message =
                "Autonomous mode enabled";
        }





        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "[MANUAL] MODE ENABLED"
            );



            pwm_enabled_ = false;

            linear_ = 0.0;
            angular_ = 0.0;

            left_ = 0.0;
            right_ = 0.0;

            left_pwm_ = 0.0;
            right_pwm_ = 0.0;

            response->success = true;
            response->message =
                "Manual mode enabled";
        }

        publish_mode_state();
    }






    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {


        if (!automate_)
        {
            return;
        }





        linear_ =
            msg->linear.x * 1.2;

        angular_ =
            -msg->angular.z * 1.2;





        if (
            linear_ == 0.0 &&
            angular_ == 0.0
        )
        {
            pwm_enabled_ = false;

            left_ = 0.0;
            right_ = 0.0;

            left_pwm_ = 0.0;
            right_pwm_ = 0.0;

            return;
        }

        pwm_enabled_ = true;





        if (
            linear_ >
            max_linear_velocity_
        )
        {
            linear_ =
                max_linear_velocity_;
        }

        if (
            linear_ <
            -max_linear_velocity_
        )
        {
            linear_ =
                -max_linear_velocity_;
        }





        left_ =
            linear_ +
            angular_ *
            TRACK_WIDTH /
            2.0;

        right_ =
            linear_ -
            angular_ *
            TRACK_WIDTH /
            2.0;





        left_rpm_ =
            std::abs(
                left_ *
                60.0 /
                (M_PI * WHEEL_DIAMETER)
            );

        right_rpm_ =
            std::abs(
                right_ *
                60.0 /
                (M_PI * WHEEL_DIAMETER)
            );





        left_pwm_ =
            std::min(
                left_rpm_ /
                MAX_WHEEL_RPM *
                100.0,
                99.0
            );

        right_pwm_ =
            std::min(
                right_rpm_ /
                MAX_WHEEL_RPM *
                100.0,
                99.0
            );
    }






    std::string format_motor_packet()
    {
        int left_pwm =
            static_cast<int>(left_pwm_);

        int right_pwm =
            static_cast<int>(right_pwm_);





        if (
            left_ >= 0.0 &&
            right_ >= 0.0
        )
        {
            return
                "L" +
                std::to_string(right_pwm) +
                "R" +
                std::to_string(left_pwm);
        }





        else if (
            left_ < 0.0 &&
            right_ < 0.0
        )
        {
            return
                "L-" +
                std::to_string(left_pwm) +
                "R-" +
                std::to_string(right_pwm);
        }





        else if (
            left_ < 0.0 &&
            right_ > 0.0
        )
        {
            return
                "L-" +
                std::to_string(left_pwm) +
                "R" +
                std::to_string(right_pwm);
        }





        else
        {
            return
                "L" +
                std::to_string(left_pwm) +
                "R-" +
                std::to_string(right_pwm);
        }
    }






    void joint_callback(
        const aruco_msgs::msg::ArmPwm::SharedPtr msg)
    {
        sim_swivel_ =
            static_cast<double>(msg->swivel);

        sim_l1_ =
            static_cast<double>(msg->link1);

        sim_l2_ =
            static_cast<double>(msg->link2);

        sim_gripper_ =
            msg->gripper;
    }






    void arm_callback(
        const aruco_msgs::msg::ArmPwm::SharedPtr msg)
    {
        real_l1_ =
            static_cast<double>(msg->link1);

        real_l2_ =
            static_cast<double>(msg->link2);

        real_swivel_ =
            static_cast<double>(msg->swivel);

        real_swivel_ =
            std::fmod(
                real_swivel_,
                360.0
            );

        if (real_swivel_ < 0.0)
        {
            real_swivel_ += 360.0;
        }
    }






    double angle_error(
        double target,
        double current)
    {
        double error =
            std::fmod(
                target -
                current +
                180.0,
                360.0
            );

        if (error < 0.0)
        {
            error += 360.0;
        }

        return error - 180.0;
    }






    void update_arm_pid()
    {
        pwm_l1_ = 0;
        pwm_l2_ = 0;
        pwm_swivel_ = 0;





        double link1_error =
            angle_error(
                sim_l1_,
                real_l1_
            );

        double link2_error =
            angle_error(
                sim_l2_,
                real_l2_
            );

        double swivel_error =
            angle_error(
                sim_swivel_,
                real_swivel_
            );





        if (
            std::abs(link1_error) >
            0.0
        )
        {
            link1_error_int_ +=
                link1_error *
                looptime_;

            double p =
                pid_const_link1_p_ *
                link1_error;

            double i =
                pid_const_link1_i_ *
                link1_error_int_;

            double d =
                pid_const_link1_d_ *
                (
                    link1_error -
                    link1_error_prev_
                ) /
                looptime_;

            double pwm =
                p + i + d;

            pwm =
                std::max(
                    std::min(
                        pwm,
                        255.0
                    ),
                    -255.0
                );

            pwm =
                std::round(pwm);


            pwm = -pwm;

            pwm_l1_ =
                static_cast<int>(pwm);

            link1_error_prev_ =
                link1_error;
        }
        else
        {
            pwm_l1_ = 0;
        }





        if (
            std::abs(link2_error) >
            0.0
        )
        {
            link2_error_int_ +=
                link2_error *
                looptime_;

            double p =
                pid_const_link2_p_ *
                link2_error;

            double i =
                pid_const_link2_i_ *
                link2_error_int_;

            double d =
                pid_const_link2_d_ *
                (
                    link2_error -
                    link2_error_prev_
                ) /
                looptime_;

            double pwm =
                p + i + d;

            pwm =
                std::max(
                    std::min(
                        pwm,
                        255.0
                    ),
                    -255.0
                );

            pwm =
                std::round(pwm);


            pwm = -pwm;

            pwm_l2_ =
                static_cast<int>(pwm);

            link2_error_prev_ =
                link2_error;
        }
        else
        {
            pwm_l2_ = 0;
        }





        if (
            std::abs(swivel_error) >
            0.0
        )
        {
            swivel_error_int_ +=
                swivel_error *
                looptime_;

            double p =
                pid_const_swivel_p_ *
                swivel_error;

            double i =
                pid_const_swivel_i_ *
                swivel_error_int_;

            double d =
                pid_const_swivel_d_ *
                (
                    swivel_error -
                    swivel_error_prev_
                ) /
                looptime_;

            double pwm =
                p + i + d;

            pwm =
                std::max(
                    std::min(
                        pwm,
                        255.0
                    ),
                    -255.0
                );

            pwm =
                std::round(pwm);





            pwm = 0;

            pwm_swivel_ = 0;

            swivel_error_prev_ =
                swivel_error;
        }
        else
        {
            pwm_swivel_ = 0;
        }
    }






    std::string build_autonomous_packet()
    {




        update_arm_pid();

        std::string packet =
            ref_string_;





        std::size_t pos =
            packet.find("U0");

        if (pos != std::string::npos)
        {
            packet.replace(
                pos,
                2,
                "U" +
                std::to_string(pwm_l1_)
            );
        }





        pos =
            packet.find("T0");

        if (pos != std::string::npos)
        {
            packet.replace(
                pos,
                2,
                "T" +
                std::to_string(pwm_l2_)
            );
        }





        pos =
            packet.find("S0");

        if (pos != std::string::npos)
        {
            packet.replace(
                pos,
                2,
                "S" +
                std::to_string(pwm_swivel_)
            );
        }





        std::string motor_packet;

        if (pwm_enabled_)
        {
            motor_packet =
                format_motor_packet();
        }
        else
        {
            motor_packet = "L0R0";
        }





        pos =
            packet.find("L0R0");

        if (pos != std::string::npos)
        {
            packet.replace(
                pos,
                4,
                motor_packet
            );
        }

        return packet;
    }






    void send_pwm()
    {




        if (automate_)
        {
            send_string_ =
                build_autonomous_packet();

            std::cout
                << "[AUTONOMOUS] TX: "
                << send_string_
                << std::endl;

            ssize_t sent =
                sendto(
                    sock_tx_,
                    send_string_.c_str(),
                    send_string_.size(),
                    0,
                    reinterpret_cast<sockaddr*>(
                        &tx_addr_
                    ),
                    sizeof(tx_addr_)
                );

            if (sent < 0)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[AUTONOMOUS] UDP TX error: %s",
                    std::strerror(errno)
                );
            }
        }





        else
        {
            char buffer[1024];

            sockaddr_in sender_addr{};

            socklen_t sender_len =
                sizeof(sender_addr);

            ssize_t received =
                recvfrom(
                    rx_sock_,
                    buffer,
                    sizeof(buffer) - 1,
                    0,
                    reinterpret_cast<sockaddr*>(
                        &sender_addr
                    ),
                    &sender_len
                );

            if (received > 0)
            {
                buffer[received] = '\0';

                std::string received_string(
                    buffer
                );





                while (
                    !received_string.empty() &&
                    (
                        received_string.back() ==
                            '\n' ||
                        received_string.back() ==
                            '\r' ||
                        received_string.back() ==
                            ' '
                    )
                )
                {
                    received_string.pop_back();
                }





                std::cout
                    << "[MANUAL] RX: "
                    << received_string
                    << std::endl;





                char sender_ip[
                    INET_ADDRSTRLEN
                ];

                inet_ntop(
                    AF_INET,
                    &sender_addr.sin_addr,
                    sender_ip,
                    INET_ADDRSTRLEN
                );

                RCLCPP_INFO(
                    this->get_logger(),
                    "[MANUAL] Packet from %s:%d: %s",
                    sender_ip,
                    ntohs(sender_addr.sin_port),
                    received_string.c_str()
                );





                std::cout
                    << "[MANUAL] TX: "
                    << received_string
                    << std::endl;

                ssize_t sent =
                    sendto(
                        sock_tx_,
                        received_string.c_str(),
                        received_string.size(),
                        0,
                        reinterpret_cast<sockaddr*>(
                            &tx_addr_
                        ),
                        sizeof(tx_addr_)
                    );

                if (sent < 0)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "[MANUAL] UDP TX error: %s",
                        std::strerror(errno)
                    );
                }
            }
            else if (
                received < 0 &&
                errno != EAGAIN &&
                errno != EWOULDBLOCK
            )
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "[MANUAL] UDP RX error: %s",
                    std::strerror(errno)
                );
            }
        }
    }






    void stop_robot()
    {
        std::string stop_packet =
            "M0L0R0T0U0S0G0Z0E";

        ssize_t sent =
            sendto(
                sock_tx_,
                stop_packet.c_str(),
                stop_packet.size(),
                0,
                reinterpret_cast<sockaddr*>(
                    &tx_addr_
                ),
                sizeof(tx_addr_)
            );

        if (sent >= 0)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "STOP packet sent."
            );
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to send stop packet: %s",
                std::strerror(errno)
            );
        }
    }






    ~ArmStateSubscriber()
    {
        if (sock_tx_ >= 0)
        {
            close(sock_tx_);
            sock_tx_ = -1;
        }

        if (rx_sock_ >= 0)
        {
            close(rx_sock_);
            rx_sock_ = -1;
        }
    }


private:





    int sock_tx_ = -1;
    int rx_sock_ = -1;

    std::string udp_ip_tx_;
    int udp_port_tx_;

    sockaddr_in tx_addr_{};





    std::string ref_string_;
    std::string send_string_;





    bool automate_;

    rclcpp::Service<
        std_srvs::srv::Trigger
    >::SharedPtr automation_service_;

    rclcpp::Publisher<
        std_msgs::msg::Bool
    >::SharedPtr mode_state_pub_;





    double looptime_;

    rclcpp::TimerBase::SharedPtr timer_;





    rclcpp::Subscription<
        geometry_msgs::msg::Twist
    >::SharedPtr cmd_vel_sub_;

    rclcpp::Subscription<
        aruco_msgs::msg::ArmPwm
    >::SharedPtr virtualarm_;

    rclcpp::Subscription<
        aruco_msgs::msg::ArmPwm
    >::SharedPtr realArm_;





    double real_l1_;
    double real_l2_;
    double real_swivel_;

    double sim_l1_;
    double sim_l2_;
    double sim_swivel_;

    int64_t sim_gripper_;

    bool gripperEngage_;





    double link1_error_prev_;
    double link1_error_int_;

    double pid_const_link1_p_;
    double pid_const_link1_i_;
    double pid_const_link1_d_;

    double link2_error_prev_;
    double link2_error_int_;

    double pid_const_link2_p_;
    double pid_const_link2_i_;
    double pid_const_link2_d_;

    double swivel_error_prev_;
    double swivel_error_int_;

    double pid_const_swivel_p_;
    double pid_const_swivel_i_;
    double pid_const_swivel_d_;

    int pwm_l1_;
    int pwm_l2_;
    int pwm_swivel_;





    double linear_;
    double angular_;

    double left_;
    double right_;

    double left_rpm_;
    double right_rpm_;

    double left_pwm_;
    double right_pwm_;

    bool pwm_enabled_;

    double max_linear_velocity_;
};






int main(
    int argc,
    char * argv[])
{
    rclcpp::init(
        argc,
        argv
    );

    auto node =
        std::make_shared<
            ArmStateSubscriber
        >();

    try
    {
        rclcpp::spin(node);
    }
    catch (
        const rclcpp::exceptions::RCLError &
    )
    {

    }
    catch (
        const std::exception &e
    )
    {
        RCLCPP_ERROR(
            node->get_logger(),
            "Exception: %s",
            e.what()
        );
    }

    node->stop_robot();

    rclcpp::shutdown();

    return 0;
}
