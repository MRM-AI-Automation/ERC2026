#include "planner/irc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace planner
{

    // Constructor

        SensorCallback::SensorCallback()
        : Node("planner_node"),

        vel_pub(nullptr),
        imu_sub_(nullptr),
        external_imu_sub_(nullptr),
        gps_sub_(nullptr),
        aruco_sub_(nullptr),
        lidar_sub_(nullptr),
        auto_sub_(nullptr),
        toggle_client_(nullptr),
        stack_timer_(nullptr),
        CurrState(kManualState),
        FollowPattern(kMoveForward),
        nav_mode(-1),
        target_aruco_id_(0),
        nav_select_done_(false),
        rover_state(false),
        last_rover_state(false),
        gps_goal_set(false),
        gps_goal_reached(false),
        gps_aligned_(false),
        curr_location{0.0, 0.0},
        goal_location{0.0, 0.0},
        aruco_detect(false),
        aruco_goal_reached(false),
        aruco_x(0.0),
        aruco_y(0.0),
        obstacle_detect(false),
        obs_x(0.0),
        obs_y(0.0),
        search_ref_set_(false),
        spot_turn_back_(false),
        spot_done_(false),
        search_cycle_(0),
        search_end_time_(this->now()),
        search_forward_time_(4.0),
        search_skew(kNoSkew),
        avoiding_obstacle_(false),
        prev_state_(kManualState),
        prev_search_pattern_(kMoveForward),
        zed_yaw(0.0),
        bno_yaw(0.0),
        current_orientation(0.0)

    {
        declare_parameter("imu_topic", "/imu_data");
        declare_parameter("gps_topic", "/fix");
        declare_parameter("aruco_topic", "/aruco_detected");
        declare_parameter("lidar_topic", "/scan");
        declare_parameter("cmd_vel_topic", "/cmd_vel");
        declare_parameter("state_topic", "/autonomous_mode_state");
        declare_parameter("target_aruco_id", 1);

        const auto imu_topic = get_parameter("imu_topic").as_string();
        const auto gps_topic = get_parameter("gps_topic").as_string();
        const auto aruco_topic = get_parameter("aruco_topic").as_string();
        const auto lidar_topic = get_parameter("lidar_topic").as_string();
        const auto cmd_vel = get_parameter("cmd_vel_topic").as_string();
        const auto state_topic = get_parameter("state_topic").as_string();

        target_aruco_id_ = get_parameter("target_aruco_id").as_int();

        // Publishers
        vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);

        // Subscribers
        imu_sub_ = create_subscription<aruco_msgs::msg::ImuData>(
            imu_topic, 10, std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));

        external_imu_sub_ = create_subscription<aruco_msgs::msg::ImuData>(
            "/external_imu", 10, std::bind(&SensorCallback::externalImuCallback, this, std::placeholders::_1));

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic, 10, std::bind(&SensorCallback::gpsCallback, this, std::placeholders::_1));

        aruco_sub_ = create_subscription<aruco_msgs::msg::ArucoTag>(
            aruco_topic, 10, std::bind(&SensorCallback::arucoCallback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic, 10, std::bind(&SensorCallback::lidarCallback, this, std::placeholders::_1));

        auto_sub_ = create_subscription<std_msgs::msg::Bool>(
            state_topic, 10, std::bind(&SensorCallback::stateCallback, this, std::placeholders::_1));

        // Timers & Services
        stack_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&SensorCallback::stackRun, this));
        toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

        last_gps_time_ = this->now();
        last_aruco_time_ = this->now();

        obj_follow_linear = straightLineEquation(0.0, 0.0, 5.0, kMaxLinearVel);
        obj_follow_angular = straightLineEquation(0.0, 0.0, 0.7, kMaxAngularVel);
    }

    // Core Functions :

    // Planner Main control loop
    void SensorCallback::stackRun()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    current_orientation = zed_yaw;
    auto now = this->get_clock()->now();

    if (last_lidar_scan_)
    obstacleClassifier();

    if ((now - last_aruco_time_).seconds() > 0.9)
        aruco_detect = false;

    if (!rover_state)
    {
        publishVel(geometry_msgs::msg::Twist());
        return;
    }

    if (CurrState == kNavigationModeSelect)
    {
        navigationModeSelect();
        return;
    }

    if (nav_mode == 0 && !gps_goal_set)
        return;

    if (nav_mode == 1 && CurrState == kSearchPattern && target_aruco_id_ <= 0)
        return;

    RoverStateClassifier();
    setGoalStatus();

    if (obstacle_detect &&
    CurrState != kObstacleAvoidance)
    {
        prev_state_ = CurrState;
        prev_search_pattern_ = FollowPattern;
        CurrState = kObstacleAvoidance;
    }


    switch (CurrState)
    {
        case kObstacleAvoidance:
            obstacleAvoidance();
            break;

        case kSearchPattern:
            callSearchPattern();
            break;

        case kArucoFollowing:
            arucoFollowing();
            break;

        case kCoordinateFollowing:
            coordinateFollowing();
            break;

        default:
            publishVel(geometry_msgs::msg::Twist());
            break;
    }
}


    // Planner state decision logic
    void SensorCallback::RoverStateClassifier()
    {
        // Priority states: never override them
        if (CurrState == kObstacleAvoidance)
            return;

        // Goal reached → stop and go manual
        if ((nav_mode == 0 && gps_goal_reached) ||
            (nav_mode == 1 && aruco_goal_reached))
        {
            hardStop();
            disableAutonomous();
            CurrState = kManualState;
            return;
        }

        // GPS navigation
        if (nav_mode == 0)
        {
            if (gps_goal_set && !gps_goal_reached)
                CurrState = kCoordinateFollowing;

            return;
        }

        // ArUco navigation
        if (nav_mode == 1)
        {
            if (aruco_detect)
            {
                CurrState = kArucoFollowing;
                return;
            }

            CurrState = kSearchPattern;
            return;
        }
    }

    // Callbacks:-

    void SensorCallback::imuCallback(const aruco_msgs::msg::ImuData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        zed_yaw = normalize360(msg->orientation.z);
    }

    void SensorCallback::externalImuCallback(const aruco_msgs::msg::ImuData::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        bno_yaw = normalize360(msg->orientation.z);
    }

    void SensorCallback::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix_)
    {
        if (fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) // STATUS_FIX means at least a 2D GPS fix
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);

        curr_location.latitude = fix_->latitude;
        curr_location.longitude = fix_->longitude;
        last_gps_time_ = this->get_clock()->now();
	std::cout<<curr_location.latitude<<":"<<curr_location.longitude<<std::endl;
}

    void SensorCallback::lidarCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (!rover_state)
            return;

        last_lidar_scan_ = scan_msg;
    }

    void SensorCallback::arucoCallback(const aruco_msgs::msg::ArucoTag::SharedPtr msg)
    {
        if (!rover_state)
            return;

        std::lock_guard<std::mutex> lock(state_mutex_);

        if (msg->is_detected && msg->id == target_aruco_id_)
        {
            aruco_detect = true;
            aruco_x = static_cast<double>(msg->aruco_x);
            aruco_y = static_cast<double>(msg->aruco_y);
            last_aruco_time_ = this->get_clock()->now();
        }
    }

    void SensorCallback::stateCallback(const std_msgs::msg::Bool::SharedPtr state)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (state->data == last_rover_state)
            return;

        last_rover_state = state->data;
        rover_state = state->data;

        if (!rover_state)
        {
            avoiding_obstacle_ = false;

            publishVel(geometry_msgs::msg::Twist());

            CurrState = kManualState;
            nav_mode = -1;
            nav_select_done_ = false;

            gps_goal_set = false;
            gps_goal_reached = false;
            aruco_goal_reached = false;
            gps_aligned_ = false;

            aruco_detect = false;
            obstacle_detect = false;

            resetSearchPattern();
            search_skew = kNoSkew;

            last_aruco_time_ = this->get_clock()->now();
            last_gps_time_ = this->get_clock()->now();

            RCLCPP_INFO(this->get_logger(), "[MODE] MANUAL MODE");
            return;
        }

        avoiding_obstacle_ = false;

        CurrState = kNavigationModeSelect;

        nav_mode = -1;
        nav_select_done_ = false;

        gps_goal_set = false;
        gps_goal_reached = false;
        aruco_goal_reached = false;
        gps_aligned_ = false;

        aruco_detect = false;
        obstacle_detect = false;

        resetSearchPattern();
        search_skew = kNoSkew;

        last_aruco_time_ = this->get_clock()->now();
        last_gps_time_ = this->get_clock()->now();

        RCLCPP_INFO(this->get_logger(),
                    "[MODE] AUTONOMOUS MODE → NAVIGATION SELECT");
    }


    // States :

    // Aligns to GPS goal and drives toward it until the target is reached.

    // Used for selection of the navigation mode and other variables
    void SensorCallback::navigationModeSelect()
    {
        if (nav_select_done_)
            return;

        publishVel(geometry_msgs::msg::Twist());

        std::cout << "\nNAVIGATION MODE SELECT\n";
        std::cout << "0 → GPS Navigation\n";
        std::cout << "1 → ArUco Navigation\n";
        std::cout << "Select mode: ";
        std::cin >> nav_mode;

        if (nav_mode == 0)
        {
            std::cout << "Enter goal latitude  : ";
            std::cin >> goal_location.latitude;
            std::cout << "Enter goal longitude : ";
            std::cin >> goal_location.longitude;

            gps_goal_set = true;
            gps_goal_reached = false;
            gps_aligned_ = false;

            CurrState = kCoordinateFollowing;

            RCLCPP_INFO(
                this->get_logger(),
                "[NAV][GPS] Goal saved | lat=%.7f lon=%.7f",
                goal_location.latitude,
                goal_location.longitude);

            std::cout << "[CLI] GPS navigation selected\n";
        }
        else if (nav_mode == 1)
        {
            std::cout << "Enter target ArUco ID : ";
            std::cin >> target_aruco_id_;

            std::cout << "Search skew (-1 = LEFT, 0 = NONE, 1 = RIGHT): ";
            int skew;
            std::cin >> skew;
            setSearchSkew(skew);

            std::cout << "Forward search time (sec): ";
            std::cin >> search_forward_time_;

            std::cout << "Spot turn back before search? (0/1): ";
            search_end_time_ =
                this->get_clock()->now() + rclcpp::Duration::from_seconds(6.5);
            int back;
            std::cin >> back;
            spot_turn_back_ = (back == 1);

            // Reset runtime flags
            aruco_detect = false;
            aruco_goal_reached = false;
            obstacle_detect = false;

            resetSearchPattern();
            search_ref_set_ = false;
            search_aligned_ = false;

            CurrState = kSearchPattern;
            std::cout << "[CLI] ArUco navigation selected → SEARCH\n";
        }
        else
        {
            CurrState = kManualState;
            std::cout << "[CLI] Invalid selection → MANUAL\n";
        }

        nav_select_done_ = true;
    }

    void SensorCallback::coordinateFollowing()
    {
        std::cout << "yes coordintate following is going" << std::endl;
        if (!gps_goal_set)
        {
//            std::cout << "lmao" << std::endl;

            publishVel(geometry_msgs::msg::Twist()); // Zero vel gng
            return;
        }

/*
        auto now = this->get_clock()->now();

        if ((now - last_gps_time_).seconds() > 1.5)
        {
            gps_aligned_ = false;
            bearing_locked_ = false;
            publishVel(geometry_msgs::msg::Twist()); // stale gps
            return;
        }
*/
        double dist = haversine(curr_location, goal_location);
        double currangle_coordF = bno_yaw;

        if (dist <= 4.8)
        {
            gps_goal_reached = true;
            gps_aligned_ = false;
            bearing_locked_ = false;

            RCLCPP_INFO(this->get_logger(),
                        "[GPS] Goal reached | remaining_dist=%.2f m", dist);

            hardStop();
            return;
        }
/*
        double minDistanceObstacle = 10000;
        double minDistanceObstacleAngle = 180;
        for (int i = 0; i < obstacleDataType1.size(); i++)
        {

            if (minDistanceObstacle > obstacleDataType1[i][0])
            {
                minDistanceObstacle = obstacleDataType1[i][0];
                minDistanceObstacleAngle = obstacleDataType1[i][1];
            }
            // RCLCPP_INFO(this->get_logger(),
            //             "NoOfObstacle %d:  distance=%.2f m, angle=%.2f°",
            //             i + 1, obstacleDataType1[i][0], obstacleDataType1[i][1]);
        }
*/

        double destAngle_local = gpsBearing(curr_location, goal_location);
        bool angleSetted = false;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0;
        cmd.angular.z = 0;
        double angle_diff = abs(destAngle_local - currangle_coordF);

        if (angle_diff < 52)
        {
            angleSetted = true;
        }
        else
        {
            angleSetted = false;
        }

        angle_diff = destAngle_local - currangle_coordF;
        angle_diff = angle_diff - 360.0 * floor((angle_diff + 180.0) / 360.0);

        double speedx = 0.0;
        double anglex = 0.0;

        if (angleSetted)
        {
            std::cout<<"angle setted"<<std::endl;
            speedx = std::clamp(0.4 + 0.2 * dist, 0.4, 2.0);

            if (angle_diff > 0)
            {
                anglex = -abs(pow(dist, 3) / 100);
                if (abs(anglex) > 1.5) anglex = -1.5;
                if (abs(anglex) < 0.3) anglex = -0.4;
            }
            else if (angle_diff < 0)
            {
                anglex = abs(pow(dist, 3) / 100);
                if (abs(anglex) > 1.5) anglex = 1.5;
                if (abs(anglex) < 0.3) anglex = 0.4;
            }
        }
        else
        {
            std::cout << " not setted" << std::endl;

            if (angle_diff > 0)
            {
                anglex = -abs(pow(dist, 3) / 100);
                if (abs(anglex) > 1.5) anglex = -1.5;
                if (abs(anglex) < 0.25) anglex = -0.4;
            }
            else if (angle_diff < 0)
            {
                anglex = abs(pow(dist, 3) / 100);
                if (abs(anglex) > 1.5) anglex = 1.5;
                if (abs(anglex) < 0.25) anglex = 0.4;
            }
        }


        cmd.linear.x = speedx;
        cmd.angular.z = anglex;

        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000,"[GPS][TRACK] dist=%.2f | heading angle=%.2f deg | angle=%.2f deg | lin=%.2f | ang=%.2f",dist, destAngle_local, currangle_coordF, cmd.linear.x, cmd.angular.z);
        publishVel(cmd);
    }

    void SensorCallback::obstacleAvoidance()
{
    if (obstacle_detect)
    {
        obstacle_clear_timing_ = false;
        avoiding_obstacle_ = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.angular.z = -1.0;

        RCLCPP_WARN(
            get_logger(),
            "[AVOID] avoiding obstacle x=%.2f y=%.2f ang=%.2f",
            obs_x, obs_y, cmd.angular.z);

        publishVel(cmd);
        return;
    }

    avoiding_obstacle_ = false;

    if (prev_state_ == kArucoFollowing && !aruco_detect)
    {
        CurrState = kSearchPattern;
        resetSearchPattern();
    }
    else if (prev_state_ == kSearchPattern)
    {
        CurrState = kSearchPattern;
        resetSearchPattern();
    }
    else
    {
        CurrState = prev_state_;
    }
}



	void SensorCallback::arucoFollowing()
    {
        if (!aruco_detect)
        {
            publishVel(geometry_msgs::msg::Twist());
            return;
        }

        geometry_msgs::msg::Twist cmd;

        constexpr double k_ang = 0.7;
        constexpr double y_deadband = 0.06;
        constexpr double max_ang = 0.7;
        constexpr double constant_lin = 0.8;

        double ang_err = aruco_y;
        if (std::abs(ang_err) < y_deadband) ang_err = 0.0;

        cmd.angular.z = std::clamp(-k_ang * ang_err, -max_ang, max_ang);

        if (aruco_x == -1.0)
            cmd.linear.x = 0.8;
        else if (aruco_x <= kDistanceThreshold)
            cmd.linear.x = 0.0;
        else
            cmd.linear.x = std::min(constant_lin, kMaxLinearVel);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 300,
            "[ARUCO][FOLLOW] x=%.2f y=%.2f lin=%.2f ang=%.2f",
            aruco_x, aruco_y, cmd.linear.x, cmd.angular.z);

        publishVel(cmd);
    }


    void SensorCallback::callSearchPattern()
    {
        geometry_msgs::msg::Twist cmd;
        auto clock = this->get_clock();
        auto now = clock->now();


        const double ang_vel = 1.0;
        const double lin_vel = 0.65;

        if (aruco_detect)
            return;

        if (spot_turn_back_ && !spot_done_)
        {
            cmd.angular.z = ang_vel;
            publishVel(cmd);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *clock, 1000,
                "[SEARCH][SPOT] Turning in place | ang=%.2f", cmd.angular.z);

            if (now >= search_end_time_)
            {
                spot_done_ = true;
                search_ref_set_ = false;

                RCLCPP_INFO(get_logger(),"[SEARCH][SPOT] 8s turn done → start pattern");
            }
            return;
        }

        if (!search_ref_set_)
        {
            FollowPattern = kMoveForward;
            search_end_time_ =
                now + rclcpp::Duration::from_seconds(search_forward_time_);
            search_ref_set_ = true;
            RCLCPP_INFO(get_logger(),"[SEARCH] Starting the search pattern, moving forward");
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = lin_vel;
            publishVel(cmd);
            return;
        }



        if (FollowPattern == kTurnA)
        {
            const bool right_skew = (search_skew == kRightSkew);
            cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
            publishVel(cmd);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *clock, 1000,
                "[SEARCH][TURN A] ang=%.2f skew=%d cycle=%d",
                cmd.angular.z, search_skew, search_cycle_);

            if (now >= search_end_time_)
            {
                FollowPattern = kTurnB;
                search_end_time_ =
                    now + rclcpp::Duration::from_seconds(7.0);
            }
            return;
        }

        if (FollowPattern == kTurnB)
        {
            const bool right_skew = (search_skew == kRightSkew);
            cmd.angular.z = right_skew ? +ang_vel : -ang_vel;
            publishVel(cmd);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *clock, 1000,
                "[SEARCH][TURN B] ang=%.2f skew=%d cycle=%d",
                cmd.angular.z, search_skew, search_cycle_);

            if (now >= search_end_time_)
            {
                FollowPattern = kTurnC;

                double extra = 0.0;
                if (search_skew != kNoSkew)
                    extra = search_cycle_;

                search_end_time_ =
                    now + rclcpp::Duration::from_seconds(3.5 + extra);
            }
            return;
        }

        if (FollowPattern == kTurnC)
        {
            const bool right_skew = (search_skew == kRightSkew);
            cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
            publishVel(cmd);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *clock, 1000,
                "[SEARCH][TURN C] ang=%.2f skew=%d cycle=%d",
                cmd.angular.z, search_skew, search_cycle_);

            if (now >= search_end_time_)
            {
                FollowPattern = kMoveForward;
                search_end_time_ =
                    now + rclcpp::Duration::from_seconds(search_forward_time_);
            }
            return;
        }

        if (FollowPattern == kMoveForward)
        {
            cmd.linear.x = lin_vel;
            publishVel(cmd);

            RCLCPP_INFO_THROTTLE(
                get_logger(), *clock, 1000,
                "[SEARCH][FORWARD] lin=%.2f cycle=%d skew=%d",
                cmd.linear.x, search_cycle_, search_skew);

            if (now >= search_end_time_)
            {
                search_cycle_++;
                FollowPattern = kTurnA;
                search_end_time_ =
                    now + rclcpp::Duration::from_seconds(3.5);
            }
            return;
        }
    }

    // Helpers :

    void SensorCallback::publishVel(const geometry_msgs::msg::Twist &msg)
    {
        geometry_msgs::msg::Twist cmd = msg;

        cmd.linear.x = std::clamp(cmd.linear.x, 0.0, kMaxLinearVel);

        if (std::abs(cmd.angular.z) < 1e-3)
            cmd.angular.z = 0.0;
        else
            cmd.angular.z = std::clamp(cmd.angular.z, -kMaxAngularVel, kMaxAngularVel);

        vel_pub->publish(cmd);
    }

    void SensorCallback::hardStop()
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.linear.y = 0.0;
        stop.linear.z = 0.0;
        stop.angular.x = 0.0;
        stop.angular.y = 0.0;
        stop.angular.z = 0.0;

        for (int i = 0; i < 5; ++i)
        {
            vel_pub->publish(stop);
            rclcpp::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void SensorCallback::disableAutonomous()
    {
        if (!toggle_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "[MODE] toggle_autonomous service not available");
            return;
        }

        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        toggle_client_->async_send_request(
            req,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
            {
                auto res = future.get();
                if (res->success)
                {
                    RCLCPP_INFO(this->get_logger(), "[MODE] Autonomous DISABLED via service");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "[MODE] Failed to disable autonomy: %s", res->message.c_str());
                }
            });
    }

    void SensorCallback::obstacleClassifier()
{
    auto clock = this->get_clock();

    if (!last_lidar_scan_) {
        obstacle_detect = false;
        return;
    }

    bool found = false;
    float best_x = std::numeric_limits<float>::max();
    float x = 0.0f, y = 0.0f;

    constexpr float min_x = 0.5f;
    constexpr float max_x = 3.0f;
    constexpr float half_w = 0.40f;

    const auto& scan = *last_lidar_scan_;
    float angle = scan.angle_min;

    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment)
    {
        float r = scan.ranges[i];

        if (r < scan.range_min || r > scan.range_max)
            continue;
        if (!std::isfinite(r))
            continue;

        // Convert polar to Cartesian (laser frame: x forward, y left)
        float px = r * std::cos(angle);
        float py = r * std::sin(angle);

        if (px < min_x || px > max_x) continue;
        if (std::abs(py) > half_w) continue;

        if (px < best_x) {
            best_x = px;
            x = px;
            y = py;
            found = true;
        }
    }

    obstacle_detect = found;
    obs_x = x;
    obs_y = y;

    if (obstacle_detect)
    {
        const float dist = std::hypot(obs_x, obs_y);
        const char *side =
            (obs_y > 0.15f) ? "left" :
            (obs_y < -0.15f) ? "right" :
                               "center";

        RCLCPP_INFO_THROTTLE(
            get_logger(), *clock, 300,
            "[OBS] Obstacle detected | x=%.2f y=%.2f dist=%.2f side=%s",
            obs_x, obs_y, dist, side);
    }
}

    void SensorCallback::setGoalStatus()
{
    static int valid_count = 0;

    if (nav_mode != 1 ||
        CurrState != kArucoFollowing ||
        aruco_goal_reached ||
        !aruco_detect)
        return;

    if (aruco_x >= 0.0 &&
        aruco_x <= kDistanceThreshold)
    {
        ++valid_count;
    }
    else
    {
        valid_count = 0;
    }

    if (valid_count >= 5)
    {
        aruco_goal_reached = true;
        RCLCPP_INFO(
            this->get_logger(),
            "[ARUCO] Goal reached");
    }
}



    void SensorCallback::setSearchSkew(int skew)
    {
        if (skew == kLeftSkew)
            search_skew = kLeftSkew;
        else if (skew == kRightSkew)
            search_skew = kRightSkew;
        else
            search_skew = kNoSkew;
    }

    void SensorCallback::resetSearchPattern()
    {
        FollowPattern = kMoveForward;
        search_ref_set_ = false;
        spot_done_ = false;
        search_cycle_ = 0;
    }


    // Math Functions :

    // Computes the straight-line equation passing through two points (x1,y1) and (x2,y2)
    std::vector<double> SensorCallback::straightLineEquation(double x1, double y1, double x2, double y2)
    {
        std::vector<double> eq(2);
        if (std::abs(x2 - x1) < 1e-6)
        {
            eq[0] = 0.0;
            eq[1] = y1;
            return eq;
        }

        double m = (y2 - y1) / (x2 - x1);
        double c = y1 - m * x1;

        eq[0] = m;
        eq[1] = c;
        return eq;
    }

    // Computes the distance (in meters) between two GPS coordinates
    double SensorCallback::haversine(Coordinates curr, Coordinates dest)
    {
        double lat1 = curr.latitude * M_PI / 180.0;
        double lat2 = dest.latitude * M_PI / 180.0;
        double dLat = lat2 - lat1;
        double dLon = (dest.longitude - curr.longitude) * M_PI / 180.0;

        double h = sin(dLat * 0.5) * sin(dLat * 0.5) +
                   cos(lat1) * cos(lat2) * sin(dLon * 0.5) * sin(dLon * 0.5);

        return 2.0 * 6371000.0 * asin(sqrt(h));
    }

    // Computes the initial bearing (in degrees) from the current GPS coordinate to the destination GPS coordinate,
    // measured clockwise from geographic North and normalized to the range [0, 360).
    double SensorCallback::gpsBearing(Coordinates curr, Coordinates dest)
    {
        double lat1 = curr.latitude * M_PI / 180.0;
        double lon1 = curr.longitude * M_PI / 180.0;
        double lat2 = dest.latitude * M_PI / 180.0;
        double lon2 = dest.longitude * M_PI / 180.0;

        double dLon = lon2 - lon1;

        double x = sin(dLon) * cos(lat2);
        double y = cos(lat1) * sin(lat2) -
                   sin(lat1) * cos(lat2) * cos(dLon);

        double angle = atan2(x, y) * 180.0 / M_PI;
        if (angle < 0)
            angle += 360.0;

        return angle;
    }

    // Converts a GPS bearing (0° = North, clockwise positive) into the rover's
    // internal heading convention by applying a +90° frame shift, sign inversion,
    // and wrapping the result to the range [-180, 180].
    double SensorCallback::gpsAngleFix(double angle)
    {
        double bearing = fmod((angle + 90.0), 360.0);
        double toAngle = -bearing;

        if (toAngle > 180.0)
            toAngle -= 360.0;
        if (toAngle < -180.0)
            toAngle += 360.0;

        return toAngle;
    }

    // Computes the shortest signed angular difference (in degrees) between a target
    // heading and the current heading, normalized to the range [-180, 180] so that
    // the result represents the minimal rotation direction and magnitude.
    double SensorCallback::headingError(double target, double current)
    {
        double diff = target - current;

        if (diff > 180.0)
            diff -= 360.0;
        if (diff < -180.0)
            diff += 360.0;

        return diff;
    }

    // Normalizes any input angle (in degrees) into the range [0, 360)
    double SensorCallback::normalize360(double angle)
    {
        angle = fmod(angle, 360.0);
        if (angle < 0.0)
            angle += 360.0;
        return angle;
    }

} // namespace planner
