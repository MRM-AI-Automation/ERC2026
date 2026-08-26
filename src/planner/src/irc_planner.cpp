#include "planner/irc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace planner
{

SensorCallback::SensorCallback()
: Node("planner_node"),
  vel_pub(nullptr), imu_sub_(nullptr), gps_sub_(nullptr), aruco_sub_(nullptr),
  lidar_sub_(nullptr), auto_sub_(nullptr), toggle_client_(nullptr), stack_timer_(nullptr),
  CurrState(kManualState), FollowPattern(kMoveForward), nav_mode(-1),
  target_aruco_id_(0), nav_select_done_(false), rover_state(false), last_rover_state(false),
  gps_goal_set(false), gps_goal_reached(false), gps_aligned_(false),
  curr_location{0.0, 0.0}, goal_location{0.0, 0.0}, imu_yaw(0.0),
  aruco_detect(false), aruco_goal_reached(false), aruco_x(0.0), aruco_y(0.0),
  obstacle_detect(false), obs_x(0.0), obs_y(0.0),
  search_ref_set_(false), spot_turn_back_(false), spot_done_(false), search_cycle_(0),
  search_end_time_(this->now()), search_forward_time_(4.0), search_skew(kNoSkew),
  avoiding_obstacle_(false), prev_state_(kManualState), prev_search_pattern_(kMoveForward)
{
    declare_parameter("imu_topic", "/imu_data");
    declare_parameter("gps_topic", "/gps");
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

    vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);
    imu_sub_ = create_subscription<aruco_msgs::msg::ImuData>(imu_topic, 10,std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, 10, std::bind(&SensorCallback::gpsCallback, this, std::placeholders::_1));
    aruco_sub_ = create_subscription<aruco_msgs::msg::ArucoTag>(aruco_topic, 10, std::bind(&SensorCallback::arucoCallback, this, std::placeholders::_1));
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(lidar_topic, 10, std::bind(&SensorCallback::lidarCallback, this, std::placeholders::_1));
    auto_sub_ = create_subscription<std_msgs::msg::Bool>(state_topic, 10, std::bind(&SensorCallback::stateCallback, this, std::placeholders::_1));

    stack_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&SensorCallback::stackRun, this));
    toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

    last_gps_time_ = this->now();
    last_aruco_time_ = this->now();
    gps_last_check_time_ = this->now();
}

void SensorCallback::stackRun()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto now = this->get_clock()->now();

    if (last_lidar_scan_) obstacleClassifier();
    if ((now - last_aruco_time_).seconds() > 0.9) aruco_detect = false;

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

    if (nav_mode == 0 && !gps_goal_set) return;
    if (nav_mode == 1 && CurrState == kSearchPattern && target_aruco_id_ <= 0) return;

    RoverStateClassifier();
    setGoalStatus();

    // ArUco obstacle avoidance remains unchanged
    if (nav_mode == 1 && obstacle_detect && CurrState != kObstacleAvoidance)
    {
        prev_state_ = CurrState;
        prev_search_pattern_ = FollowPattern;
        CurrState = kObstacleAvoidance;

        RCLCPP_WARN(get_logger(), "[ARUCO] OBSTACLE DETECTED -> OBSTACLE AVOIDANCE");
    }

    // GPS obstacle avoidance state exists as a placeholder,
    // but GPS navigation does not enter it yet.

    switch (CurrState)
    {
        case kObstacleAvoidance:    obstacleAvoidance(); break;
        case kGPSObstacleAvoidance: gpsObstacleAvoidance(); break;
        case kSearchPattern:        callSearchPattern(); break;
        case kArucoFollowing:       arucoFollowing(); break;
        case kCoordinateFollowing:  coordinateFollowing(); break;
        default: publishVel(geometry_msgs::msg::Twist()); break;
    }
}

void SensorCallback::RoverStateClassifier()
{
    if (CurrState == kObstacleAvoidance || CurrState == kGPSObstacleAvoidance) return;

    if ((nav_mode == 0 && gps_goal_reached) || (nav_mode == 1 && aruco_goal_reached))
    {
        hardStop();
        disableAutonomous();
        CurrState = kManualState;
        return;
    }

    if (nav_mode == 0)
    {
        if (gps_goal_set && !gps_goal_reached) CurrState = kCoordinateFollowing;
        return;
    }

    if (nav_mode == 1)
    {
        CurrState = aruco_detect ? kArucoFollowing : kSearchPattern;
        return;
    }
}

void SensorCallback::imuCallback(
    const aruco_msgs::msg::ImuData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_yaw = msg->orientation.z;
}

void SensorCallback::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix_)
{
    if (fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX) return;
    if (!std::isfinite(fix_->latitude) || !std::isfinite(fix_->longitude)) return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    curr_location.latitude = fix_->latitude;
    curr_location.longitude = fix_->longitude;
    last_gps_time_ = this->get_clock()->now();
}

void SensorCallback::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    if (!rover_state) return;
    last_lidar_scan_ = scan_msg;
}

void SensorCallback::arucoCallback(const aruco_msgs::msg::ArucoTag::SharedPtr msg)
{
    if (!rover_state) return;

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

    if (state->data == last_rover_state) return;

    last_rover_state = state->data;
    rover_state = state->data;

    avoiding_obstacle_ = false;
    publishVel(geometry_msgs::msg::Twist());

    nav_mode = -1;
    nav_select_done_ = false;

    gps_goal_set = false;
    gps_goal_reached = false;
    aruco_goal_reached = false;

    gps_aligned_ = false;
    gps_waiting_ = false;
    gps_last_check_time_ = this->now();

    aruco_detect = false;
    obstacle_detect = false;

    resetSearchPattern();
    search_skew = kNoSkew;

    last_aruco_time_ = this->get_clock()->now();
    last_gps_time_ = this->get_clock()->now();

    if (!rover_state)
    {
        CurrState = kManualState;
        RCLCPP_INFO(this->get_logger(), "[MODE] MANUAL MODE");
        return;
    }

    CurrState = kNavigationModeSelect;
    RCLCPP_INFO(this->get_logger(), "[MODE] AUTONOMOUS MODE");
}

void SensorCallback::navigationModeSelect()
{
    if (nav_select_done_) return;

    publishVel(geometry_msgs::msg::Twist());

    std::cout << "\n=================================\n";
    std::cout << "      NAVIGATION MODE SELECT\n";
    std::cout << "=================================\n";
    std::cout << "0 -> GPS Navigation\n";
    std::cout << "1 -> ArUco Navigation\n";
    std::cout << "Select mode: ";

    std::cin >> nav_mode;

    if (nav_mode == 0)
    {
        std::cout << "\n========== GPS NAVIGATION ==========\n";
        std::cout << "Current GPS position:\n";
        std::cout << "Latitude  : " << curr_location.latitude << "\n";
        std::cout << "Longitude : " << curr_location.longitude << "\n\n";

        std::cout << "Enter goal latitude  : ";
        std::cin >> goal_location.latitude;

        std::cout << "Enter goal longitude : ";
        std::cin >> goal_location.longitude;

        if (goal_location.latitude < -90.0 || goal_location.latitude > 90.0 ||
            goal_location.longitude < -180.0 || goal_location.longitude > 180.0)
        {
            std::cout << "[GPS] ERROR: Invalid coordinates\n";
            nav_mode = -1;
            nav_select_done_ = true;
            CurrState = kManualState;
            return;
        }

        gps_goal_set = true;
        gps_goal_reached = false;

        // Reset GPS heading-following state
        gps_aligned_ = false;
        gps_waiting_ = false;
        gps_last_check_time_ = this->now();

        CurrState = kCoordinateFollowing;

        RCLCPP_INFO(get_logger(), "[NAV] GPS NAVIGATION SELECTED");

        std::cout << "[GPS] Navigation selected, starting coordinate following...\n";
        std::cout << "====================================\n";
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
        int back;
        std::cin >> back;

        search_end_time_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(6.5);
        spot_turn_back_ = (back == 1);

        aruco_detect = false;
        aruco_goal_reached = false;
        obstacle_detect = false;

        resetSearchPattern();

        CurrState = kSearchPattern;

        RCLCPP_INFO(get_logger(), "[NAV] ARUCO NAVIGATION SELECTED");
    }
    else
    {
        std::cout << "[CLI] Invalid navigation selection\n";
        nav_mode = -1;
        CurrState = kManualState;
    }

    nav_select_done_ = true;
}

void SensorCallback::coordinateFollowing()
{
    geometry_msgs::msg::Twist stop_cmd;

    // -----------------------------------------------------
    // No GPS goal
    // -----------------------------------------------------
    if (!gps_goal_set)
    {
        publishVel(stop_cmd);
        return;
    }

    const auto now = get_clock()->now();

    // -----------------------------------------------------
    // GPS freshness check
    // -----------------------------------------------------
    if ((now - last_gps_time_).seconds() > 1.5)
    {
        gps_aligned_ = false;
        gps_waiting_ = false;

        publishVel(stop_cmd);

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[GPS] GPS data stale -> rover stopped");
        return;
    }

    // -----------------------------------------------------
    // Calculate distance using Haversine
    // -----------------------------------------------------
    const double dist = haversine(curr_location, goal_location);

    if (dist <= kDistanceThreshold)
    {
        if (!gps_goal_reached)
        {
            RCLCPP_INFO(get_logger(), "[GPS] GOAL REACHED | Distance: %.2f m", dist);
        }

        gps_goal_reached = true;
        gps_aligned_ = false;
        gps_waiting_ = false;

        hardStop();
        return;
    }

    gps_goal_reached = false;

    // -----------------------------------------------------
    // Calculate target bearing and 0-360 heading error
    // -----------------------------------------------------
    const double target_bearing = gpsBearing(curr_location, goal_location);
    const double error = headingError(target_bearing, imu_yaw);

    // Shortest angular distance for tolerance checking
    const double shortest_error = std::min(error, 360.0 - error);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[GPS] Distance: %.2f m | Target: %.1f deg | Current: %.1f deg | Error: %.1f deg",
        dist, target_bearing, imu_yaw, shortest_error);

    constexpr double HEADING_TOLERANCE = 10.0;
    constexpr double WAIT_TIME = 2.0;
    constexpr double CHECK_TIME = 2.0;
    constexpr double TURN_SPEED = 0.5;

    geometry_msgs::msg::Twist cmd;

    // =====================================================
    // STEP 1: ALIGN WITH THE GPS TARGET
    // =====================================================
    if (!gps_aligned_)
    {
        if (shortest_error <= HEADING_TOLERANCE)
        {
            gps_aligned_ = true;
            gps_waiting_ = true;
            gps_wait_end_ = now + rclcpp::Duration::from_seconds(WAIT_TIME);

            publishVel(stop_cmd);
            return;
        }

        cmd.linear.x = 0.0;

        // error 0-180: turn toward target one way
        // error 180-360: turn the opposite way
        //
        // Negative angular.z turns this rover right,
        // so the command direction follows the existing
        // rover convention.
        if (error <= 180.0)
            cmd.angular.z = -TURN_SPEED;
        else
            cmd.angular.z = TURN_SPEED;

        publishVel(cmd);
        return;
    }

    // =====================================================
    // STEP 2: STOP AND WAIT 2 SECONDS
    // =====================================================
    if (gps_waiting_)
    {
        publishVel(stop_cmd);

        if (now < gps_wait_end_) return;

        // Still within 10 degrees after waiting
        if (shortest_error > HEADING_TOLERANCE)
        {
            gps_aligned_ = false;
            gps_waiting_ = false;
            return;
        }

        gps_waiting_ = false;
        gps_last_check_time_ = now;

        RCLCPP_INFO(get_logger(), "[GPS] Aligned -> driving");
        return;
    }

    // =====================================================
    // STEP 3: DRIVE STRAIGHT AND CHECK EVERY 2 SECONDS
    // =====================================================
    if ((now - gps_last_check_time_).seconds() >= CHECK_TIME)
    {
        gps_last_check_time_ = now;

        if (shortest_error > HEADING_TOLERANCE)
        {
            gps_aligned_ = false;
            gps_waiting_ = false;

            publishVel(stop_cmd);

            RCLCPP_WARN(get_logger(), "[GPS] Heading lost -> realigning");
            return;
        }
    }

    // Heading is acceptable: drive straight
    const double speed = std::clamp(0.25 + (0.10 * dist), 0.25, 1.0);

    cmd.linear.x = speed;
    cmd.angular.z = 0.0;

    publishVel(cmd);
}

void SensorCallback::obstacleAvoidance()
{
    if (obstacle_detect)
    {
        avoiding_obstacle_ = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = (obs_side_ && std::string(obs_side_) == "left") ? -1.0 : 1.0;

        publishVel(cmd);
        return;
    }

    avoiding_obstacle_ = false;

    CurrState = kSearchPattern;
    resetSearchPattern();

    RCLCPP_INFO(get_logger(), "[AVOID] obstacle cleared -> SEARCH PATTERN");
}

void SensorCallback::gpsObstacleAvoidance()
{
    // Placeholder for future GPS obstacle avoidance.
    // No obstacle avoidance logic implemented yet.
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
        get_logger(), *get_clock(), 1000,
        "[ARUCO] Following | Distance: %.2f m | Error: %.2f",
        aruco_x, aruco_y);

    publishVel(cmd);
}

void SensorCallback::callSearchPattern()
{
    geometry_msgs::msg::Twist cmd;
    auto clock = this->get_clock();
    auto now = clock->now();

    const double ang_vel = 1.0;
    const double lin_vel = 0.65;

    if (aruco_detect) return;

    if (spot_turn_back_ && !spot_done_)
    {
        cmd.angular.z = ang_vel;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[SEARCH][SPOT] Turning in place | ang=%.2f", cmd.angular.z);

        if (now >= search_end_time_)
        {
            spot_done_ = true;
            search_ref_set_ = false;
            RCLCPP_INFO(get_logger(), "[SEARCH][SPOT] turn done -> start pattern");
        }
        return;
    }

    if (!search_ref_set_)
    {
        FollowPattern = kMoveForward;
        search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
        search_ref_set_ = true;

        RCLCPP_INFO(get_logger(), "[SEARCH] Starting the search pattern, moving forward");

        geometry_msgs::msg::Twist c;
        c.linear.x = lin_vel;
        publishVel(c);
        return;
    }

    if (FollowPattern == kTurnA)
    {
        const bool right_skew = (search_skew == kRightSkew);
        cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[SEARCH][TURN A] ang=%.2f skew=%d cycle=%d", cmd.angular.z, search_skew, search_cycle_);

        if (now >= search_end_time_)
        {
            FollowPattern = kTurnB;
            search_end_time_ = now + rclcpp::Duration::from_seconds(7.0);
        }
        return;
    }

    if (FollowPattern == kTurnB)
    {
        const bool right_skew = (search_skew == kRightSkew);
        cmd.angular.z = right_skew ? +ang_vel : -ang_vel;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[SEARCH][TURN B] ang=%.2f skew=%d cycle=%d", cmd.angular.z, search_skew, search_cycle_);

        if (now >= search_end_time_)
        {
            FollowPattern = kTurnC;
            double extra = (search_skew != kNoSkew) ? static_cast<double>(search_cycle_) : 0.0;
            search_end_time_ = now + rclcpp::Duration::from_seconds(3.5 + extra);
        }
        return;
    }

    if (FollowPattern == kTurnC)
    {
        const bool right_skew = (search_skew == kRightSkew);
        cmd.angular.z = right_skew ? -ang_vel : +ang_vel;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[SEARCH][TURN C] ang=%.2f skew=%d cycle=%d", cmd.angular.z, search_skew, search_cycle_);

        if (now >= search_end_time_)
        {
            FollowPattern = kMoveForward;
            search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
        }
        return;
    }

    if (FollowPattern == kMoveForward)
    {
        cmd.linear.x = lin_vel;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[SEARCH][FORWARD] lin=%.2f cycle=%d skew=%d", cmd.linear.x, search_cycle_, search_skew);

        if (now >= search_end_time_)
        {
            search_cycle_++;
            FollowPattern = kTurnA;
            search_end_time_ = now + rclcpp::Duration::from_seconds(3.5);
        }
        return;
    }
}

void SensorCallback::publishVel(const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::Twist cmd = msg;

    cmd.linear.x = std::clamp(cmd.linear.x, 0.0, kMaxLinearVel);
    if (cmd.linear.x > 0.0 && cmd.linear.x < kMinLinearVel)
        cmd.linear.x = kMinLinearVel;

    if (std::abs(cmd.angular.z) < 1e-3)
    {
        cmd.angular.z = 0.0;
    }
    else
    {
        cmd.angular.z = std::clamp(cmd.angular.z, -kMaxAngularVel, kMaxAngularVel);
        if (std::abs(cmd.angular.z) < kMinAngularVel)
            cmd.angular.z = std::copysign(kMinAngularVel, cmd.angular.z);
    }

    vel_pub->publish(cmd);
}

void SensorCallback::hardStop()
{
    geometry_msgs::msg::Twist stop;

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
                RCLCPP_INFO(this->get_logger(), "[MODE] Autonomous DISABLED via service");
            else
                RCLCPP_ERROR(this->get_logger(), "[MODE] Failed to disable autonomy: %s", res->message.c_str());
        });
}

void SensorCallback::obstacleClassifier()
{
    if (!last_lidar_scan_)
    {
        obstacle_detect = false;
        obs_x = 0.0;
        obs_y = 0.0;
        return;
    }

    bool found = false;
    float best_x = std::numeric_limits<float>::max();
    float x = 0.0f;
    float y = 0.0f;

    constexpr float min_x = 0.4f;
    constexpr float max_x = 2.0f;
    constexpr float half_w = 1.20f;
    constexpr float aruco_mask_radius = 0.40f;

    const auto &scan = *last_lidar_scan_;
    float angle = scan.angle_min;

    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment)
    {
        const float r = scan.ranges[i];

        if (r < scan.range_min || r > scan.range_max || !std::isfinite(r)) continue;

        const float px = r * std::cos(angle);
        const float py = r * std::sin(angle);

        if (aruco_detect)
        {
            const float dx = px - static_cast<float>(aruco_x);
            const float dy = py - static_cast<float>(aruco_y);

            if (std::sqrt(dx * dx + dy * dy) < aruco_mask_radius) continue;
        }

        if (px < min_x || px > max_x || std::abs(py) > half_w) continue;

        if (px < best_x)
        {
            best_x = px;
            x = px;
            y = py;
            found = true;
        }
    }

    obstacle_detect = found;

    if (found)
    {
        obs_x = x;
        obs_y = y;
        obs_side_ = (obs_y > 0.15) ? "left" : (obs_y < -0.15) ? "right" : "center";
    }
    else
    {
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "center";
    }
}

void SensorCallback::setGoalStatus()
{
    static int valid_count = 0;

    if (nav_mode != 1 || CurrState != kArucoFollowing || aruco_goal_reached || !aruco_detect) return;

    if (aruco_x >= 0.0 && aruco_x <= kDistanceThreshold)
        ++valid_count;
    else
        valid_count = 0;

    if (valid_count >= 5)
    {
        aruco_goal_reached = true;
        RCLCPP_INFO(this->get_logger(), "[ARUCO] Goal reached");
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

double SensorCallback::gpsBearing(Coordinates curr, Coordinates dest)
{
    double lat1 = curr.latitude * M_PI / 180.0;
    double lon1 = curr.longitude * M_PI / 180.0;
    double lat2 = dest.latitude * M_PI / 180.0;
    double lon2 = dest.longitude * M_PI / 180.0;

    double dLon = lon2 - lon1;

    double x = sin(dLon) * cos(lat2);
    double y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double bearing = atan2(x, y) * 180.0 / M_PI;

    if (bearing < 0.0) bearing += 360.0;

    return bearing;
}

double SensorCallback::headingError(double target, double current)
{
    double error = std::fmod(target - current + 360.0, 360.0);

    if (error < 0.0) error += 360.0;

    return error;
}

} // namespace planner
