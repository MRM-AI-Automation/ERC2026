#include "planner/irc_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace planner
{

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

    imu_sub_ = create_subscription<aruco_msgs::msg::ImuData>(
        imu_topic, 10, std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));

    external_imu_sub_ = create_subscription<aruco_msgs::msg::ImuData>(
        "/imu_data", 10, std::bind(&SensorCallback::externalImuCallback, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, 10, std::bind(&SensorCallback::gpsCallback, this, std::placeholders::_1));

    aruco_sub_ = create_subscription<aruco_msgs::msg::ArucoTag>(
        aruco_topic, 10, std::bind(&SensorCallback::arucoCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        lidar_topic, 10, std::bind(&SensorCallback::lidarCallback, this, std::placeholders::_1));

    auto_sub_ = create_subscription<std_msgs::msg::Bool>(
        state_topic, 10, std::bind(&SensorCallback::stateCallback, this, std::placeholders::_1));

    stack_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&SensorCallback::stackRun, this));
    toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

    last_gps_time_ = this->now();
    last_aruco_time_ = this->now();

    obj_follow_linear = straightLineEquation(0.0, 0.0, 5.0, kMaxLinearVel);
    obj_follow_angular = straightLineEquation(0.0, 0.0, 0.7, kMaxAngularVel);
}

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

    if (nav_mode == 1 && obstacle_detect && CurrState != kObstacleAvoidance)
    {
        prev_state_ = CurrState;
        prev_search_pattern_ = FollowPattern;
        CurrState = kObstacleAvoidance;
    }

    if (nav_mode == 0 && obstacle_detect && CurrState == kCoordinateFollowing)
    {
        gps_wall_following_active_ = true;
        CurrState = kGPSObstacleAvoidance;

        RCLCPP_WARN(get_logger(), "[GPS] OBSTACLE DETECTED -> GPS OBSTACLE AVOIDANCE");
    }

    switch (CurrState)
    {
        case kObstacleAvoidance:
            obstacleAvoidance();
            break;

        case kGPSObstacleAvoidance:
            gpsObstacleAvoidance();
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

void SensorCallback::RoverStateClassifier()
{
    if (CurrState == kObstacleAvoidance || CurrState == kGPSObstacleAvoidance)
        return;

    if ((nav_mode == 0 && gps_goal_reached) || (nav_mode == 1 && aruco_goal_reached))
    {
        hardStop();
        disableAutonomous();

        CurrState = kManualState;

        return;
    }

    if (nav_mode == 0)
    {
        if (gps_goal_set && !gps_goal_reached)
        {
            if (CurrState != kGPSObstacleAvoidance)
                CurrState = kCoordinateFollowing;
        }

        return;
    }

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
    if (fix_->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        return;

    if (!std::isfinite(fix_->latitude) || !std::isfinite(fix_->longitude))
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    curr_location.latitude = fix_->latitude;
    curr_location.longitude = fix_->longitude;

    last_gps_time_ = this->get_clock()->now();

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[GPS] Current = %.7f, %.7f",
        curr_location.latitude, curr_location.longitude);
}

void SensorCallback::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
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
        gps_wall_following_active_ = false;

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
    gps_wall_following_active_ = false;

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

    RCLCPP_INFO(this->get_logger(), "[MODE] AUTONOMOUS MODE -> NAVIGATION SELECT");
}

void SensorCallback::navigationModeSelect()
{
    if (nav_select_done_)
        return;

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

        if (goal_location.latitude < -90.0 || goal_location.latitude > 90.0)
        {
            std::cout << "[GPS] ERROR: Invalid latitude\n";

            nav_mode = -1;
            nav_select_done_ = true;
            CurrState = kManualState;

            return;
        }

        if (goal_location.longitude < -180.0 || goal_location.longitude > 180.0)
        {
            std::cout << "[GPS] ERROR: Invalid longitude\n";

            nav_mode = -1;
            nav_select_done_ = true;
            CurrState = kManualState;

            return;
        }

        gps_goal_set = true;
        gps_goal_reached = false;
        gps_aligned_ = false;

        gps_wall_following_active_ = false;

        CurrState = kCoordinateFollowing;

        RCLCPP_INFO(
            get_logger(), "[NAV][GPS] Goal saved | lat=%.7f lon=%.7f",
            goal_location.latitude, goal_location.longitude);

        std::cout << "\n[GPS] Navigation selected\n";
        std::cout << "[GPS] Target latitude  : " << goal_location.latitude << "\n";
        std::cout << "[GPS] Target longitude : " << goal_location.longitude << "\n";
        std::cout << "[GPS] Starting coordinate following...\n";
        std::cout << "====================================\n";
    }
    else if (nav_mode == 1)
    {
        gps_wall_following_active_ = false;

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

        search_ref_set_ = false;
        search_aligned_ = false;

        CurrState = kSearchPattern;

        std::cout << "[CLI] ArUco navigation selected -> SEARCH\n";
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
    if (!gps_goal_set)
    {
        publishVel(geometry_msgs::msg::Twist());

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[GPS] No GPS goal set");

        return;
    }

    auto now = this->get_clock()->now();

    if ((now - last_gps_time_).seconds() > 1.5)
    {
        gps_aligned_ = false;

        publishVel(geometry_msgs::msg::Twist());

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "[GPS] GPS data stale - stopping rover");

        return;
    }

    double dist = haversine(curr_location, goal_location);

    constexpr double goal_threshold = 2.0;

    if (dist <= goal_threshold)
    {
        gps_goal_reached = true;
        gps_aligned_ = false;

        hardStop();

        RCLCPP_INFO(get_logger(), "[GPS] GOAL REACHED | distance=%.2f m", dist);

        return;
    }

    double target_bearing = gpsBearing(curr_location, goal_location);
    double current_heading = normalize360(bno_yaw);
    double heading_error_value = headingError(target_bearing, current_heading);

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    constexpr double heading_tolerance = 8.0;

    if (std::abs(heading_error_value) > heading_tolerance)
    {
        gps_aligned_ = false;

        cmd.linear.x = 0.0;

        constexpr double k_heading = 0.02;

        double angular = k_heading * heading_error_value;
        angular = std::clamp(angular, -1.0, 1.0);

        if (std::abs(angular) > 0.0 && std::abs(angular) < 0.93)
            angular = std::copysign(0.93, angular);

        cmd.angular.z = angular;

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "[GPS][ALIGN] dist=%.2f m | target=%.2f deg | imu=%.2f deg | error=%.2f deg | angular=%.2f",
            dist, target_bearing, current_heading, heading_error_value, cmd.angular.z);

        publishVel(cmd);

        return;
    }

    gps_aligned_ = true;

    double speed = std::clamp(0.25 + (0.10 * dist), 0.25, 1.0);

    constexpr double k_steering = 0.015;

    cmd.linear.x = speed;

    double steering = k_steering * heading_error_value;
    steering = std::clamp(steering, -1.0, 1.0);

    if (std::abs(steering) > 0.0 && std::abs(steering) < 0.93)
        steering = std::copysign(0.93, steering);

    cmd.angular.z = steering;

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 500,
        "[GPS][FOLLOW] dist=%.2f m | target=%.2f deg | imu=%.2f deg | error=%.2f deg | linear=%.2f | angular=%.2f",
        dist, target_bearing, current_heading, heading_error_value, cmd.linear.x, cmd.angular.z);

    publishVel(cmd);
}

void SensorCallback::obstacleAvoidance()
{
    if (obstacle_detect)
    {
        obstacle_clear_timing_ = false;
        avoiding_obstacle_ = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = -1.0;

        RCLCPP_WARN(
            get_logger(), "[AVOID] avoiding obstacle x=%.2f y=%.2f ang=%.2f",
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

void SensorCallback::gpsObstacleAvoidance()
{
    if (!gps_wall_following_active_)
    {
        gps_wall_following_active_ = true;

        RCLCPP_INFO(
            get_logger(),
            "[GPS][BUG] Entered GPS obstacle avoidance");
    }

    if (!gps_goal_set)
    {
        gps_wall_following_active_ = false;

        publishVel(geometry_msgs::msg::Twist());

        CurrState = kCoordinateFollowing;

        return;
    }

    auto now = this->get_clock()->now();

    if ((now - last_gps_time_).seconds() > 1.5)
    {
        gps_aligned_ = false;

        publishVel(geometry_msgs::msg::Twist());

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[GPS][BUG] GPS data stale - stopping rover");

        return;
    }

    double dist = haversine(
        curr_location,
        goal_location);

    constexpr double goal_threshold = 3.0;

    if (dist <= goal_threshold)
    {
        gps_goal_reached = true;
        gps_aligned_ = false;
        gps_wall_following_active_ = false;

        hardStop();

        CurrState = kManualState;

        RCLCPP_INFO(
            get_logger(),
            "[GPS] GOAL REACHED DURING OBSTACLE AVOIDANCE | distance=%.2f m",
            dist);

        return;
    }

    if (!last_lidar_scan_)
    {
        publishVel(geometry_msgs::msg::Twist());

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[GPS][BUG] No LiDAR data");

        return;
    }

    if (!obstacle_detect)
    {
        gps_wall_following_active_ = false;

        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;

        publishVel(stop);

        CurrState = kCoordinateFollowing;

        RCLCPP_INFO(
            get_logger(),
            "[GPS][BUG] Obstacle cleared -> GPS coordinate following");

        return;
    }

    const auto &scan = *last_lidar_scan_;

    constexpr double front_threshold = 0.80;

    constexpr double right_min_angle =
        -35.0 * M_PI / 180.0;

    constexpr double right_max_angle =
        -5.0 * M_PI / 180.0;

    constexpr double front_min_angle =
        -25.0 * M_PI / 180.0;

    constexpr double front_max_angle =
        25.0 * M_PI / 180.0;

    constexpr double max_valid_range = 3.0;

    const double desired_right_distance =
        gps_wall_target_distance_;

    double right_distance =
        max_valid_range;

    double front_distance =
        max_valid_range;

    bool right_found = false;
    bool front_found = false;

    double angle = scan.angle_min;

    for (size_t i = 0;
         i < scan.ranges.size();
         ++i,
         angle += scan.angle_increment)
    {
        const double r = scan.ranges[i];

        if (!std::isfinite(r))
            continue;

        if (r < scan.range_min ||
            r > scan.range_max)
            continue;

        if (angle >= front_min_angle &&
            angle <= front_max_angle)
        {
            front_distance =
                std::min(front_distance, r);

            front_found = true;
        }

        if (angle >= right_min_angle &&
            angle <= right_max_angle)
        {
            right_distance =
                std::min(right_distance, r);

            right_found = true;
        }
    }

    if (!right_found)
        right_distance = max_valid_range;

    if (front_found &&
        front_distance < front_threshold)
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.angular.z = 0.65;

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            300,
            "[GPS][BUG] FRONT BLOCKED | front=%.2f m | TURN LEFT",
            front_distance);

        publishVel(cmd);

        return;
    }

    if (right_distance >
        desired_right_distance + 0.15)
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.55;
        cmd.angular.z = -0.55;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "[GPS][BUG] RIGHT WALL FAR | right=%.2f m | TURN RIGHT",
            right_distance);

        publishVel(cmd);

        return;
    }

    if (right_distance <
        desired_right_distance - 0.15)
    {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.55;
        cmd.angular.z = 0.55;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "[GPS][BUG] RIGHT WALL CLOSE | right=%.2f m | TURN LEFT",
            right_distance);

        publishVel(cmd);

        return;
    }

    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = 0.55;
    cmd.angular.z = 0.0;

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[GPS][BUG] RIGHT WALL FOLLOW | dist=%.2f m | front=%.2f m | right=%.2f m",
        dist,
        front_distance,
        right_distance);

    publishVel(cmd);
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

            RCLCPP_INFO(get_logger(), "[SEARCH][SPOT] 8s turn done -> start pattern");
        }
        return;
    }

    if (!search_ref_set_)
    {
        FollowPattern = kMoveForward;
        search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
        search_ref_set_ = true;

        RCLCPP_INFO(get_logger(), "[SEARCH] Starting the search pattern, moving forward");

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
            search_end_time_ = now + rclcpp::Duration::from_seconds(7.0);
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

            search_end_time_ = now + rclcpp::Duration::from_seconds(3.5 + extra);
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
            search_end_time_ = now + rclcpp::Duration::from_seconds(search_forward_time_);
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
            search_end_time_ = now + rclcpp::Duration::from_seconds(3.5);
        }
        return;
    }
}

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

    if (!last_lidar_scan_)
    {
        obstacle_detect = false;
        obs_x = 0.0f;
        obs_y = 0.0f;
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

    const auto& scan = *last_lidar_scan_;

    float angle = scan.angle_min;

    for (size_t i = 0; i < scan.ranges.size(); ++i, angle += scan.angle_increment)
    {
        const float r = scan.ranges[i];

        if (r < scan.range_min || r > scan.range_max)
            continue;

        if (!std::isfinite(r))
            continue;

        const float px = r * std::cos(angle);
        const float py = r * std::sin(angle);

        if (aruco_detect)
        {
            const float dx = px - static_cast<float>(aruco_x);
            const float dy = py - static_cast<float>(aruco_y);

            const float distance_to_target = std::sqrt(dx * dx + dy * dy);

            if (distance_to_target < aruco_mask_radius)
                continue;
        }

        if (px < min_x || px > max_x)
            continue;

        if (std::abs(py) > half_w)
            continue;

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
    else
    {
        obs_x = 0.0f;
        obs_y = 0.0f;

        RCLCPP_INFO_THROTTLE(get_logger(), *clock, 1000, "[OBS] No obstacle");
    }
}

void SensorCallback::setGoalStatus()
{
    static int valid_count = 0;

    if (nav_mode != 1 || CurrState != kArucoFollowing || aruco_goal_reached || !aruco_detect)
        return;

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

    double angle = atan2(x, y) * 180.0 / M_PI;
    if (angle < 0)
        angle += 360.0;

    return angle;
}

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

double SensorCallback::headingError(double target, double current)
{
    double diff = target - current;

    if (diff > 180.0)
        diff -= 360.0;
    if (diff < -180.0)
        diff += 360.0;

    return diff;
}

double SensorCallback::normalize360(double angle)
{
    angle = fmod(angle, 360.0);
    if (angle < 0.0)
        angle += 360.0;
    return angle;
}

} // namespace planner
