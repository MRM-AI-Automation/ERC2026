#include "planner/lidar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace planner
{

SensorCallback::SensorCallback()
: Node("planner_node"),
  vel_pub(nullptr), imu_sub_(nullptr), gps_sub_(nullptr), aruco_sub_(nullptr),
  scan_sub_(nullptr), auto_sub_(nullptr), toggle_client_(nullptr), stack_timer_(nullptr),
  CurrState(kNavigationModeSelect), FollowPattern(kMoveForward), nav_mode(-1),
  target_aruco_id_(0), nav_select_done_(false), rover_state(false), last_rover_state(false),
  gps_goal_set(false), gps_goal_reached(false), gps_aligned_(false),
  curr_location{0.0, 0.0}, goal_location{0.0, 0.0}, imu_yaw(0.0),
  aruco_detect(false), aruco_goal_reached(false), aruco_x(0.0), aruco_y(0.0),
  obstacle_detect(false), obs_x(0.0), obs_y(0.0),
  last_scan_(nullptr), last_scan_time_(this->now()),
  search_ref_set_(false), spot_turn_back_(false), spot_done_(false), search_cycle_(0),
  search_end_time_(this->now()), search_forward_time_(4.0), search_skew(kNoSkew),
  avoiding_obstacle_(false), prev_state_(kNavigationModeSelect),
  prev_search_pattern_(kMoveForward),
  gps_avoiding_(false), gps_avoid_direction_(0),
  gps_avoid_moving_forward_(false)
{
    declare_parameter("imu_topic", "/imu_data");
    declare_parameter("gps_topic", "/gps");
    declare_parameter("aruco_topic", "/aruco_detected");
    declare_parameter("scan_topic", "/scan_front");
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("state_topic", "/autonomous_mode_state");
    declare_parameter("target_aruco_id", 1);

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto aruco_topic = get_parameter("aruco_topic").as_string();
    const auto scan_topic = get_parameter("scan_topic").as_string();
    const auto cmd_vel = get_parameter("cmd_vel_topic").as_string();

    target_aruco_id_ = get_parameter("target_aruco_id").as_int();

    vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);

    imu_sub_ = create_subscription<msgs::msg::ImuData>(
        imu_topic,
        10,
        std::bind(
            &SensorCallback::imuCallback,
            this,
            std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic,
        10,
        std::bind(
            &SensorCallback::gpsCallback,
            this,
            std::placeholders::_1));

    aruco_sub_ = create_subscription<msgs::msg::ArucoTag>(
        aruco_topic,
        10,
        std::bind(
            &SensorCallback::arucoCallback,
            this,
            std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic,
        rclcpp::SensorDataQoS(),
        std::bind(
            &SensorCallback::scanCallback,
            this,
            std::placeholders::_1));

    stack_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&SensorCallback::stackRun, this));

    toggle_client_ =
        create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

    last_gps_time_ = this->now();
    last_aruco_time_ = this->now();
    gps_last_check_time_ = this->now();
}


// ============================================================
// MAIN STATE MACHINE
// ============================================================

void SensorCallback::stackRun()
{
    const auto now = get_clock()->now();

    // ========================================================
    // GLOBAL LIDAR SAFETY
    // ========================================================

    constexpr double LIDAR_TIMEOUT = 0.3;

    double lidar_age = std::numeric_limits<double>::infinity();

    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (last_scan_)
        {
            lidar_age =
                (now - last_scan_time_).seconds();
        }
    }

    if (!last_scan_ ||
        last_scan_->ranges.empty() ||
        lidar_age > LIDAR_TIMEOUT)
    {
        hardStop();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SAFETY] LIDAR STALE/MISSING -> ROVER STOPPED | age=%.2f s",
            lidar_age);

        return;
    }


    // ========================================================
    // UPDATE OBSTACLE INFORMATION
    // ========================================================

    obstacleClassifier();


    // ========================================================
    // CHECK GPS GOAL FIRST
    //
    // This is deliberately BEFORE obstacle avoidance.
    // Therefore GPS goal always wins.
    // ========================================================

    if (nav_mode == 0 &&
        gps_goal_set &&
        !gps_goal_reached)
    {
        const double gps_dist =
            haversine(curr_location, goal_location);

        if (gps_dist <= kDistanceThreshold)
        {
            gps_goal_reached = true;
            gps_goal_set = false;

            gps_aligned_ = false;
            gps_waiting_ = false;

            gps_avoiding_ = false;
            gps_avoid_direction_ = 0;
            gps_avoid_moving_forward_ = false;

            // FIRST stop the rover
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[GPS] GOAL REACHED -> ROVER STOPPED | Distance: %.2f m",
                gps_dist);

            // =================================================
            // TERMINATE ROS2 NODE
            // =================================================

            RCLCPP_INFO(
                get_logger(),
                "[GPS] Shutting down planner node...");

            rclcpp::shutdown();

            return;
        }
    }


    // ========================================================
    // ARUCO FRESHNESS
    // ========================================================

    constexpr double ARUCO_TIMEOUT = 0.5;

    if (aruco_detect &&
        (now - last_aruco_time_).seconds() > ARUCO_TIMEOUT)
    {
        aruco_detect = false;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[ARUCO] Marker lost");
    }


    // ========================================================
    // ARUCO GOAL CHECK
    //
    // This is BEFORE obstacle avoidance.
    // Therefore ArUco goal always wins.
    // ========================================================

    if (nav_mode == 1 &&
        CurrState == kArucoFollowing &&
        aruco_detect &&
        aruco_x >= 0.0 &&
        aruco_x <= kDistanceThreshold)
    {
        if (!aruco_goal_reached)
        {
            aruco_goal_reached = true;

            // FIRST stop the rover
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO] GOAL REACHED -> ROVER STOPPED | Distance: %.2f m",
                aruco_x);

            // =================================================
            // TERMINATE ROS2 NODE
            // =================================================

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO] Shutting down planner node...");

            rclcpp::shutdown();

            return;
        }

        return;
    }


    // ========================================================
    // ARUCO GOAL REACHED SAFETY
    // ========================================================

    if (nav_mode == 1 && aruco_goal_reached)
    {
        hardStop();
        return;
    }


    // ========================================================
    // OBSTACLE AVOIDANCE CAN ONLY INTERRUPT FORWARD DRIVING
    // ========================================================

    const bool gps_driving_straight =
        (nav_mode == 0 &&
         CurrState == kCoordinateFollowing &&
         gps_aligned_ &&
         !gps_waiting_ &&
         gps_goal_set &&
         !gps_goal_reached);

    const bool aruco_driving_forward =
        (nav_mode == 1 &&
         CurrState == kArucoFollowing &&
         aruco_detect &&
         aruco_x > 0.0);

    const bool allowed_to_avoid =
        gps_driving_straight ||
        aruco_driving_forward;


    if (allowed_to_avoid)
    {
        // ====================================================
        // ARUCO OBSTACLE AVOIDANCE
        // ====================================================

        if (aruco_driving_forward &&
            obstacle_detect &&
            obs_x > 0.0 &&
            aruco_x > 0.0 &&
            static_cast<double>(obs_x) < aruco_x)
        {
            avoiding_obstacle_ = true;
            prev_state_ = CurrState;
            prev_search_pattern_ = FollowPattern;

            CurrState = kObstacleAvoidance;

            RCLCPP_WARN(
                get_logger(),
                "[ARUCO OA] Obstacle %.2f m is before target %.2f m "
                "-> entering avoidance",
                obs_x,
                aruco_x);

            obstacleAvoidance();
            return;
        }


        // ====================================================
        // GPS OBSTACLE AVOIDANCE
        // ====================================================

        if (gps_driving_straight &&
            obstacle_detect &&
            obs_x > 0.0)
        {
            const double goal_distance =
                haversine(curr_location, goal_location);

            const double obstacle_distance =
                static_cast<double>(obs_x);

            if (goal_distance > obstacle_distance)
            {
                gps_avoiding_ = true;
                gps_avoid_direction_ = 0;
                gps_avoid_moving_forward_ = false;

                gps_aligned_ = false;
                gps_waiting_ = false;

                CurrState = kGPSObstacleAvoidance;

                RCLCPP_WARN(
                    get_logger(),
                    "[GPS AVOID] Obstacle %.2f m ahead, goal %.2f m away "
                    "-> starting avoidance",
                    obstacle_distance,
                    goal_distance);

                gpsObstacleAvoidance();
                return;
            }
        }
    }


    // ========================================================
    // ACTIVE ARUCO OBSTACLE AVOIDANCE
    // ========================================================

    if (CurrState == kObstacleAvoidance)
    {
        obstacleAvoidance();
        return;
    }


    // ========================================================
    // ACTIVE GPS OBSTACLE AVOIDANCE
    // ========================================================

    if (CurrState == kGPSObstacleAvoidance)
    {
        gpsObstacleAvoidance();
        return;
    }


    // ========================================================
    // ARUCO STATE TRANSITIONS
    // ========================================================

    if (nav_mode == 1 &&
        CurrState != kObstacleAvoidance)
    {
        if (CurrState == kSearchPattern &&
            aruco_detect)
        {
            hardStop();

            CurrState = kArucoFollowing;

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO] Target detected -> switching to ArUco following");

            return;
        }


        if (CurrState == kArucoFollowing &&
            !aruco_detect)
        {
            hardStop();

            resetSearchPattern();

            CurrState = kSearchPattern;

            RCLCPP_WARN(
                get_logger(),
                "[ARUCO] Target lost -> returning to search pattern");

            return;
        }
    }


    // ========================================================
    // RUN CURRENT STATE
    // ========================================================

    switch (CurrState)
    {
        case kManualState:
            hardStop();
            break;

        case kNavigationModeSelect:
            navigationModeSelect();
            break;

        case kCoordinateFollowing:
            coordinateFollowing();
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

        case kObstacleAvoidance:
            obstacleAvoidance();
            break;

        default:
            hardStop();
            break;
    }
}


// ============================================================
// ROVER STATE CLASSIFIER
// ============================================================

void SensorCallback::RoverStateClassifier()
{
    return;
}


// ============================================================
// IMU CALLBACK
// ============================================================

void SensorCallback::imuCallback(
    const msgs::msg::ImuData::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    imu_yaw = msg->orientation.z;
}


// ============================================================
// GPS CALLBACK
// ============================================================

void SensorCallback::gpsCallback(
    const sensor_msgs::msg::NavSatFix::SharedPtr fix_)
{
    if (fix_->status.status <
        sensor_msgs::msg::NavSatStatus::STATUS_FIX)
        return;

    if (!std::isfinite(fix_->latitude) ||
        !std::isfinite(fix_->longitude))
        return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    curr_location.latitude = fix_->latitude;
    curr_location.longitude = fix_->longitude;

    last_gps_time_ =
        this->get_clock()->now();
}


// ============================================================
// LIDAR CALLBACK
// ============================================================

void SensorCallback::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    last_scan_ = scan_msg;
    last_scan_time_ =
        this->get_clock()->now();
}


// ============================================================
// ARUCO CALLBACK
// ============================================================

void SensorCallback::arucoCallback(
    const msgs::msg::ArucoTag::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (msg->is_detected &&
        msg->id == target_aruco_id_)
    {
        aruco_detect = true;

        aruco_x =
            static_cast<double>(msg->aruco_x);

        aruco_y =
            static_cast<double>(msg->aruco_y);

        last_aruco_time_ =
            this->get_clock()->now();
    }
}


// ============================================================
// STATE CALLBACK
// ============================================================

void SensorCallback::stateCallback(
    const std_msgs::msg::Bool::SharedPtr state)
{
    (void)state;
    return;
}


// ============================================================
// NAVIGATION MODE SELECTION
// ============================================================

void SensorCallback::navigationModeSelect()
{
    if (nav_select_done_)
        return;

    publishVel(geometry_msgs::msg::Twist());

    std::cout
        << "\n=================================\n"
        << "      NAVIGATION MODE SELECT\n"
        << "=================================\n"
        << "0 -> GPS Navigation\n"
        << "1 -> ArUco Navigation\n"
        << "Select mode: ";

    std::cin >> nav_mode;


    // ========================================================
    // GPS
    // ========================================================

    if (nav_mode == 0)
    {
        std::cout
            << "\n========== GPS NAVIGATION ==========\n"
            << "Current GPS position:\n"
            << "Latitude  : "
            << curr_location.latitude
            << "\n"
            << "Longitude : "
            << curr_location.longitude
            << "\n\n";

        std::cout << "Enter goal latitude  : ";
        std::cin >> goal_location.latitude;

        std::cout << "Enter goal longitude : ";
        std::cin >> goal_location.longitude;


        if (goal_location.latitude < -90.0 ||
            goal_location.latitude > 90.0 ||
            goal_location.longitude < -180.0 ||
            goal_location.longitude > 180.0)
        {
            std::cout
                << "[GPS] ERROR: Invalid coordinates\n";

            nav_mode = -1;
            nav_select_done_ = true;

            CurrState = kNavigationModeSelect;

            return;
        }


        gps_goal_set = true;
        gps_goal_reached = false;

        gps_aligned_ = false;
        gps_waiting_ = false;

        gps_avoiding_ = false;
        gps_avoid_direction_ = 0;
        gps_avoid_moving_forward_ = false;

        gps_last_check_time_ =
            this->now();

        CurrState =
            kCoordinateFollowing;

        RCLCPP_INFO(
            get_logger(),
            "[NAV] GPS NAVIGATION SELECTED");

        std::cout
            << "[GPS] Navigation selected, "
            << "starting coordinate following...\n";
    }


    // ========================================================
    // ARUCO
    // ========================================================

    else if (nav_mode == 1)
    {
        std::cout
            << "Enter target ArUco ID : ";

        std::cin >>
            target_aruco_id_;

        std::cout
            << "Search skew (-1 = LEFT, "
            << "0 = NONE, 1 = RIGHT): ";

        int skew;
        std::cin >> skew;

        setSearchSkew(skew);

        std::cout
            << "Forward search time (sec): ";

        std::cin >>
            search_forward_time_;

        std::cout
            << "Spot turn back before search? (0/1): ";

        int back;
        std::cin >> back;

        search_end_time_ =
            this->get_clock()->now() +
            rclcpp::Duration::from_seconds(6.5);

        spot_turn_back_ =
            (back == 1);

        aruco_detect = false;
        aruco_goal_reached = false;
        obstacle_detect = false;

        resetSearchPattern();

        CurrState =
            kSearchPattern;

        RCLCPP_INFO(
            get_logger(),
            "[NAV] ARUCO NAVIGATION SELECTED");
    }

    else
    {
        std::cout
            << "[CLI] Invalid navigation selection\n";

        nav_mode = -1;

        CurrState =
            kNavigationModeSelect;
    }

    nav_select_done_ = true;
}


// ============================================================
// GPS COORDINATE FOLLOWING
// ============================================================

void SensorCallback::coordinateFollowing()
{
    geometry_msgs::msg::Twist stop_cmd;

    // No active GPS goal
    if (!gps_goal_set)
    {
        hardStop();
        return;
    }

    const auto now =
        get_clock()->now();


    // ========================================================
    // GPS FRESHNESS
    // ========================================================

    if ((now - last_gps_time_).seconds() > 1.5)
    {
        gps_aligned_ = false;
        gps_waiting_ = false;

        hardStop();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "[GPS] GPS data stale -> rover stopped");

        return;
    }


    const double dist =
        haversine(
            curr_location,
            goal_location);


    // ========================================================
    // GPS GOAL REACHED
    //
    // This is also checked here as a backup.
    // ========================================================

    if (dist <= kDistanceThreshold)
    {
        gps_goal_reached = true;
        gps_goal_set = false;

        gps_aligned_ = false;
        gps_waiting_ = false;

        gps_avoiding_ = false;
        gps_avoid_direction_ = 0;
        gps_avoid_moving_forward_ = false;

        hardStop();

        RCLCPP_INFO(
            get_logger(),
            "[GPS] GOAL REACHED -> ROVER STOPPED | Distance: %.2f m",
            dist);

        RCLCPP_INFO(
            get_logger(),
            "[GPS] Shutting down planner node...");

        rclcpp::shutdown();

        return;
    }


    gps_goal_reached = false;


    // ========================================================
    // TARGET BEARING
    // ========================================================

    const double target_bearing =
        gpsBearing(
            curr_location,
            goal_location);

    const double error =
        headingError(
            target_bearing,
            imu_yaw);

    const double shortest_error =
        std::min(
            error,
            360.0 - error);


    constexpr double HEADING_TOLERANCE = 10.0;
    constexpr double WAIT_TIME = 2.0;
    constexpr double CHECK_TIME = 2.0;
    constexpr double TURN_SPEED = 1.8;


    geometry_msgs::msg::Twist cmd;


    // ========================================================
    // STEP 1: ALIGN
    // ========================================================

    if (!gps_aligned_)
    {
        if (shortest_error <=
            HEADING_TOLERANCE)
        {
            gps_aligned_ = true;
            gps_waiting_ = true;

            gps_wait_end_ =
                now +
                rclcpp::Duration::from_seconds(
                    WAIT_TIME);

            hardStop();

            return;
        }


        cmd.linear.x = 0.0;


        if (error <= 180.0)
            cmd.angular.z = -TURN_SPEED;
        else
            cmd.angular.z = TURN_SPEED;


        publishVel(cmd);

        return;
    }


    // ========================================================
    // STEP 2: VERIFY ALIGNMENT
    // ========================================================

    if (gps_waiting_)
    {
        hardStop();

        if (now < gps_wait_end_)
            return;

        if (shortest_error >
            HEADING_TOLERANCE)
        {
            gps_aligned_ = false;
            gps_waiting_ = false;

            return;
        }

        gps_waiting_ = false;
        gps_last_check_time_ = now;

        RCLCPP_INFO(
            get_logger(),
            "[GPS] Aligned -> driving");

        return;
    }


    // ========================================================
    // STEP 3: DRIVE
    // ========================================================

    if ((now - gps_last_check_time_).seconds()
        >= CHECK_TIME)
    {
        gps_last_check_time_ = now;

        if (shortest_error >
            HEADING_TOLERANCE)
        {
            gps_aligned_ = false;
            gps_waiting_ = false;

            hardStop();

            RCLCPP_WARN(
                get_logger(),
                "[GPS] Heading lost -> realigning");

            return;
        }
    }


    const bool goal_before_obstacle =
        obstacle_detect &&
        dist < static_cast<double>(obs_x);


    if (goal_before_obstacle)
        cmd.linear.x = 0.8;
    else
        cmd.linear.x = 1.5;

    cmd.angular.z = 0.0;

    publishVel(cmd);
}


// ============================================================
// ARUCO OBSTACLE AVOIDANCE
// ============================================================

void SensorCallback::obstacleAvoidance()
{
    // ========================================================
    // ARUCO PRIORITY
    // ========================================================

    if (nav_mode == 1 &&
        prev_state_ == kArucoFollowing &&
        aruco_detect &&
        aruco_x > 0.0)
    {
        const bool aruco_is_closer =
            !obstacle_detect ||
            obs_x <= 0.0 ||
            aruco_x <=
                static_cast<double>(obs_x);

        if (aruco_is_closer)
        {
            avoiding_obstacle_ = false;

            CurrState =
                kArucoFollowing;

            arucoFollowing();

            return;
        }
    }


    // ========================================================
    // OBSTACLE STILL PRESENT
    // ========================================================

    if (obstacle_detect)
    {
        avoiding_obstacle_ = true;

        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;


        if (obs_side_ &&
            std::string(obs_side_) == "left")
        {
            cmd.angular.z = -1.0;
        }
        else if (obs_side_ &&
                 std::string(obs_side_) == "right")
        {
            cmd.angular.z = 1.0;
        }
        else
        {
            cmd.angular.z = 1.0;
        }

        publishVel(cmd);

        return;
    }


    // ========================================================
    // OBSTACLE CLEARED
    // ========================================================

    hardStop();

    avoiding_obstacle_ = false;


    if (nav_mode == 1 &&
        prev_state_ == kArucoFollowing)
    {
        if (aruco_detect)
        {
            CurrState =
                kArucoFollowing;

            return;
        }

        resetSearchPattern();

        FollowPattern =
            kMoveForward;

        CurrState =
            kSearchPattern;

        return;
    }


    if (nav_mode == 1 &&
        prev_state_ == kSearchPattern)
    {
        FollowPattern =
            prev_search_pattern_;

        CurrState =
            kSearchPattern;

        return;
    }


    hardStop();
}


// ============================================================
// GPS OBSTACLE AVOIDANCE
// ============================================================

void SensorCallback::gpsObstacleAvoidance()
{
    // ========================================================
    // GOAL CHECK DURING GPS AVOIDANCE
    // ========================================================

    if (nav_mode == 0 &&
        gps_goal_set &&
        !gps_goal_reached)
    {
        const double dist =
            haversine(
                curr_location,
                goal_location);

        if (dist <=
            kDistanceThreshold)
        {
            gps_goal_reached = true;
            gps_goal_set = false;

            gps_avoiding_ = false;
            gps_avoid_direction_ = 0;
            gps_avoid_moving_forward_ = false;

            gps_aligned_ = false;
            gps_waiting_ = false;

            // STOP ROVER FIRST
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[GPS] GOAL REACHED DURING AVOIDANCE "
                "-> ROVER STOPPED | Distance: %.2f m",
                dist);

            // TERMINATE NODE
            RCLCPP_INFO(
                get_logger(),
                "[GPS] Shutting down planner node...");

            rclcpp::shutdown();

            return;
        }
    }


    geometry_msgs::msg::Twist cmd;

    const auto now =
        get_clock()->now();

    constexpr double TURN_SPEED = 1.8;
    constexpr double FORWARD_SPEED = 1.1;
    constexpr double FORWARD_TIME = 1.0;


    // ========================================================
    // STEP 1: ROTATE UNTIL FRONT IS CLEAR
    // ========================================================

    if (!gps_avoid_moving_forward_)
    {
        if (obstacle_detect &&
            obs_x > 0.0)
        {
            if (gps_avoid_direction_ == 0)
            {
                if (obs_side_ &&
                    std::string(obs_side_) == "left")
                {
                    gps_avoid_direction_ = -1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] "
                        "Obstacle LEFT -> turning RIGHT");
                }
                else if (obs_side_ &&
                         std::string(obs_side_) == "right")
                {
                    gps_avoid_direction_ = 1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] "
                        "Obstacle RIGHT -> turning LEFT");
                }
                else
                {
                    gps_avoid_direction_ = 1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] "
                        "Obstacle CENTER/UNKNOWN -> turning LEFT");
                }
            }


            // NO FORWARD MOVEMENT
            cmd.linear.x = 0.0;

            cmd.angular.z =
                gps_avoid_direction_ *
                TURN_SPEED;

            publishVel(cmd);

            return;
        }


        // ====================================================
        // FRONT CLEAR -> MOVE FORWARD
        // ====================================================

        gps_avoid_moving_forward_ = true;

        gps_avoid_forward_end_ =
            now +
            rclcpp::Duration::from_seconds(
                FORWARD_TIME);

        cmd.linear.x =
            FORWARD_SPEED;

        cmd.angular.z = 0.0;

        publishVel(cmd);

        RCLCPP_INFO(
            get_logger(),
            "[GPS AVOID] Front clear -> "
            "moving forward for %.1f seconds",
            FORWARD_TIME);

        return;
    }


    // ========================================================
    // STEP 2: FORWARD
    //
    // FRONT MUST REMAIN CLEAR
    // ========================================================

    if (now <
        gps_avoid_forward_end_)
    {
        if (obstacle_detect &&
            obs_x > 0.0)
        {
            gps_avoid_moving_forward_ =
                false;

            gps_avoid_direction_ = 0;

            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;

            if (obs_side_ &&
                std::string(obs_side_) == "left")
            {
                gps_avoid_direction_ = -1;
            }
            else
            {
                gps_avoid_direction_ = 1;
            }

            cmd.angular.z =
                gps_avoid_direction_ *
                TURN_SPEED;

            publishVel(cmd);

            RCLCPP_WARN(
                get_logger(),
                "[GPS AVOID] Obstacle appeared "
                "during forward motion (%.2f m) "
                "-> STOP + ROTATE",
                obs_x);

            return;
        }


        // FRONT STILL CLEAR
        cmd.linear.x =
            FORWARD_SPEED;

        cmd.angular.z = 0.0;

        publishVel(cmd);

        return;
    }


    // ========================================================
    // STEP 3: AVOIDANCE COMPLETE
    // ========================================================

    hardStop();

    gps_avoiding_ = false;
    gps_avoid_direction_ = 0;
    gps_avoid_moving_forward_ = false;

    gps_aligned_ = false;
    gps_waiting_ = false;

    CurrState =
        kCoordinateFollowing;

    RCLCPP_INFO(
        get_logger(),
        "[GPS AVOID] Forward movement complete "
        "-> returning to GPS alignment");
}


// ============================================================
// ARUCO FOLLOWING
// ============================================================

void SensorCallback::arucoFollowing()
{
    // ========================================================
    // GOAL CHECK
    // ========================================================

    if (aruco_goal_reached)
    {
        hardStop();
        return;
    }


    if (!aruco_detect)
    {
        hardStop();
        return;
    }


    // ========================================================
    // EXTRA DIRECT GOAL CHECK
    // ========================================================

    if (aruco_x >= 0.0 &&
        aruco_x <= kDistanceThreshold)
    {
        aruco_goal_reached = true;

        hardStop();

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO] GOAL REACHED -> "
            "ROVER STOPPED | Distance: %.2f m",
            aruco_x);

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO] Shutting down planner node...");

        rclcpp::shutdown();

        return;
    }


    geometry_msgs::msg::Twist cmd;

    constexpr double ANGULAR_GAIN = 1.2;
    constexpr double Y_DEADBAND = 0.05;
    constexpr double MAX_ANGULAR = 2.5;

    constexpr double FOLLOW_SPEED = 0.75;
    constexpr double SPEED_SCALING_DISTANCE = 4.0;
    constexpr double MIN_FOLLOW_SPEED = 0.50;


    // ========================================================
    // LATERAL ALIGNMENT
    // ========================================================

    double lateral_error =
        aruco_y;

    if (std::abs(lateral_error) <
        Y_DEADBAND)
    {
        lateral_error = 0.0;
    }


    cmd.angular.z =
        std::clamp(
            -ANGULAR_GAIN * lateral_error,
            -MAX_ANGULAR,
            MAX_ANGULAR);


    // ========================================================
    // GOAL BEFORE OBSTACLE
    // ========================================================

    const bool goal_before_obstacle =
        obstacle_detect &&
        obs_x > 0.0 &&
        aruco_x > 0.0 &&
        aruco_x <=
            static_cast<double>(obs_x);


    // ========================================================
    // SPEED
    // ========================================================

    if (goal_before_obstacle)
    {
        cmd.linear.x = 0.6;
    }
    else if (aruco_x >=
             SPEED_SCALING_DISTANCE)
    {
        cmd.linear.x =
            std::min(
                FOLLOW_SPEED,
                kMaxLinearVel);
    }
    else
    {
        double distance_scale =
            std::clamp(
                aruco_x /
                    SPEED_SCALING_DISTANCE,
                0.0,
                1.0);

        distance_scale =
            distance_scale *
            distance_scale;

        const double scaled_speed =
            MIN_FOLLOW_SPEED +
            (FOLLOW_SPEED -
             MIN_FOLLOW_SPEED) *
                distance_scale;

        cmd.linear.x =
            std::min(
                scaled_speed,
                kMaxLinearVel);
    }


    publishVel(cmd);
}


// ============================================================
// SEARCH PATTERN
// ============================================================

void SensorCallback::callSearchPattern()
{
    constexpr double LINEAR_SPEED = 0.75;
    constexpr double ANGULAR_SPEED = 1.0;

    geometry_msgs::msg::Twist cmd;

    const auto now =
        get_clock()->now();


    // ========================================================
    // OPTIONAL INITIAL SPOT TURN
    // ========================================================

    if (spot_turn_back_ &&
        !spot_done_)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z =
            ANGULAR_SPEED;

        publishVel(cmd);

        if (now >= search_end_time_)
        {
            spot_done_ = true;
            search_ref_set_ = false;

            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][SPOT] "
                "Turn done -> starting search pattern");
        }

        return;
    }


    // ========================================================
    // INITIALIZE
    // ========================================================

    if (!search_ref_set_)
    {
        FollowPattern =
            kMoveForward;

        search_end_time_ =
            now +
            rclcpp::Duration::from_seconds(
                search_forward_time_);

        search_ref_set_ = true;

        cmd.linear.x =
            LINEAR_SPEED;

        cmd.angular.z = 0.0;

        publishVel(cmd);

        return;
    }


    // ========================================================
    // MOVE FORWARD
    // ========================================================

    if (FollowPattern ==
        kMoveForward)
    {
        cmd.linear.x =
            LINEAR_SPEED;

        cmd.angular.z = 0.0;

        publishVel(cmd);


        if (now >=
            search_end_time_)
        {
            hardStop();

            search_cycle_++;

            FollowPattern =
                kTurnA;

            search_end_time_ =
                now +
                rclcpp::Duration::from_seconds(
                    3.5);
        }

        return;
    }


    // ========================================================
    // TURN A
    // ========================================================

    if (FollowPattern ==
        kTurnA)
    {
        const bool right_skew =
            (search_skew ==
             kRightSkew);

        cmd.linear.x = 0.0;

        cmd.angular.z =
            right_skew
                ? -ANGULAR_SPEED
                : ANGULAR_SPEED;

        publishVel(cmd);


        if (now >=
            search_end_time_)
        {
            hardStop();

            FollowPattern =
                kTurnB;

            search_end_time_ =
                now +
                rclcpp::Duration::from_seconds(
                    7.0);
        }

        return;
    }


    // ========================================================
    // TURN B
    // ========================================================

    if (FollowPattern ==
        kTurnB)
    {
        const bool right_skew =
            (search_skew ==
             kRightSkew);

        cmd.linear.x = 0.0;

        cmd.angular.z =
            right_skew
                ? ANGULAR_SPEED
                : -ANGULAR_SPEED;

        publishVel(cmd);


        if (now >=
            search_end_time_)
        {
            hardStop();

            FollowPattern =
                kTurnC;

            const double extra =
                (search_skew != kNoSkew)
                    ? static_cast<double>(
                        search_cycle_)
                    : 0.0;

            search_end_time_ =
                now +
                rclcpp::Duration::from_seconds(
                    3.5 + extra);
        }

        return;
    }


    // ========================================================
    // TURN C
    // ========================================================

    if (FollowPattern ==
        kTurnC)
    {
        const bool right_skew =
            (search_skew ==
             kRightSkew);

        cmd.linear.x = 0.0;

        cmd.angular.z =
            right_skew
                ? -ANGULAR_SPEED
                : ANGULAR_SPEED;

        publishVel(cmd);


        if (now >=
            search_end_time_)
        {
            hardStop();

            FollowPattern =
                kMoveForward;

            search_end_time_ =
                now +
                rclcpp::Duration::from_seconds(
                    search_forward_time_);
        }

        return;
    }


    // ========================================================
    // SAFETY FALLBACK
    // ========================================================

    hardStop();
}


// ============================================================
// PUBLISH VELOCITY
// ============================================================

void SensorCallback::publishVel(
    const geometry_msgs::msg::Twist &msg)
{
    geometry_msgs::msg::Twist cmd =
        msg;

    cmd.linear.x =
        std::clamp(
            cmd.linear.x,
            0.0,
            kMaxLinearVel);

    if (cmd.linear.x > 0.0 &&
        cmd.linear.x < kMinLinearVel)
    {
        cmd.linear.x =
            kMinLinearVel;
    }


    if (std::abs(cmd.angular.z) < 1e-3)
    {
        cmd.angular.z = 0.0;
    }
    else
    {
        cmd.angular.z =
            std::clamp(
                cmd.angular.z,
                -kMaxAngularVel,
                kMaxAngularVel);

        if (std::abs(cmd.angular.z) <
            kMinAngularVel)
        {
            cmd.angular.z =
                std::copysign(
                    kMinAngularVel,
                    cmd.angular.z);
        }
    }

    vel_pub->publish(cmd);
}


// ============================================================
// HARD STOP
// ============================================================

void SensorCallback::hardStop()
{
    geometry_msgs::msg::Twist stop;

    stop.linear.x = 0.0;
    stop.linear.y = 0.0;
    stop.linear.z = 0.0;

    stop.angular.x = 0.0;
    stop.angular.y = 0.0;
    stop.angular.z = 0.0;

    if (vel_pub)
    {
        vel_pub->publish(stop);
    }
}


// ============================================================
// OBSTACLE CLASSIFIER
// ============================================================

void SensorCallback::obstacleClassifier()
{
    std::lock_guard<std::mutex> lock(
        state_mutex_);


    if (!last_scan_ ||
        last_scan_->ranges.empty())
    {
        obstacle_detect = false;
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "none";

        return;
    }


    constexpr double SCAN_TIMEOUT = 0.5;

    const double scan_age =
        (this->get_clock()->now() -
         last_scan_time_).seconds();


    if (scan_age > SCAN_TIMEOUT)
    {
        obstacle_detect = false;
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "none";

        hardStop();

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SAFETY] /scan_front stale "
            "(%.2f s old) -> ROVER STOPPED",
            scan_age);

        return;
    }


    bool found = false;

    float best_forward =
        std::numeric_limits<float>::max();

    float nearest_forward = 0.0f;
    float nearest_lateral = 0.0f;


    constexpr float HALF_WIDTH = 0.6f;

    constexpr float MIN_FORWARD = 0.30f;
    constexpr float MAX_FORWARD = 1.50f;

    constexpr float ARUCO_MASK_RADIUS = 0.40f;


    const auto &scan =
        *last_scan_;

    const size_t num_ranges =
        scan.ranges.size();


    for (size_t i = 0;
         i < num_ranges;
         ++i)
    {
        const float range =
            scan.ranges[i];


        if (!std::isfinite(range) ||
            range < scan.range_min ||
            range > scan.range_max)
        {
            continue;
        }


        const float angle =
            scan.angle_min +
            static_cast<float>(i) *
                scan.angle_increment;


        const float forward =
            range * std::cos(angle);

        const float lateral =
            range * std::sin(angle);


        if (forward < MIN_FORWARD ||
            forward > MAX_FORWARD ||
            std::abs(lateral) >
                HALF_WIDTH)
        {
            continue;
        }


        // Ignore ArUco itself
        if (aruco_detect)
        {
            const float dx =
                forward -
                static_cast<float>(
                    aruco_x);

            const float dy =
                lateral -
                static_cast<float>(
                    aruco_y);


            if ((dx * dx) +
                    (dy * dy) <
                (ARUCO_MASK_RADIUS *
                 ARUCO_MASK_RADIUS))
            {
                continue;
            }
        }


        if (forward < best_forward)
        {
            best_forward = forward;

            nearest_forward =
                forward;

            nearest_lateral =
                lateral;

            found = true;
        }
    }


    obstacle_detect = found;


    if (!found)
    {
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "none";

        return;
    }


    obs_x =
        nearest_forward;

    obs_y =
        nearest_lateral;


    if (obs_y >= 0.0)
        obs_side_ = "left";
    else
        obs_side_ = "right";


    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[OBSTACLE] Detected | "
        "Forward: %.2f m | "
        "Lateral: %.2f m | "
        "Side: %s",
        obs_x,
        obs_y,
        obs_side_ ?
            obs_side_ :
            "unknown");
}


// ============================================================
// ARUCO GOAL STATUS
// ============================================================

void SensorCallback::setGoalStatus()
{
    if (nav_mode != 1 ||
        CurrState != kArucoFollowing ||
        aruco_goal_reached ||
        !aruco_detect)
    {
        return;
    }


    if (aruco_x < 0.0)
        return;


    if (aruco_x <=
        kDistanceThreshold)
    {
        aruco_goal_reached = true;

        hardStop();

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO] GOAL REACHED -> "
            "ROVER STOPPED | Distance: %.2f m",
            aruco_x);

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO] Shutting down planner node...");

        rclcpp::shutdown();
    }
}


// ============================================================
// SEARCH SKEW
// ============================================================

void SensorCallback::setSearchSkew(
    int skew)
{
    if (skew == kLeftSkew)
        search_skew = kLeftSkew;
    else if (skew == kRightSkew)
        search_skew = kRightSkew;
    else
        search_skew = kNoSkew;
}


// ============================================================
// RESET SEARCH
// ============================================================

void SensorCallback::resetSearchPattern()
{
    FollowPattern =
        kMoveForward;

    search_ref_set_ =
        false;

    spot_done_ =
        false;

    search_cycle_ =
        0;
}


// ============================================================
// HAVERSINE
// ============================================================

double SensorCallback::haversine(
    Coordinates curr,
    Coordinates dest)
{
    double lat1 =
        curr.latitude *
        M_PI / 180.0;

    double lat2 =
        dest.latitude *
        M_PI / 180.0;

    double dLat =
        lat2 - lat1;

    double dLon =
        (dest.longitude -
         curr.longitude) *
        M_PI / 180.0;


    double h =
        sin(dLat * 0.5) *
        sin(dLat * 0.5) +
        cos(lat1) *
        cos(lat2) *
        sin(dLon * 0.5) *
        sin(dLon * 0.5);


    return 2.0 *
           6371000.0 *
           asin(sqrt(h));
}


// ============================================================
// GPS BEARING
// ============================================================

double SensorCallback::gpsBearing(
    Coordinates curr,
    Coordinates dest)
{
    double lat1 =
        curr.latitude *
        M_PI / 180.0;

    double lon1 =
        curr.longitude *
        M_PI / 180.0;

    double lat2 =
        dest.latitude *
        M_PI / 180.0;

    double lon2 =
        dest.longitude *
        M_PI / 180.0;


    double dLon =
        lon2 - lon1;


    double x =
        sin(dLon) *
        cos(lat2);

    double y =
        cos(lat1) *
        sin(lat2) -
        sin(lat1) *
        cos(lat2) *
        cos(dLon);


    double bearing =
        atan2(x, y) *
        180.0 / M_PI;


    if (bearing < 0.0)
        bearing += 360.0;


    return bearing;
}


// ============================================================
// HEADING ERROR
// ============================================================

double SensorCallback::headingError(
    double target,
    double current)
{
    double error =
        std::fmod(
            target -
            current +
            360.0,
            360.0);


    if (error < 0.0)
        error += 360.0;


    return error;
}


} // namespace planner
