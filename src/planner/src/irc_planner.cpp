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
  pointcloud_sub_(nullptr), auto_sub_(nullptr), toggle_client_(nullptr), stack_timer_(nullptr),
  // MANUAL MODE DISABLED: start directly in navigation selection.
  CurrState(kNavigationModeSelect), FollowPattern(kMoveForward), nav_mode(-1),
  target_aruco_id_(0), nav_select_done_(false), rover_state(false), last_rover_state(false),
  gps_goal_set(false), gps_goal_reached(false), gps_aligned_(false),
  curr_location{0.0, 0.0}, goal_location{0.0, 0.0}, imu_yaw(0.0),
  aruco_detect(false), aruco_goal_reached(false), aruco_x(0.0), aruco_y(0.0),
  obstacle_detect(false), obs_x(0.0), obs_y(0.0),
  last_point_cloud_(nullptr), last_pointcloud_time_(this->now()),
  search_ref_set_(false), spot_turn_back_(false), spot_done_(false), search_cycle_(0),
  search_end_time_(this->now()), search_forward_time_(4.0), search_skew(kNoSkew),
  avoiding_obstacle_(false), prev_state_(kNavigationModeSelect), prev_search_pattern_(kMoveForward),
  gps_avoiding_(false), gps_avoid_direction_(0), gps_avoid_moving_forward_(false)
{
    declare_parameter("imu_topic", "/imu_data");
    declare_parameter("gps_topic", "/gps");
    declare_parameter("aruco_topic", "/aruco_detected");
    declare_parameter("pointcloud_topic", "/local_grid_obstacle");
    declare_parameter("cmd_vel_topic", "/cmd_vel");
    declare_parameter("state_topic", "/autonomous_mode_state");
    declare_parameter("target_aruco_id", 1);

    const auto imu_topic = get_parameter("imu_topic").as_string();
    const auto gps_topic = get_parameter("gps_topic").as_string();
    const auto aruco_topic = get_parameter("aruco_topic").as_string();
    const auto pointcloud_topic = get_parameter("pointcloud_topic").as_string();
    const auto cmd_vel = get_parameter("cmd_vel_topic").as_string();
    // state_topic intentionally unused: manual/autonomous switching is disabled.

    target_aruco_id_ = get_parameter("target_aruco_id").as_int();

    vel_pub = create_publisher<geometry_msgs::msg::Twist>(cmd_vel, 10);
    imu_sub_ = create_subscription<msgs::msg::ImuData>(imu_topic,10,std::bind(&SensorCallback::imuCallback, this, std::placeholders::_1));
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, 10, std::bind(&SensorCallback::gpsCallback, this, std::placeholders::_1));
    aruco_sub_ = create_subscription<msgs::msg::ArucoTag>(aruco_topic, 10, std::bind(&SensorCallback::arucoCallback, this, std::placeholders::_1));
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, rclcpp::SensorDataQoS(),std::bind(&SensorCallback::pointCloudCallback, this, std::placeholders::_1));
    // MANUAL MODE DISABLED:
    // auto_sub_ = create_subscription<std_msgs::msg::Bool>(
    //     state_topic, 10,
    //     std::bind(&SensorCallback::stateCallback, this, std::placeholders::_1));

    stack_timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&SensorCallback::stackRun, this));
    // MANUAL MODE DISABLED:
    // toggle_client_ = create_client<std_srvs::srv::Trigger>("/toggle_autonomous");

    last_gps_time_ = this->now();
    last_aruco_time_ = this->now();
    gps_last_check_time_ = this->now();
}

void SensorCallback::stackRun()
{
    // =====================================================
    // MANUAL MODE DISABLED
    // =====================================================
    // Autonomous execution is no longer gated by rover_state.
    // The node runs autonomous navigation continuously.
    //
    const auto now = get_clock()->now();

    // =====================================================
    // GPS GOAL CHECK HAS ABSOLUTE PRIORITY
    //
    // This must happen BEFORE obstacle avoidance and BEFORE
    // the state machine. Otherwise GPS obstacle avoidance
    // can continue moving after the rover reaches the goal.
    // =====================================================
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

            // Cancel every GPS movement state immediately
            gps_aligned_ = false;
            gps_waiting_ = false;
            gps_avoiding_ = false;
            gps_avoid_moving_forward_ = false;
            gps_avoid_direction_ = 0;

            // STOP BEFORE changing modes
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[GPS] GOAL REACHED -> STOPPED | Distance: %.2f m",
                gps_dist);

            // MANUAL MODE DISABLED:
            // Do not switch to manual or toggle the external mode.
            // gps_goal_reached + gps_goal_set=false keep navigation stopped.
            return;
        }
    }

    // =====================================================
    // ARUCO FRESHNESS
    // =====================================================
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

    // =====================================================
    // UPDATE OBSTACLE INFORMATION
    // =====================================================
    obstacleClassifier();

    // =====================================================
    // ARUCO STATE TRANSITIONS
    // =====================================================
    if (nav_mode == 1 && CurrState != kObstacleAvoidance)
    {
        if (CurrState == kSearchPattern && aruco_detect)
        {
            hardStop();
            CurrState = kArucoFollowing;

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO] Target detected -> switching to ArUco following");

            return;
        }

        if (CurrState == kArucoFollowing && !aruco_detect)
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

    // =====================================================
    // ARUCO GOAL CHECK HAS ABSOLUTE PRIORITY
    // =====================================================
    if (nav_mode == 1 &&
        CurrState == kArucoFollowing &&
        aruco_detect &&
        aruco_x >= 0.0 &&
        aruco_x <= kDistanceThreshold)
    {
        if (!aruco_goal_reached)
        {
            aruco_goal_reached = true;
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO] GOAL REACHED -> STOPPED | Distance: %.2f m",
                aruco_x);
        }

        // MANUAL MODE DISABLED:
        // Remain in autonomous code; the goal-reached safety check
        // below will keep publishing zero velocity.
        return;
    }

    // =====================================================
    // ARUCO GOAL REACHED SAFETY CHECK
    // =====================================================
    if (nav_mode == 1 && aruco_goal_reached)
    {
        hardStop();
        return;
    }

    // =====================================================
    // ARUCO OBSTACLE AVOIDANCE
    // =====================================================
    const bool aruco_searching_forward =
        nav_mode == 1 &&
        CurrState == kSearchPattern &&
        FollowPattern == kMoveForward &&
        !spot_turn_back_;

    const bool aruco_following =
        nav_mode == 1 &&
        CurrState == kArucoFollowing;

    const bool obstacle_before_aruco =
        aruco_following &&
        aruco_detect &&
        obstacle_detect &&
        aruco_x > 0.0 &&
        obs_x > 0.0 &&
        static_cast<double>(obs_x) < aruco_x;

    const bool avoid_during_search =
        aruco_searching_forward &&
        obstacle_detect;

    if ((avoid_during_search || obstacle_before_aruco) &&
        CurrState != kObstacleAvoidance)
    {
        avoiding_obstacle_ = true;
        prev_state_ = CurrState;
        prev_search_pattern_ = FollowPattern;

        hardStop();
        CurrState = kObstacleAvoidance;

        RCLCPP_WARN(
            get_logger(),
            "[ARUCO OA] STOPPED -> entering obstacle avoidance");

        return;
    }

    // =====================================================
    // TARGET IS BEFORE THE OBSTACLE -> IGNORE OBSTACLE
    // =====================================================
    if (aruco_following &&
        aruco_detect &&
        obstacle_detect &&
        aruco_x > 0.0 &&
        obs_x > 0.0 &&
        aruco_x <= static_cast<double>(obs_x))
    {
        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[ARUCO] Target at %.2f m is before obstacle at %.2f m "
            "-> continuing ArUco following",
            aruco_x,
            obs_x);
    }

    // =====================================================
    // GPS OBSTACLE AVOIDANCE
    //
    // GPS goal has already been checked above, so we cannot
    // enter avoidance after reaching the goal.
    // =====================================================
    if (nav_mode == 0 &&
        obstacle_detect &&
        CurrState == kCoordinateFollowing &&
        gps_aligned_ &&
        !gps_waiting_ &&
        gps_goal_set &&
        !gps_goal_reached)
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

            hardStop();
            CurrState = kGPSObstacleAvoidance;

            RCLCPP_WARN(
                get_logger(),
                "[GPS AVOID] Obstacle at %.2f m, goal at %.2f m "
                "-> starting avoidance",
                obstacle_distance,
                goal_distance);

            return;
        }
    }

    // =====================================================
    // RUN CURRENT STATE
    // =====================================================
    switch (CurrState)
    {
        // kManualState intentionally disabled.
        // case kManualState:
        //     hardStop();
        //     break;

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

void SensorCallback::RoverStateClassifier()
{
    // =====================================================
    // MANUAL MODE DISABLED
    // =====================================================
    // This classifier no longer changes the rover into manual
    // mode after a goal or based on rover_state.
    //
    // Autonomous state transitions are handled by stackRun().
    // Function retained for compatibility with the existing
    // class declaration.
    return;
}

void SensorCallback::imuCallback(
    const msgs::msg::ImuData::SharedPtr msg)
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

void SensorCallback::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    // MANUAL MODE DISABLED: point clouds are processed continuously.

    // Convert the ZED2i PointCloud2 message (XYZ in the camera optical frame:
    // +x = right, +y = down, +z = forward) into a PCL cloud for processing.
    auto pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

    std::lock_guard<std::mutex> lock(state_mutex_);
    last_point_cloud_ = pcl_cloud;
    last_pointcloud_time_ = this->get_clock()->now();
}

void SensorCallback::arucoCallback(const msgs::msg::ArucoTag::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    if (msg->is_detected && msg->id == target_aruco_id_)
    {
        aruco_detect = true;
        aruco_x = static_cast<double>(msg->aruco_x);
        aruco_y = static_cast<double>(msg->aruco_y);
        last_aruco_time_ = this->get_clock()->now();
    }
}

void SensorCallback::stateCallback(
    const std_msgs::msg::Bool::SharedPtr state)
{
    (void)state;

    // =====================================================
    // MANUAL MODE DISABLED
    // =====================================================
    // The external autonomous/manual state topic is ignored.
    // This node remains autonomous and is not switched to manual.
    //
    // Kept as a no-op because the callback is still declared in
    // the existing class interface.
    return;
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
            // MANUAL MODE DISABLED.
            // Invalid input leaves the node stopped in navigation selection.
            CurrState = kNavigationModeSelect;
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
        // MANUAL MODE DISABLED:
        // Stay in navigation selection instead of entering manual.
        CurrState = kNavigationModeSelect;
    }

    nav_select_done_ = true;
}

void SensorCallback::coordinateFollowing()
{
    geometry_msgs::msg::Twist stop_cmd;

    // No active GPS goal -> remain stopped
    if (!gps_goal_set)
    {
        hardStop();
        return;
    }

    const auto now = get_clock()->now();

    // =====================================================
    // GPS FRESHNESS CHECK
    // =====================================================
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

    const double dist = haversine(curr_location, goal_location);

    // =====================================================
    // GOAL REACHED -> STOP COMPLETELY
    // =====================================================
    if (dist <= kDistanceThreshold)
    {
        if (!gps_goal_reached)
        {
            RCLCPP_INFO(
                get_logger(),
                "[GPS] GOAL REACHED | Distance: %.2f m",
                dist);
        }

        // Mark navigation as finished
        gps_goal_reached = true;
        gps_goal_set = false;

        // Reset GPS navigation state
        gps_aligned_ = false;
        gps_waiting_ = false;

        // Immediately publish zero velocity
        hardStop();

        // MANUAL MODE DISABLED:
        // Do not switch state or disable the external controller.
        // gps_goal_set=false makes coordinateFollowing() remain stopped.
        return;
    }

    gps_goal_reached = false;

    // =====================================================
    // CALCULATE TARGET BEARING AND HEADING ERROR
    // =====================================================
    const double target_bearing =
        gpsBearing(curr_location, goal_location);

    const double error =
        headingError(target_bearing, imu_yaw);

    const double shortest_error =
        std::min(error, 360.0 - error);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "[GPS] Distance: %.2f m | Target: %.1f deg | "
        "Current: %.1f deg | Error: %.1f deg",
        dist,
        target_bearing,
        imu_yaw,
        shortest_error);

    constexpr double HEADING_TOLERANCE = 10.0;
    constexpr double WAIT_TIME = 2.0;
    constexpr double CHECK_TIME = 2.0;
    constexpr double TURN_SPEED = 1.0;

    geometry_msgs::msg::Twist cmd;

    // =====================================================
    // STEP 1: ALIGN WITH GPS TARGET
    // =====================================================
    if (!gps_aligned_)
    {
        if (shortest_error <= HEADING_TOLERANCE)
        {
            gps_aligned_ = true;
            gps_waiting_ = true;
            gps_wait_end_ =
                now + rclcpp::Duration::from_seconds(WAIT_TIME);

            hardStop();
            return;
        }

        // No forward movement while turning
        cmd.linear.x = 0.0;

        if (error <= 180.0)
            cmd.angular.z = -TURN_SPEED;
        else
            cmd.angular.z = TURN_SPEED;

        publishVel(cmd);
        return;
    }

    // =====================================================
    // STEP 2: STOP AND VERIFY ALIGNMENT
    // =====================================================
    if (gps_waiting_)
    {
        hardStop();

        if (now < gps_wait_end_)
            return;

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
    // STEP 3: DRIVE STRAIGHT
    // =====================================================
    if ((now - gps_last_check_time_).seconds() >= CHECK_TIME)
    {
        gps_last_check_time_ = now;

        if (shortest_error > HEADING_TOLERANCE)
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

    // =====================================================
// DRIVE STRAIGHT TOWARD THE GOAL
//
// If the GPS goal is closer than the detected obstacle,
// the goal is effectively before the obstacle. Ignore
// obstacle avoidance and proceed directly at 0.75 m/s.
// =====================================================
const bool goal_before_obstacle =
    obstacle_detect &&
    dist < static_cast<double>(obs_x);

if (goal_before_obstacle)
{
    cmd.linear.x = 0.75;
}
else
{
    cmd.linear.x = 0.8;
}

cmd.angular.z = 0.0;

publishVel(cmd);
}

void SensorCallback::obstacleAvoidance()
{
    // =====================================================
    // ARUCO PRIORITY WHILE IN OBSTACLE AVOIDANCE
    //
    // If obstacle avoidance was entered while following an
    // ArUco, continuously compare their distances.
    //
    // ArUco <= obstacle -> FOLLOW ARUCO
    // Obstacle < ArUco -> KEEP AVOIDING
    // =====================================================
    if (nav_mode == 1 &&
        prev_state_ == kArucoFollowing &&
        aruco_detect &&
        aruco_x > 0.0)
    {
        const bool aruco_is_closer =
            !obstacle_detect ||
            obs_x <= 0.0 ||
            aruco_x <= static_cast<double>(obs_x);

        if (aruco_is_closer)
        {
            avoiding_obstacle_ = false;
            CurrState = kArucoFollowing;

            RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                500,
                "[ARUCO PRIORITY] ArUco %.2f m | Obstacle %.2f m "
                "-> ArUco is closer, resuming following",
                aruco_x,
                obs_x);

            // Run ArUco following immediately instead of
            // waiting for the next stackRun() cycle.
            arucoFollowing();
            return;
        }
    }

    // =====================================================
    // OBSTACLE STILL PRESENT -> TURN AWAY
    //
    // This only continues when the obstacle is genuinely
    // closer than the ArUco, or there is no valid ArUco.
    // =====================================================
    if (obstacle_detect)
    {
        avoiding_obstacle_ = true;

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;

        // Obstacle LEFT -> turn RIGHT
        // Obstacle RIGHT -> turn LEFT
        // Center/unknown -> default LEFT
        if (obs_side_ && std::string(obs_side_) == "left")
        {
            cmd.angular.z = -1.0;
        }
        else if (obs_side_ && std::string(obs_side_) == "right")
        {
            cmd.angular.z = 1.0;
        }
        else
        {
            cmd.angular.z = 1.0;
        }

        publishVel(cmd);

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            500,
            "[ARUCO OA] AVOIDING | obstacle=%.2f m forward | "
            "lateral=%.2f m | side=%s | turning=%.2f rad/s",
            obs_x,
            obs_y,
            obs_side_ ? obs_side_ : "unknown",
            cmd.angular.z);

        return;
    }

    // =====================================================
    // OBSTACLE CLEARED
    // =====================================================
    hardStop();
    avoiding_obstacle_ = false;

    RCLCPP_INFO(
        get_logger(),
        "[ARUCO OA] Obstacle cleared");

    // =====================================================
    // OA INTERRUPTED ARUCO FOLLOWING
    // =====================================================
    if (nav_mode == 1 && prev_state_ == kArucoFollowing)
    {
        // Marker survived / is still visible -> follow it again
        if (aruco_detect)
        {
            CurrState = kArucoFollowing;

            RCLCPP_INFO(
                get_logger(),
                "[ARUCO OA] ArUco still visible -> "
                "resuming ArUco following");

            return;
        }

        // Marker disappeared during avoidance -> fresh forward search
        resetSearchPattern();
        FollowPattern = kMoveForward;
        CurrState = kSearchPattern;

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO OA] ArUco lost during avoidance -> "
            "returning to search pattern at MOVE FORWARD");

        return;
    }

    // =====================================================
    // OA INTERRUPTED SEARCH PATTERN
    // =====================================================
    if (nav_mode == 1 && prev_state_ == kSearchPattern)
    {
        // Resume the exact search phase that was interrupted.
        FollowPattern = prev_search_pattern_;
        CurrState = kSearchPattern;

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO OA] Resuming interrupted search phase");

        return;
    }

    // =====================================================
    // SAFETY FALLBACK
    // =====================================================
    hardStop();

    RCLCPP_WARN(
        get_logger(),
        "[ARUCO OA] Unexpected previous state -> stopping safely");
}


void SensorCallback::gpsObstacleAvoidance()
{
    // =====================================================
    // SAFETY: GOAL REACHED WHILE AVOIDING
    // =====================================================
    if (nav_mode == 0 &&
        gps_goal_set &&
        !gps_goal_reached)
    {
        const double dist =
            haversine(curr_location, goal_location);

        if (dist <= kDistanceThreshold)
        {
            gps_goal_reached = true;
            gps_goal_set = false;

            gps_avoiding_ = false;
            gps_avoid_direction_ = 0;
            gps_avoid_moving_forward_ = false;
            gps_aligned_ = false;
            gps_waiting_ = false;

            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[GPS] GOAL REACHED DURING AVOIDANCE -> STOPPED | "
                "Distance: %.2f m",
                dist);

            // MANUAL MODE DISABLED.
            // Stay in autonomous code; goal flags keep the rover stopped.
            return;
        }
    }

    // =====================================================
    // MANUAL MODE DISABLED
    // =====================================================
    // No manual-state safety gate is used here.
    //
    geometry_msgs::msg::Twist cmd;
    const auto now = get_clock()->now();

    constexpr double TURN_SPEED = 1.0;
    constexpr double FORWARD_SPEED = 1.1;
    constexpr double FORWARD_TIME = 2.0;

    // =====================================================
    // STEP 1: TURN AWAY FROM THE OBSTACLE
    // =====================================================
    if (!gps_avoid_moving_forward_)
    {
        if (obstacle_detect)
        {
            if (gps_avoid_direction_ == 0)
            {
                if (obs_side_ &&
                    std::string(obs_side_) == "left")
                {
                    gps_avoid_direction_ = -1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] Obstacle LEFT -> turning RIGHT");
                }
                else if (obs_side_ &&
                         std::string(obs_side_) == "right")
                {
                    gps_avoid_direction_ = 1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] Obstacle RIGHT -> turning LEFT");
                }
                else
                {
                    gps_avoid_direction_ = 1;

                    RCLCPP_WARN(
                        get_logger(),
                        "[GPS AVOID] Obstacle CENTER/UNKNOWN -> turning LEFT");
                }
            }

            cmd.linear.x = 0.0;
            cmd.angular.z =
                gps_avoid_direction_ * TURN_SPEED;

            publishVel(cmd);
            return;
        }

        // Front clear -> start forward movement
        gps_avoid_moving_forward_ = true;

        gps_avoid_forward_end_ =
            now + rclcpp::Duration::from_seconds(FORWARD_TIME);

        cmd.linear.x = FORWARD_SPEED;
        cmd.angular.z = 0.0;

        publishVel(cmd);

        RCLCPP_INFO(
            get_logger(),
            "[GPS AVOID] Front clear -> moving forward for %.1f seconds",
            FORWARD_TIME);

        return;
    }

    // =====================================================
    // STEP 2: MOVE FORWARD
    // =====================================================
    if (now < gps_avoid_forward_end_)
    {
        // Check goal AGAIN because the rover may physically
        // reach it during this forward movement.
        if (gps_goal_set)
        {
            const double dist =
                haversine(curr_location, goal_location);

            if (dist <= kDistanceThreshold)
            {
                gps_goal_reached = true;
                gps_goal_set = false;

                gps_avoiding_ = false;
                gps_avoid_moving_forward_ = false;

                hardStop();

                RCLCPP_INFO(
                    get_logger(),
                    "[GPS] GOAL REACHED -> STOPPED | Distance: %.2f m",
                    dist);

                // MANUAL MODE DISABLED:
                // Do not switch state or disable the external controller.
                // Goal flags keep the rover stopped.
                return;
            }
        }

        cmd.linear.x = FORWARD_SPEED;
        cmd.angular.z = 0.0;

        publishVel(cmd);
        return;
    }

    // =====================================================
    // STEP 3: RETURN TO GPS ALIGNMENT
    // =====================================================
    hardStop();

    gps_avoiding_ = false;
    gps_avoid_direction_ = 0;
    gps_avoid_moving_forward_ = false;

    gps_aligned_ = false;
    gps_waiting_ = false;

    CurrState = kCoordinateFollowing;

    RCLCPP_INFO(
        get_logger(),
        "[GPS AVOID] Forward movement complete -> returning to GPS alignment");
}

void SensorCallback::arucoFollowing()
{
    // =====================================================
    // MARKER LOST
    // =====================================================
    if (!aruco_detect)
    {
        hardStop();
        return;
    }

    geometry_msgs::msg::Twist cmd;

    constexpr double ANGULAR_GAIN = 0.8;
    constexpr double Y_DEADBAND = 0.05;
    constexpr double MAX_ANGULAR = 1.5;

    constexpr double FOLLOW_SPEED = 0.75;
    constexpr double SPEED_SCALING_DISTANCE = 3.5;
    constexpr double MIN_FOLLOW_SPEED = 0.50;

    // =====================================================
    // LATERAL ALIGNMENT
    // =====================================================
    double lateral_error = aruco_y;

    if (std::abs(lateral_error) < Y_DEADBAND)
        lateral_error = 0.0;

    cmd.angular.z = std::clamp(
        -ANGULAR_GAIN * lateral_error,
        -MAX_ANGULAR,
        MAX_ANGULAR);

    // =====================================================
    // DISTANCE-BASED LINEAR VELOCITY
    // =====================================================
    // At >= 3.5 m: full speed.
    // Below 3.5 m: quadratic scaling slows the rover more
    // aggressively as it approaches the marker.
    if (aruco_x >= SPEED_SCALING_DISTANCE)
    {
        cmd.linear.x = std::min(FOLLOW_SPEED, kMaxLinearVel);
    }
    else
    {
        double distance_scale = std::clamp(
            aruco_x / SPEED_SCALING_DISTANCE,
            0.0,
            1.0);

        // Quadratic scaling: more aggressive slowdown near goal
        distance_scale = distance_scale * distance_scale;

        const double scaled_speed =
            MIN_FOLLOW_SPEED +
            (FOLLOW_SPEED - MIN_FOLLOW_SPEED) * distance_scale;

        cmd.linear.x = std::min(scaled_speed, kMaxLinearVel);
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[ARUCO] Following | Distance: %.2f m | "
        "Lateral error: %.2f | Lin: %.2f | Ang: %.2f",
        aruco_x,
        aruco_y,
        cmd.linear.x,
        cmd.angular.z);

    publishVel(cmd);
}


void SensorCallback::callSearchPattern()
{
    // =====================================================
    // SEARCH PATTERN SPEEDS
    // Local constants so this function does not depend on
    // undefined global/class constants.
    // =====================================================
    constexpr double LINEAR_SPEED = 0.75;
    constexpr double ANGULAR_SPEED = 1.0;

    geometry_msgs::msg::Twist cmd;
    const auto now = get_clock()->now();

    // =====================================================
    // OPTIONAL INITIAL SPOT TURN
    // =====================================================
    if (spot_turn_back_ && !spot_done_)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = ANGULAR_SPEED;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SEARCH][SPOT] Turning in place | ang=%.2f",
            cmd.angular.z);

        if (now >= search_end_time_)
        {
            spot_done_ = true;
            search_ref_set_ = false;

            // Stop before starting the forward pattern
            hardStop();

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][SPOT] Turn done -> starting search pattern");
        }

        return;
    }

    // =====================================================
    // INITIALIZE SEARCH PATTERN
    // =====================================================
    if (!search_ref_set_)
    {
        FollowPattern = kMoveForward;

        search_end_time_ =
            now + rclcpp::Duration::from_seconds(search_forward_time_);

        search_ref_set_ = true;

        RCLCPP_INFO(
            get_logger(),
            "[SEARCH] Starting search pattern -> MOVE FORWARD | "
            "duration=%.2f sec | cycle=%d | skew=%d",
            search_forward_time_,
            search_cycle_,
            search_skew);

        cmd.linear.x = LINEAR_SPEED;
        cmd.angular.z = 0.0;
        publishVel(cmd);

        return;
    }

    // =====================================================
    // MOVE FORWARD
    // =====================================================
    if (FollowPattern == kMoveForward)
    {
        cmd.linear.x = LINEAR_SPEED;
        cmd.angular.z = 0.0;
        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SEARCH][FORWARD] lin=%.2f | cycle=%d | skew=%d",
            cmd.linear.x,
            search_cycle_,
            search_skew);

        if (now >= search_end_time_)
        {
            hardStop();

            search_cycle_++;

            FollowPattern = kTurnA;

            search_end_time_ =
                now + rclcpp::Duration::from_seconds(3.5);

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][FORWARD] Done -> TURN A | cycle=%d",
                search_cycle_);
        }

        return;
    }

    // =====================================================
    // TURN A
    // =====================================================
    if (FollowPattern == kTurnA)
    {
        const bool right_skew = (search_skew == kRightSkew);

        cmd.linear.x = 0.0;
        cmd.angular.z =
            right_skew ? -ANGULAR_SPEED : ANGULAR_SPEED;

        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SEARCH][TURN A] ang=%.2f | cycle=%d | skew=%d",
            cmd.angular.z,
            search_cycle_,
            search_skew);

        if (now >= search_end_time_)
        {
            hardStop();

            FollowPattern = kTurnB;

            search_end_time_ =
                now + rclcpp::Duration::from_seconds(7.0);

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][TURN A] Done -> TURN B");
        }

        return;
    }

    // =====================================================
    // TURN B
    // =====================================================
    if (FollowPattern == kTurnB)
    {
        const bool right_skew = (search_skew == kRightSkew);

        cmd.linear.x = 0.0;
        cmd.angular.z =
            right_skew ? ANGULAR_SPEED : -ANGULAR_SPEED;

        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SEARCH][TURN B] ang=%.2f | cycle=%d | skew=%d",
            cmd.angular.z,
            search_cycle_,
            search_skew);

        if (now >= search_end_time_)
        {
            hardStop();

            FollowPattern = kTurnC;

            const double extra =
                (search_skew != kNoSkew)
                    ? static_cast<double>(search_cycle_)
                    : 0.0;

            search_end_time_ =
                now + rclcpp::Duration::from_seconds(3.5 + extra);

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][TURN B] Done -> TURN C | extra=%.2f sec",
                extra);
        }

        return;
    }

    // =====================================================
    // TURN C
    // =====================================================
    if (FollowPattern == kTurnC)
    {
        const bool right_skew = (search_skew == kRightSkew);

        cmd.linear.x = 0.0;
        cmd.angular.z =
            right_skew ? -ANGULAR_SPEED : ANGULAR_SPEED;

        publishVel(cmd);

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "[SEARCH][TURN C] ang=%.2f | cycle=%d | skew=%d",
            cmd.angular.z,
            search_cycle_,
            search_skew);

        if (now >= search_end_time_)
        {
            hardStop();

            FollowPattern = kMoveForward;

            search_end_time_ =
                now + rclcpp::Duration::from_seconds(
                    search_forward_time_);

            RCLCPP_INFO(
                get_logger(),
                "[SEARCH][TURN C] Done -> MOVE FORWARD | cycle=%d",
                search_cycle_);
        }

        return;
    }

    // =====================================================
    // SAFETY FALLBACK
    // =====================================================
    RCLCPP_WARN(
        get_logger(),
        "[SEARCH] Unknown search phase -> STOPPING");

    hardStop();
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
    vel_pub->publish(stop);
}

// =====================================================
// MANUAL MODE DISABLED
// =====================================================
// disableAutonomous() is intentionally commented out.
// The node no longer toggles the external autonomous/manual
// controller when a GPS or ArUco goal is reached.
//
// Original implementation retained below for reference.
//
//
// void SensorCallback::disableAutonomous()
// {
//     // =====================================================
//     // LOCAL SAFETY LOCKOUT FIRST
//     //
//     // The service request below is asynchronous. Therefore
//     // we MUST stop autonomous execution locally BEFORE
//     // waiting for the external controller to respond.
//     // =====================================================
//     rover_state = false;
//     last_rover_state = false;
//
//     CurrState = kManualState;
//
//     // Cancel active autonomous movement
//     gps_goal_set = false;
//
//     gps_aligned_ = false;
//     gps_waiting_ = false;
//     gps_avoiding_ = false;
//     gps_avoid_direction_ = 0;
//     gps_avoid_moving_forward_ = false;
//
//     avoiding_obstacle_ = false;
//
//     // =====================================================
//     // IMMEDIATELY STOP THE ROVER
//     // =====================================================
//     hardStop();
//
//     // Send several zero commands while we are still the
//     // autonomous cmd_vel publisher.
//     geometry_msgs::msg::Twist stop;
//
//     for (int i = 0; i < 5; ++i)
//     {
//         vel_pub->publish(stop);
//     }
//
//     RCLCPP_INFO(
//         get_logger(),
//         "[MODE] Local autonomous safety lockout -> STOPPED");
//
//     // =====================================================
//     // THEN DISABLE THE EXTERNAL AUTONOMOUS CONTROLLER
//     // =====================================================
//     if (!toggle_client_->wait_for_service(
//             std::chrono::seconds(1)))
//     {
//         RCLCPP_ERROR(
//             get_logger(),
//             "[MODE] toggle_autonomous service not available");
//
//         // We remain locally locked in manual regardless.
//         hardStop();
//         return;
//     }
//
//     auto req =
//         std::make_shared<std_srvs::srv::Trigger::Request>();
//
//     toggle_client_->async_send_request(
//         req,
//         [this](
//             rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
//         {
//             // Safety: publish zero before processing response.
//             hardStop();
//
//             auto res = future.get();
//
//             if (res->success)
//             {
//                 // Final explicit stop.
//                 hardStop();
//
//                 RCLCPP_INFO(
//                     this->get_logger(),
//                     "[MODE] Autonomous DISABLED via service");
//             }
//             else
//             {
//                 // NEVER resume autonomous movement if the
//                 // external service fails.
//                 rover_state = false;
//                 CurrState = kManualState;
//
//                 hardStop();
//
//                 RCLCPP_ERROR(
//                     this->get_logger(),
//                     "[MODE] Failed to disable autonomy: %s",
//                     res->message.c_str());
//             }
//         });
// }
//

void SensorCallback::obstacleClassifier()
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    // No point cloud available
    if (!last_point_cloud_ || last_point_cloud_->points.empty())
    {
        obstacle_detect = false;
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "none";
        return;
    }

    bool found = false;
    float best_forward = std::numeric_limits<float>::max();
    float nearest_forward = 0.0f;
    float nearest_lateral = 0.0f;

    // Detection corridor: 1.3 m to either side of the rover center
    constexpr float HALF_WIDTH = 1.10f;

    // Only consider obstacles between 0.3 m and 3.0 m ahead
    constexpr float MIN_FORWARD = 0.30f;
    constexpr float MAX_FORWARD = 2.70f;

    // Ignore the area around the detected ArUco marker
    constexpr float ARUCO_MASK_RADIUS = 0.40f;

    // Process every second point for lower CPU usage
    constexpr size_t STRIDE = 1;

    const auto &cloud = *last_point_cloud_;

    for (size_t i = 0; i < cloud.points.size(); i += STRIDE)
    {
        const auto &pt = cloud.points[i];

        // Ignore invalid points
        if (!std::isfinite(pt.x) ||
            !std::isfinite(pt.y) ||
            !std::isfinite(pt.z))
        {
            continue;
        }

        // ROS-style frame:
        // +X = forward
        // +Y = left
        // +Z = up
        const float forward = pt.x;
        const float lateral = pt.y;

        // Only consider points within the detection range and corridor
        if (forward < MIN_FORWARD ||
            forward > MAX_FORWARD ||
            std::abs(lateral) > HALF_WIDTH)
        {
            continue;
        }

        // Ignore the area around the detected ArUco marker
        if (aruco_detect)
        {
            const float dx =
                forward - static_cast<float>(aruco_x);
            const float dy =
                lateral - static_cast<float>(aruco_y);

            if ((dx * dx) + (dy * dy) <
                (ARUCO_MASK_RADIUS * ARUCO_MASK_RADIUS))
            {
                continue;
            }
        }

        // Keep the closest obstacle in the forward direction
        if (forward < best_forward)
        {
            best_forward = forward;
            nearest_forward = forward;
            nearest_lateral = lateral;
            found = true;
        }
    }

    obstacle_detect = found;

    // No obstacle found
    if (!found)
    {
        obs_x = 0.0;
        obs_y = 0.0;
        obs_side_ = "none";
        return;
    }

    // Store obstacle position
    obs_x = nearest_forward;
    obs_y = nearest_lateral;

    // +Y is left, -Y is right
    if (obs_y >= 0.0)
    {
        obs_side_ = "left";
    }
    else
    {
        obs_side_ = "right";
    }

    // Debug log: printed at most once every 500 ms
    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "[OBSTACLE] Detected | Forward: %.2f m | Lateral: %.2f m | Side: %s",
        obs_x,
        obs_y,
        obs_side_ ? obs_side_ : "unknown");
}
  

void SensorCallback::setGoalStatus()
{
    // Only check ArUco goal while actively following a valid target
    if (nav_mode != 1 ||
        CurrState != kArucoFollowing ||
        aruco_goal_reached ||
        !aruco_detect)
    {
        return;
    }

    // Invalid/behind-camera distance -> ignore
    if (aruco_x < 0.0)
        return;

    // Target reached
    if (aruco_x <= kDistanceThreshold)
    {
        aruco_goal_reached = true;

        // Immediately kill motion
        hardStop();

        RCLCPP_INFO(
            get_logger(),
            "[ARUCO] GOAL REACHED -> STOPPED | Distance: %.2f m",
            aruco_x);
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
