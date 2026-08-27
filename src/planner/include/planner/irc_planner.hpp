#ifndef IRC_PLANNER_H_
#define IRC_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <memory>

#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "aruco_msgs/msg/aruco_tag.hpp"
#include "aruco_msgs/msg/imu_data.hpp"

namespace planner
{

constexpr double kMaxLinearVel = 5.0;
constexpr double kMaxAngularVel = 5.0;

constexpr double kMinLinearVel = 0.0;
constexpr double kMinAngularVel = 0.0;

constexpr double kDistanceThreshold = 1.0;
constexpr double kHeadingTolerance = 10.0;
constexpr double kAlignWaitSec = 2.0;

enum State
{
    kManualState,
    kNavigationModeSelect,
    kCoordinateFollowing,
    kSearchPattern,
    kArucoFollowing,
    kObstacleAvoidance,
    kGPSObstacleAvoidance
};

enum SearchPattern
{
    kTurnA,
    kTurnB,
    kTurnC,
    kMoveForward
};

enum SearchSkew
{
    kNoSkew = 0,
    kLeftSkew = -1,
    kRightSkew = 1
};

struct Coordinates
{
    double latitude;
    double longitude;
};

class SensorCallback : public rclcpp::Node
{
public:
    SensorCallback();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    rclcpp::Subscription<aruco_msgs::msg::ImuData>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<aruco_msgs::msg::ArucoTag>::SharedPtr aruco_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_sub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_client_;
    rclcpp::TimerBase::SharedPtr stack_timer_;

    State CurrState;
    SearchPattern FollowPattern;

    int nav_mode;
    int target_aruco_id_;

    bool nav_select_done_;
    bool rover_state;
    bool last_rover_state;

    bool gps_goal_set;
    bool gps_goal_reached;

    bool gps_aligned_;
    bool gps_waiting_{false};

    rclcpp::Time gps_wait_end_;
    rclcpp::Time gps_last_check_time_;
    rclcpp::Time last_gps_time_;

    Coordinates curr_location;
    Coordinates goal_location;

    double imu_yaw;

    bool aruco_detect;
    bool aruco_goal_reached;

    double aruco_x;
    double aruco_y;

    rclcpp::Time last_aruco_time_;

    bool obstacle_detect;

    double obs_x;
    double obs_y;

    const char *obs_side_{"center"};

    sensor_msgs::msg::LaserScan::SharedPtr last_lidar_scan_;

    bool search_ref_set_;
    bool spot_turn_back_;
    bool spot_done_;

    int search_cycle_;

    rclcpp::Time search_end_time_;
    double search_forward_time_;

    SearchSkew search_skew;

    bool avoiding_obstacle_;
    State prev_state_;
    SearchPattern prev_search_pattern_;

    std::mutex state_mutex_;

    void stackRun();
    void RoverStateClassifier();

    void imuCallback(
        const aruco_msgs::msg::ImuData::SharedPtr msg);

    void gpsCallback(
        const sensor_msgs::msg::NavSatFix::SharedPtr fix);

    void lidarCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void arucoCallback(
        const aruco_msgs::msg::ArucoTag::SharedPtr msg);

    void stateCallback(
        const std_msgs::msg::Bool::SharedPtr state);

    void coordinateFollowing();
    void gpsObstacleAvoidance();
    void navigationModeSelect();

    void arucoFollowing();
    void callSearchPattern();

    void obstacleAvoidance();
    void obstacleClassifier();
    void setGoalStatus();

    void setSearchSkew(int skew);
    void resetSearchPattern();

    void publishVel(const geometry_msgs::msg::Twist &msg);
    void hardStop();
    void disableAutonomous();

    double haversine(Coordinates curr, Coordinates dest);
    double gpsBearing(Coordinates curr, Coordinates dest);
    double headingError(double target, double current);
};

} // namespace planner

#endif // IRC_PLANNER_H_
