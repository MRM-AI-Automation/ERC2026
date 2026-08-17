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

constexpr double kRoverLength = 1.50;
constexpr double kRoverBreadth = 1.25;
constexpr double kMaxLinearVel = 1.0;
constexpr double kMaxAngularVel = 1.5;
constexpr double kDistanceThreshold = 2.0;

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
    rclcpp::Subscription<aruco_msgs::msg::ImuData>::SharedPtr external_imu_sub_;
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

    Coordinates curr_location;
    Coordinates goal_location;
    rclcpp::Time last_gps_time_;

    bool gps_wall_following_active_{false};
    double gps_wall_target_distance_{0.0};

    bool aruco_detect;
    bool aruco_goal_reached;
    double aruco_x;
    double aruco_y;
    rclcpp::Time last_aruco_time_;

    bool obstacle_detect;
    double obs_x;
    double obs_y;

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

    double zed_yaw;
    double bno_yaw;
    double current_orientation;

    std::vector<double> obj_follow_linear;
    std::vector<double> obj_follow_angular;

    std::mutex state_mutex_;

    bool obstacle_clear_timing_{false};
    rclcpp::Time obstacle_clear_since_;

    bool bearing_locked_;
    double locked_bearing_deg_;

    bool search_aligned_;
    bool search_timing_;
    double search_offset_deg_;

    sensor_msgs::msg::LaserScan::SharedPtr last_lidar_scan_;

    void stackRun();
    void RoverStateClassifier();

    void imuCallback(const aruco_msgs::msg::ImuData::SharedPtr msg);
    void externalImuCallback(const aruco_msgs::msg::ImuData::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix);
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void arucoCallback(const aruco_msgs::msg::ArucoTag::SharedPtr msg);
    void stateCallback(const std_msgs::msg::Bool::SharedPtr state);

    void coordinateFollowing();
    void obstacleAvoidance();
    void gpsObstacleAvoidance();
    void arucoFollowing();
    void callSearchPattern();
    void navigationModeSelect();

    void publishVel(const geometry_msgs::msg::Twist& msg);
    void hardStop();
    void disableAutonomous();
    void obstacleClassifier();
    void setGoalStatus();
    void setSearchSkew(int skew);
    void resetSearchPattern();
    bool isArucoFresh();

    std::vector<double> straightLineEquation(
        double x1,
        double y1,
        double x2,
        double y2);

    double haversine(Coordinates curr, Coordinates dest);
    double gpsBearing(Coordinates curr, Coordinates dest);
    double gpsAngleFix(double angle);
    double headingError(double target, double current);
    double normalize360(double angle);

    std::vector<std::vector<double>> obstacleDataType1;
    bool obstacleIs;
};

}

#endif
