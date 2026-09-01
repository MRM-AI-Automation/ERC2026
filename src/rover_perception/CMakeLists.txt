cmake_minimum_required(VERSION 3.8)
project(rover_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)

find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# --------------------------------------------------
# PointCloud -> LaserScan
# --------------------------------------------------

add_executable(pointcloud_to_laserscan
  src/pointcloud_to_laserscan.cpp
)

ament_target_dependencies(pointcloud_to_laserscan
  rclcpp
  sensor_msgs
  std_msgs
)

# --------------------------------------------------
# Scan Fusion
# --------------------------------------------------

add_executable(scan_fusion_node
  src/scan_fusion_node.cpp
)

ament_target_dependencies(scan_fusion_node
  rclcpp
  sensor_msgs
  laser_geometry
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# --------------------------------------------------
# Front 180 Degree Scan Filter
# --------------------------------------------------

add_executable(front_scan_filter
  src/front_scan_filter.cpp
)

ament_target_dependencies(front_scan_filter
  rclcpp
  sensor_msgs
)

# --------------------------------------------------
# ArUco Detector
# --------------------------------------------------

install(
  PROGRAMS
    src/aruco_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aruco_detector
)

# --------------------------------------------------
# Install Nodes
# --------------------------------------------------

install(
  TARGETS
    pointcloud_to_laserscan
    scan_fusion_node
    front_scan_filter
  DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------
# Launch + Config
# --------------------------------------------------

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# --------------------------------------------------
# Testing
# --------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
