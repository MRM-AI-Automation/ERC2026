cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# ============================================================
# planner_node
# ============================================================

add_executable(planner_node
  src/main.cpp
  src/irc_planner.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(planner_node
  ${PCL_LIBRARIES}
)

target_compile_definitions(planner_node PUBLIC
  ${PCL_DEFINITIONS}
)


# ============================================================
# lidar_planner_node
# ============================================================

add_executable(lidar_planner_node
  src/lidar_main.cpp
  src/lidar_planner.cpp
)

target_include_directories(lidar_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(lidar_planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(lidar_planner_node
  ${PCL_LIBRARIES}
)

target_compile_definitions(lidar_planner_node PUBLIC
  ${PCL_DEFINITIONS}
)


# ============================================================
# relay_node
# ============================================================

add_executable(relay_node
  src/relay_node.cpp
)

ament_target_dependencies(relay_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  msgs
)


# ============================================================
# obstacle_detector
# ============================================================

add_executable(obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)


# ============================================================
# angle_turn_controller
# ============================================================

add_executable(angle_turn_controller
  src/angle_turn_controller.cpp
)

ament_target_dependencies(angle_turn_controller
  rclcpp
  geometry_msgs
  msgs
)


# ============================================================
# Install
# ============================================================

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    planner_node
    lidar_planner_node
    relay_node
    obstacle_detector
    angle_turn_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    src/mode_switch.py
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
