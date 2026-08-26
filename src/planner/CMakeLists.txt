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
find_package(aruco_msgs REQUIRED)


# ============================================
# Main Planner Node
# ============================================
add_executable(planner_node
  src/main.cpp
  src/irc_planner.cpp
)

target_include_directories(planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(planner_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  aruco_msgs
)


# ============================================
# Relay Node
# ============================================
add_executable(relay_node
  src/relay_node.cpp
)

ament_target_dependencies(relay_node
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  aruco_msgs
)


# ============================================
# Obstacle Detector
# ============================================
add_executable(obstacle_detector
  src/obstacle_detector.cpp
)

ament_target_dependencies(obstacle_detector
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)


# ============================================
# Angle Turn Controller
# ============================================
add_executable(angle_turn_controller
  src/angle_turn_controller.cpp
)

ament_target_dependencies(angle_turn_controller
  rclcpp
  geometry_msgs
  aruco_msgs
)


# ============================================
# Install Headers
# ============================================
install(
  DIRECTORY include/
  DESTINATION include
)


# ============================================
# Install C++ Executables
# ============================================
install(
  TARGETS
    planner_node
    relay_node
    obstacle_detector
    angle_turn_controller
  DESTINATION lib/${PROJECT_NAME}
)


# ============================================
# Install Python Nodes
# ============================================
install(
  PROGRAMS
    src/mode_switch.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_lint_auto_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
