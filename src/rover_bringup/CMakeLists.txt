cmake_minimum_required(VERSION 3.8)
project(rover_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(motorcontroller
  src/motorcontroller.cpp
)

ament_target_dependencies(motorcontroller
  rclcpp
  geometry_msgs
)

install(
  TARGETS
    motorcontroller
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
