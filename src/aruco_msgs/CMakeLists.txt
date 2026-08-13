cmake_minimum_required(VERSION 3.8)
project(aruco_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoTag.msg"
  "msg/ImuData.msg"
  "msg/ArmPwm.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
)

ament_package()
