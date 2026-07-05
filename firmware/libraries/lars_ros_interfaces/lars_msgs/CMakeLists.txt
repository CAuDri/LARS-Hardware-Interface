# CAuDri - ROS interface package build file for LARS firmware messages

cmake_minimum_required(VERSION 3.8)
project(lars_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
    "msg/Float32Stamped.msg"
    "msg/MotorFeedback.msg"
    "msg/MotorTelemetry.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
