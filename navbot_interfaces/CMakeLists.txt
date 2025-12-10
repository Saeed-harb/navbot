cmake_minimum_required(VERSION 3.8)
project(navbot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PIDGain.msg"
  "msg/Velocity.msg"
  "msg/SingleValue.msg"
  "msg/MultiValues.msg"
  "msg/Pose.msg"
  "srv/GetValue.srv"
 )


ament_package()
