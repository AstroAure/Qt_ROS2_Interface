cmake_minimum_required(VERSION 3.8)
project(minimal_ros2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(src)

add_executable(talker nodes/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener nodes/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(mirror nodes/external_ros_node.cpp src/external_ros.cpp)
ament_target_dependencies(mirror rclcpp std_msgs)

install(TARGETS
    talker listener mirror
    DESTINATION lib/${PROJECT_NAME})

ament_package()
