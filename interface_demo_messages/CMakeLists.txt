cmake_minimum_required(VERSION 3.8)
project(interface_demo_messages)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/DiscreteFdb.srv"
    "srv/DiscreteCmd.srv"
    DEPENDENCIES std_msgs
)

ament_package()
