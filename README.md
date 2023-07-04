# Qt_ROS2_Interface
A minimal ROS2 package for receiving and emitting Qt signals through a ROS2 Node and defining Qt objects that can compile using 'colcon build' and run using 'ros2 run'

## Description
- minimal_ros2_demo : Simple demo of ROS2. This package contains three executables (nodes) : talker (publish a message every 0.5s on cmd_topic), listener (listens to messages on fdb_topic), mirror (receives messages on cmd_topic and published them to fdb_topic)

## Installation
1. Download the folders in the src folder of a ROS2 workspace
2. Build using 'colcon build' from your workspace
3. Source using 'source install/setup.bash'

## Run
'ros2 run <package_name> <executable_name>'
