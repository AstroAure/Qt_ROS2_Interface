# Qt_ROS2_Interface
A minimal ROS2 package for receiving and emitting Qt signals through a ROS2 Node and defining Qt objects that can compile using ```colcon build``` and run using ```ros2 run```.

## Description
- ```minimal_ros2_demo``` : Simple demo of ROS2. This package contains three executables (nodes) : ```talker``` (publish a message every 0.5s on ```cmd_topic```), ```listener``` (listens to messages on ```fdb_topic```), ```mirror``` (receives messages on ```cmd_topic``` and publishes them to ```fdb_topic```).
- ```qt_object``` : Minimal definition of a QObject compilable using ```colcon build```. This package contains only one executable : ```qt_object_node``` which creates a QtObject and a QTimer. They are connected so that the ```QTimer::timeout``` signal triggers the ```QtObject::onSignalReceived``` slot and prints "Signal received !" in the terminal. The QtObject also emits a ```signalReceived``` signal.
- ```interface_node_demo``` : Package showing an example interface between a QObject using signals and slots and a ROS2 Node using topics. This package contains two executables : ```external_ros``` and ```interface```. ```external_ros``` emulates a ROS2 computer using topics and services to send commands[^command] and receive feedback[^feedback] (publishes a topic command every 0.1s on ```cmd_topic```, listens for topic feedback on ```fdb_topic```, writes a command request every 1s on ```cmd_service``` and reads from feedback service every 2s on ```fdb_service```). ```interface``` creates a QObject defined in ```external_qt``` which receives Qt signals (discrete[^discrete] and continuous[^continuous] command) and emits signals (discrete and continuous feedback); and a mixed QObject/Node object which receives the ROS2 commands, emits the Qt command signals, receives the Qt feedback signals and transmits the feedback to ROS2.

The workings of the Qt-ROS2 interface is pretty logical in the ```interface.h```, ```interface.cpp``` and ```interface_node.cpp``` files but the main complexity lies in the ```CMakeLists.txt``` of the ```interface_node_demo``` package. Some explanations are provided as comments but it is recommanded to keep this base and only add dependencies and packages if needed.

[^command]: Communication from ROS2 to Qt.
[^feedback]: Communication from Qt to ROS2.
[^discrete]: Discrete is used here for command and feedback that aren't updated continuously (version infos, max speeed...) or can't be updated continuously (emergency stop, state reset...). This is using ROS2 services.
[^continuous]: Continuous is used here for command and feedback updated (or which could be udpated) very regularly (odometry, velocity command...). This is using ROS2 topics.

## Requirements
Requires Qt5 (```sudo apt-get install -y qtbase5-dev```) and [ROS2](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) to be installed

## Installation
1. Download the folders in the src folder of a ROS2 workspace
2. Build using ```colcon build``` from your workspace
3. Source using ```source install/setup.bash```

## Run
```ros2 run <package_name> <executable_name>```
