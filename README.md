# Qt_ROS2_Interface
A minimal ROS2 package for receiving and emitting Qt signals through a ROS2 Node and defining Qt objects that can compile using ```colcon build``` and run using ```ros2 run```.

## Description
- ```minimal_ros2_demo``` : Simple demo of ROS2. This package contains three executables (nodes) : ```talker``` (publish a message every 0.5s on ```cmd_topic```), ```listener``` (listens to messages on ```fdb_topic```), ```mirror``` (receives messages on ```cmd_topic``` and publishes them to ```fdb_topic```).
- ```qt_object``` : Minimal definition of a QObject compilable using ```colcon build```. This package contains only one executable : ```qt_object_node``` which creates a QtObject and a QTimer. They are connected so that the ```QTimer::timeout``` signal triggers the ```QtObject::onSignalReceived``` slot and prints "Signal received !" in the terminal. The QtObject also emits a ```signalReceived``` signal.
- ```interface_node_demo``` : Package showing a minimal interface between a QObject using signals and slots and a ROS2 Node using topics. This package contains two executables : ```external_ros``` (publish a command every 0.5s on ```cmd_topic``` and listens for feedback on ```fdb_topic```) and ```interface```. ```interface``` creates a QObject defined in ```external_qt``` which receives a Qt signal (command) and emits another signal (feedback) with the same message ; and a mixed QObject/Node object which receives the ROS2 command topic, emits the Qt command signal, receives the Qt feedback signal and publishes the ROS2 feedback topic. A third executable (```external_qt```) is disabled but can be enabled to test just Qt part (the result is identical to the ```qt_object``` package).

The workings of the Qt-ROS2 interface is pretty logical in the ```interface.h```, ```interface.cpp``` and ```interface_node.cpp``` files but the main complexity lies in the ```CMakeLists.txt``` of the ```interface_node_demo``` package. Some explanations are provided as comments but it is recommanded to keep this base and only add dependencies and packages if needed.

## Requirements
Requires Qt5 and ROS2 to be installed

## Installation
1. Download the folders in the src folder of a ROS2 workspace
2. Build using ```colcon build``` from your workspace
3. Source using ```source install/setup.bash```

## Run
```ros2 run <package_name> <executable_name>```
