#include "interface.h"

Interface::Interface() : QObject(nullptr), Node("ros_qt_interface") {
    fdb_publisher = this->create_publisher<std_msgs::msg::String>("fdb_topic", 10);
    cmd_subscriber = this->create_subscription<std_msgs::msg::String>("cmd_topic", 10, std::bind(&Interface::topic_cmd_callback, this, std::placeholders::_1));
}

void Interface::fdbSlot(std::string msg) {
    RCLCPP_INFO(this->get_logger(), "[Interface] Received Qt feedback : '%s'", msg.c_str());
    fdb_publisher->publish(String2rosString(msg));
    RCLCPP_INFO(this->get_logger(), "[Interface] Sending ROS2 feedback : '%s'", msg.c_str());
}

void Interface::topic_cmd_callback(const std_msgs::msg::String& ros_msg) {
    std::string msg = rosString2String(ros_msg);
    std::cout << std::endl;
    RCLCPP_INFO(this->get_logger(), "[Interface] Received ROS2 command : '%s'", msg.c_str());
    emit cmdChanged(msg);
    RCLCPP_INFO(this->get_logger(), "[Interface] Sending Qt command : '%s'", msg.c_str());
}

std_msgs::msg::String Interface::String2rosString (std::string str) {
    std_msgs::msg::String ros_str;
    ros_str.data = str;
    return ros_str;
}

std::string Interface::rosString2String (std_msgs::msg::String ros_str) {
    return ros_str.data;
}