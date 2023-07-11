#include "interface.h"

#include <rclcpp/rclcpp.hpp>
#include <QObject>
#include <std_msgs/msg/string.hpp>
#include <interface_demo_messages/srv/discrete_cmd.hpp>
#include <interface_demo_messages/srv/discrete_fdb.hpp>

Interface::Interface() : QObject(nullptr), Node("interface") {
  // Feedback topic (ex : /odometry)
  continuousFdbPublisher_ = this->create_publisher<std_msgs::msg::String>(
    "fdb_topic", 
    10);
  // Command topic (ex : /cmd_vel)
  continuousCmdSubscriber_ = this->create_subscription<std_msgs::msg::String>(
    "cmd_topic", 
    10, 
    std::bind(&Interface::ContinousCmdTopicCallback, this, std::placeholders::_1));
  // Feedback service (ex : /infos)
  discreteFdbService_ = this->create_service<interface_demo_messages::srv::DiscreteFdb>(
    "fdb_service", 
    std::bind(&Interface::DiscreteFdbServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
  // Command service (ex : /cmd_pos)
  discreteCmdService_ = this->create_service<interface_demo_messages::srv::DiscreteCmd>(
    "cmd_service", 
    std::bind(&Interface::DiscreteCmdServiceCallback, this, std::placeholders::_1, std::placeholders::_2));
}
// Feedback topic (ex : /odometry)
void Interface::ContinousFdbSlot(std::string msg) {
    // RCLCPP_INFO(this->get_logger(), "[Interface] Received continuous feedback : '%s'", msg.c_str());
    continuousFdbPublisher_->publish(stdString2rosString(msg));
}
// Command topic (ex : /cmd_vel)
void Interface::ContinousCmdTopicCallback(const std_msgs::msg::String& ros_msg) {
    std::string msg = rosString2stdString(ros_msg);
    // std::cout << std::endl;
    // RCLCPP_INFO(this->get_logger(), "[Interface] Received contiunous command : '%s'", msg.c_str());
    emit ContinuousCmdChanged(msg);
}
// Feedback service (ex : /infos)
void Interface::DiscreteFdbSlot(std::string msg) {
    // RCLCPP_INFO(this->get_logger(), "[Interface] Received discrete feedback : '%s'", msg.c_str());
    discreteFdbData_ = stdString2rosString(msg);
}
void Interface::DiscreteFdbServiceCallback(
    const std::shared_ptr<interface_demo_messages::srv::DiscreteFdb::Request> request,
    std::shared_ptr<interface_demo_messages::srv::DiscreteFdb::Response> response) {
  response->data = discreteFdbData_;
}
// Command service (ex : /cmd_pos)
void Interface::DiscreteCmdServiceCallback(
    const std::shared_ptr<interface_demo_messages::srv::DiscreteCmd::Request> request,
    std::shared_ptr<interface_demo_messages::srv::DiscreteCmd::Response> response) {
  std::string msg = rosString2stdString(request->data);
  // std::cout << std::endl;
  // RCLCPP_INFO(this->get_logger(), "[Interface] Received discrete command : '%s'", msg.c_str());
  emit DiscreteCmdChanged(msg);
}

//Data Qt-ROS translation functions
std_msgs::msg::String Interface::stdString2rosString (std::string str) {
  std_msgs::msg::String ros_str;
  ros_str.data = str;
  return ros_str;
}
std::string Interface::rosString2stdString (std_msgs::msg::String ros_str) {
  return ros_str.data;
}