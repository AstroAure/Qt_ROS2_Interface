#ifndef EXTERNAL_ROS_H
#define EXTERNAL_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interface_demo_messages/srv/discrete_cmd.hpp>
#include <interface_demo_messages/srv/discrete_fdb.hpp>

class ExternalROSNode : public rclcpp::Node {
 public:
  ExternalROSNode();

 private:
  // Feedback topic (ex : /odometry)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr continuousFdbSubscription_;
  void ContinuousFdbTopicCallback(const std_msgs::msg::String& msg);

  // Command topic (ex : /cmd_vel)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr continuousCmdPublisher_;
  // Emulation of command publisher through timers
  rclcpp::TimerBase::SharedPtr continuousCmdTimer_;
  int continuousCmdCount_;
  void ContinuousCmdTimerCallback();

  // Feedback service (ex : /infos)
  rclcpp::Client<interface_demo_messages::srv::DiscreteFdb>::SharedPtr discreteFdbClient_;
  // Emulation of feedback call through timers
  rclcpp::CallbackGroup::SharedPtr discreteFdbTimerCallbackGroup_;
  rclcpp::TimerBase::SharedPtr discreteFdbTimer_;
  void DiscreteFdbTimerCallback();
  
  // Command service (ex : /cmd_pos)
  rclcpp::Client<interface_demo_messages::srv::DiscreteCmd>::SharedPtr discreteCmdClient_;
  // Emulation of command call through timers
  rclcpp::CallbackGroup::SharedPtr discreteCmdTimerCallbackGroup_;
  rclcpp::TimerBase::SharedPtr discreteCmdTimer_;
  int discreteCmdCount_;
  void DiscreteCmdTimerCallback();

};

#endif // EXTERNAL_ROS_H
