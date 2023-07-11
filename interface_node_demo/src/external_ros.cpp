#include "external_ros.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interface_demo_messages/srv/discrete_cmd.hpp>
#include <interface_demo_messages/srv/discrete_fdb.hpp>

using namespace std::chrono_literals;

ExternalROSNode::ExternalROSNode() : Node("external_ros") {
  // Feedback topic (ex : /odometry)
  continuousFdbSubscription_ = this->create_subscription<std_msgs::msg::String>(
    "fdb_topic", 
    10, 
    std::bind(&ExternalROSNode::ContinuousFdbTopicCallback, this, std::placeholders::_1));

  // Command topic (ex : /cmd_vel)
  continuousCmdPublisher_ = this->create_publisher<std_msgs::msg::String>(
    "cmd_topic", 
    10);
  // Emulation of command publisher through timers
  continuousCmdTimer_ = this->create_wall_timer(
    100ms, 
    std::bind(&ExternalROSNode::ContinuousCmdTimerCallback, this));
  continuousCmdCount_ = 0;

  // Feedback service (ex : /infos)
  discreteFdbClient_ = this->create_client<interface_demo_messages::srv::DiscreteFdb>("fdb_service");
  // Emulation of feedback call through timers
  discreteFdbTimerCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  discreteFdbTimer_ = this->create_wall_timer(
    2000ms, 
    std::bind(&ExternalROSNode::DiscreteFdbTimerCallback, this), discreteFdbTimerCallbackGroup_);

  // Command service (ex : /cmd_pos)
  discreteCmdClient_ = this->create_client<interface_demo_messages::srv::DiscreteCmd>("cmd_service");
  // Emulation of command call through timers
  discreteCmdTimerCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  discreteCmdTimer_ = this->create_wall_timer(
    1000ms, 
    std::bind(&ExternalROSNode::DiscreteCmdTimerCallback, this), discreteCmdTimerCallbackGroup_);
  discreteCmdCount_ = 0;
}
// Feedback topic (ex : /odometry)
void ExternalROSNode::ContinuousFdbTopicCallback(const std_msgs::msg::String& msg) {
    RCLCPP_INFO(this->get_logger(), "[ExternalROSNode] Received continuous feedback : '%s'", msg.data.c_str());
}
// Command topic (ex : /cmd_vel)
// Emulation of command publisher through timers
void ExternalROSNode::ContinuousCmdTimerCallback() {
  auto msg = std_msgs::msg::String();
  msg.data = "C_Cmd " + std::to_string(continuousCmdCount_);
  std::cout << std::endl;
  RCLCPP_INFO(this->get_logger(), "[ExternalROSNode] Sending continuous command : '%s'", msg.data.c_str());
  continuousCmdPublisher_->publish(msg);
  ++continuousCmdCount_;
}
// Feedback service (ex : /infos)
// Emulation of feedback call through timers
void ExternalROSNode::DiscreteFdbTimerCallback() {
  auto request = std::make_shared<interface_demo_messages::srv::DiscreteFdb::Request>();
  auto response = discreteFdbClient_->async_send_request(request);
  std::future_status status = response.wait_for(5s);
  if (status == std::future_status::ready) {
    std_msgs::msg::String msg = response.get()->data;
    RCLCPP_INFO(this->get_logger(), "[ExternalROSNode] Received discrete feedback : '%s'", msg.data.c_str());
  }
}
// Command service (ex : /cmd_pos)
// Emulation of feedback call through timers
void ExternalROSNode::DiscreteCmdTimerCallback() {
  auto msg = std_msgs::msg::String();
  msg.data = "D_Cmd " + std::to_string(discreteCmdCount_);
  auto request = std::make_shared<interface_demo_messages::srv::DiscreteCmd::Request>();
  request->data = msg;
  auto response = discreteCmdClient_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "[ExternalROSNode] Sending discrete command : '%s'", msg.data.c_str());
  ++discreteCmdCount_;
}