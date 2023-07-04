#include "external_ros.h"

using namespace std::chrono_literals;

ExternalROSNode::ExternalROSNode() : Node("mirror_ros_node") {
  publisher_ = this->create_publisher<std_msgs::msg::String>("fdb_topic", 10);
  subscription_ = this->create_subscription<std_msgs::msg::String>("cmd_topic", 10, std::bind(&ExternalROSNode::topic_callback, this, std::placeholders::_1));
}

void ExternalROSNode::topic_callback(const std_msgs::msg::String& msg) {
  RCLCPP_INFO(this->get_logger(), "Received command / Sending feedback : '%s'", msg.data.c_str());
  publisher_->publish(msg);
}
