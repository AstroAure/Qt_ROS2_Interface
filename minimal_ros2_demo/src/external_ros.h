#ifndef EXTERNAL_ROS_H
#define EXTERNAL_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ExternalROSNode : public rclcpp::Node {

  public:
    ExternalROSNode();

  private:

    void topic_callback(const std_msgs::msg::String& msg);

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

#endif // EXTERNAL_ROS_H
