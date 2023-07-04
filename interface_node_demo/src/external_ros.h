#ifndef EXTERNAL_ROS_H
#define EXTERNAL_ROS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ExternalROSNode : public rclcpp::Node {

  public:
    ExternalROSNode();

  private:

    void timer_callback();

    void topic_callback(const std_msgs::msg::String& msg);

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;

};

#endif // EXTERNAL_ROS_H
