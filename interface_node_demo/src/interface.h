#ifndef INTERFACE_H
#define INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <QObject>
#include <std_msgs/msg/string.hpp>
#include <interface_demo_messages/srv/discrete_cmd.hpp>
#include <interface_demo_messages/srv/discrete_fdb.hpp>

class Interface : public QObject, public rclcpp::Node {
  Q_OBJECT

 public:
  Interface();
  
 signals:
  // Signal emitted for data translated from topics (ex : /cmd_vel)
  void ContinuousCmdChanged(std::string msg);
  // Signal emitted for data translated from services (ex : /cmd_pos)
  void DiscreteCmdChanged(std::string msg);
 public slots:
  // Slot to receive signal to translate to topic (ex : /odometry)
  void ContinousFdbSlot(std::string msg);
  // Slot to receive signal to translate to service (ex : /infos)
  void DiscreteFdbSlot(std::string msg);

 private:
  // Feedback topic (ex : /odometry)
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr continuousFdbPublisher_;

  // Command topic (ex : /cmd_vel)
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr continuousCmdSubscriber_;
  void ContinousCmdTopicCallback(const std_msgs::msg::String& msg);

  // Feedack service (ex : /infos)
  std_msgs::msg::String discreteFdbData_;
  rclcpp::Service<interface_demo_messages::srv::DiscreteFdb>::SharedPtr discreteFdbService_;
  void DiscreteFdbServiceCallback(
    const std::shared_ptr<interface_demo_messages::srv::DiscreteFdb::Request> request,
    std::shared_ptr<interface_demo_messages::srv::DiscreteFdb::Response> response);

  // Command service (ex : /cmd_pos)
  rclcpp::Service<interface_demo_messages::srv::DiscreteCmd>::SharedPtr discreteCmdService_;
  void DiscreteCmdServiceCallback(
    const std::shared_ptr<interface_demo_messages::srv::DiscreteCmd::Request> request,
    std::shared_ptr<interface_demo_messages::srv::DiscreteCmd::Response> response);
  
  // Data Qt-ROS translation functions
  std_msgs::msg::String stdString2rosString (std::string str);
  std::string rosString2stdString (std_msgs::msg::String str);
};

#endif // INTERFACE_H
