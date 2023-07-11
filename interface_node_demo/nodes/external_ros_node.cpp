#include <rclcpp/rclcpp.hpp>

#include "external_ros.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ExternalROSNode> external_ros = std::make_shared<ExternalROSNode>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(external_ros);

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
