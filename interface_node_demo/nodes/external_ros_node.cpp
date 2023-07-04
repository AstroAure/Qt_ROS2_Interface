#include "external_ros.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ExternalROSNode> external_node = std::make_shared<ExternalROSNode>();

  rclcpp::spin(external_node);
  rclcpp::shutdown();

  return 0;
}
