#include <QCoreApplication>
#include <QMetaType>
#include <thread>
#include <rclcpp/rclcpp.hpp>

#include "interface.h"
#include "external_qt.h"

void rosSpinThread(rclcpp::Node::SharedPtr node) {
  rclcpp::spin(node);
  rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
  // Initialization
  rclcpp::init(argc, argv);
  QCoreApplication app(argc, argv);

  // Defining Qt signals messages types
  qRegisterMetaType<std::string>("std::string");

  //Qt ROS2 Interface
  std::shared_ptr<Interface> interface_node = std::make_shared<Interface>();
  std::thread ros_spin_thread(std::bind(rosSpinThread, interface_node));

  //External Qt Object
  ExternalQtObject* external_qt = new ExternalQtObject();

  // Qt signals connections
  // Feedback topic (ex : /odometry)
  QObject::connect(external_qt, &ExternalQtObject::ContinuousFdbChanged, 
                   interface_node.get(), &Interface::ContinousFdbSlot);
  // Command topic (ex : /cmd_vel)
  QObject::connect(interface_node.get(), &Interface::ContinuousCmdChanged, 
                   external_qt, &ExternalQtObject::ContinuousCmdSlot);
  // Feedback service (ex : /infos)
  QObject::connect(external_qt, &ExternalQtObject::DiscreteFdbChanged, 
                   interface_node.get(), &Interface::DiscreteFdbSlot);
  // Command service (ex : /cmd_pos)
  QObject::connect(interface_node.get(), &Interface::DiscreteCmdChanged, 
                   external_qt, &ExternalQtObject::DiscreteCmdSlot);

  return app.exec();
}
