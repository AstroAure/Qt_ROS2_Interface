#include <QCoreApplication>
#include <QMetaType>
#include <thread>

#include "interface.h"
#include "external_qt.h"

void rosSpinThread(rclcpp::Node::SharedPtr node) {
  rclcpp::spin(node);
  rclcpp::shutdown();
}

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    QCoreApplication app(argc, argv);
    qRegisterMetaType<std::string>("std::string");

    std::shared_ptr<Interface> interface_node = std::make_shared<Interface>();

    std::thread ros_spin_thread(std::bind(rosSpinThread, interface_node));

    ExternalQtObject* qt_mirror = new ExternalQtObject();
    QObject::connect(qt_mirror, &ExternalQtObject::fdbChanged, interface_node.get(), &Interface::fdbSlot);
    QObject::connect(interface_node.get(), &Interface::cmdChanged, qt_mirror, &ExternalQtObject::cmdSlot);

    return app.exec();

}
