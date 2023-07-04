#ifndef INTERFACE_H
#define INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <QObject>

#include <std_msgs/msg/string.hpp>

class Interface : public QObject, public rclcpp::Node {

     Q_OBJECT

    public:
        Interface();

    signals:
        void cmdChanged(std::string msg);

    public slots:
        void fdbSlot(std::string msg);

    private:
        
        std_msgs::msg::String String2rosString (std::string str);
        std::string rosString2String (std_msgs::msg::String str);

        void topic_cmd_callback(const std_msgs::msg::String& msg);

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fdb_publisher;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscriber;

};

#endif
