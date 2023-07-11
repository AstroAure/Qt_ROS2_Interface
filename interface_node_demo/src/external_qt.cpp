#include "external_qt.h"

#include <iostream>

#include <QObject>
#include <QTimer>

ExternalQtObject::ExternalQtObject() : QObject(nullptr) {
  // Feedback topic (ex : /odometry)
  continuousFdbTimer_ = new QTimer();
  continuousFdbCount_ = 0;
  connect(continuousFdbTimer_, &QTimer::timeout, this, &ExternalQtObject::ContinuousFdbTriggerSignal);
  continuousFdbTimer_->start(100);
  // Feedback service (ex : /infos)
  discreteFdbTimer_ = new QTimer();
  discreteFdbCount_ = 0;
  connect(discreteFdbTimer_, &QTimer::timeout, this, &ExternalQtObject::DiscreteFdbTriggerSignal);
  discreteFdbTimer_->start(1000);
}
// Command topic (ex : /cmd_vel)
void ExternalQtObject::ContinuousCmdSlot(std::string msg) {
  std::cout << "[ExternalQtObject] Received continuous command : " << msg << std::endl;
}
// Command service (ex : /cmd_pos)
void ExternalQtObject::DiscreteCmdSlot(std::string msg) {
  std::cout << "[ExternalQtObject] Received discrete command : " << msg << std::endl;
}
// Emulation of feedback signals through timers
void ExternalQtObject::ContinuousFdbTriggerSignal() {
  std::string msg = "C_Fdb " + std::to_string(continuousFdbCount_);
  std::cout << std::endl;
  std::cout << "[ExternalQtObject] Sending continuous feedback : " << msg << std::endl;
  emit ContinuousFdbChanged(msg);
  ++continuousFdbCount_;
}
void ExternalQtObject::DiscreteFdbTriggerSignal() {
  std::string msg = "D_Fdb " + std::to_string(discreteFdbCount_);
  std::cout << "[ExternalQtObject] Sending discrete feedback : " << msg << std::endl;
  emit DiscreteFdbChanged(msg);
  ++discreteFdbCount_;
}