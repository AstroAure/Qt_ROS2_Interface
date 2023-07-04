#include "external_qt.h"

#include <iostream>

ExternalQtObject::ExternalQtObject()
    : QObject(nullptr) 
{
}

void ExternalQtObject::cmdSlot(std::string msg) {
    std::cout << "[ExternalQtObject] Received Qt command : " << msg << std::endl;
    std::cout << "[ExternalQtObject] Sending Qt feedback : " << msg << std::endl;
    emit fdbChanged(msg); //Feedback is mirror of command
}

// void ExternalQtObject::cmdSlot() {
//     std::cout << "[ExternalQtObject] Timer received !" << std::endl;
//     emit fdbChanged("Timer");
// }