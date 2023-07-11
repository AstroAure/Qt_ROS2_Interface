#include "qt_object.h"

#include <iostream>

QtObject::QtObject()
    : QObject(nullptr)
{
}

void QtObject::onSignalReceived()
{
    std::cout << "Signal received !" << std::endl;
    emit signalReceived();
}