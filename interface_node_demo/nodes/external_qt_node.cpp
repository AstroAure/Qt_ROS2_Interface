// Not used ! Requires modification in external_qt. and external_qt.cpp to replace void cmdSlot(std::string msg) with void cmdSlot()

#include <QCoreApplication>
#include <QTimer>
#include <memory>
#include "external_qt.h"

int main(int argc, char**argv)
{

    QCoreApplication app(argc, argv);

    std::shared_ptr<ExternalQtObject> external_qt = std::make_shared<ExternalQtObject>();

    //Definition of a QTimer and connection to the ExternalQtObject
    QTimer* timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, external_qt.get(), &ExternalQtObject::cmdSlot); //qt_object.get() returns the adress of the qt_object shared_ptr
    timer->start(1000);

    return app.exec();
}