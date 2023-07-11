#include <QCoreApplication>

#include <QTimer>
#include <memory>

#include "qt_object.h"

int main(int argc, char* argv[])
{

    QCoreApplication app(argc, argv);

    std::shared_ptr<QtObject> qt_object = std::make_shared<QtObject>(); //QtObject* qt_object = new QtObject(); would also work

    //Definition of a QTimer and connection to the QtObject
    QTimer* timer = new QTimer();
    QObject::connect(timer, &QTimer::timeout, qt_object.get(), &QtObject::onSignalReceived); //qt_object.get() returns the adress of the qt_object shared_ptr
    timer->start(500);

    return app.exec();
}