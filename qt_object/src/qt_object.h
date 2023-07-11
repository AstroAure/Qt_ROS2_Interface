#ifndef QT_OBJECT_H
#define QT_OBJECT_H

#include <QObject>

class QtObject : public QObject
{
    Q_OBJECT

public:
    QtObject();

signals:
    void signalReceived(); //Signal emitted by QtObject when onSignalReceived is called

public slots:
    void onSignalReceived(); //Slot to receive the timeout signal of the QTimer
};

#endif