#ifndef EXTERNAL_QT_H
#define EXTERNAL_QT_H

#include <QObject>

class ExternalQtObject : public QObject { //Mirror QObject : Sends as feedback the command just received

    Q_OBJECT

    public:
        ExternalQtObject();

    signals:
        void fdbChanged(std::string msg);

    public slots:
        void cmdSlot(std::string msg);
        // void cmdSlot();

};

#endif