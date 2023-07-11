#ifndef EXTERNAL_QT_H
#define EXTERNAL_QT_H

#include <QObject>
#include <QTimer>

class ExternalQtObject : public QObject {
  Q_OBJECT

 public:
  ExternalQtObject();

 signals:
  // Signal emitted for data to be translated to topics (ex : /odometry)
  void ContinuousFdbChanged(std::string msg);
  // Signal emitted for data to be translated to services (ex : /infos)
  void DiscreteFdbChanged(std::string msg);

 public slots:
  // Slot to receive signal translated from topic (ex : /cmd_vel)
  void ContinuousCmdSlot(std::string msg);
  // Slot to receive signal translated from service (ex : /cmd_pos)
  void DiscreteCmdSlot(std::string msg);
  // Emulation of feedback signals through timers
  void ContinuousFdbTriggerSignal();
  void DiscreteFdbTriggerSignal();
  
 private:
  // Emulation of feedback signals through timers
  QTimer* continuousFdbTimer_;
  int continuousFdbCount_;
  QTimer* discreteFdbTimer_;
  int discreteFdbCount_;
};

#endif // EXTERNAL_QT_H