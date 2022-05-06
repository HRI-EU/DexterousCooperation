/*******************************************************************************

  Copyright (c) by Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef RCS_MONITORWIDGETBASE_H
#define RCS_MONITORWIDGETBASE_H



#include <QtGlobal>
#if QT_VERSION >= 0x050000
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSlider>
#else
#include <QtGui/QScrollArea>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QCheckBox>
#include <QtGui/QLabel>
#include <QtGui/QLCDNumber>
#include <QtGui/QLineEdit>
#include <QtGui/QSlider>
#endif

#include <vector>
#include <qtimer.h>

#include <map>
#include <pthread.h>

namespace Rcs
{

class EntityBase;
class MonitorRequest;
class MonitorRequestWidget;
class SensorSignalWidget;

/*!
 *  \brief MonitorWidgetBase provides the QWidget GUI for monitoring and manipulating all sensor cues and
 *         MonitorRequests. For each sensor cue the current value is visualized and can also be overwritten
 *         manually. All current MonitorRequest are displayed including details on the involved conditions.
 *         The can be triggered manually from the GUI.
 *         In addition, a few buttons provide access to basic functionality:
 *         - Confirmations can be set to automatically confirm. This is very useful for debugging and simulation.
 *         - Auto evaluate: if disabled, no MonitorRequests will trigger even if their conditions are all
 *           fulfilled. Useful for debugging but also for pausing the system.
 *         Additional functionality is provided by the GUI elements mentioned below.
 *
 *         The class publishes the following events:
 *         - SetTTC: GUI elements -- set the base time to completion (TTC).
 *         - MoveToStartPose: GUI button -- send the robot to a start pose.
 *         - Tare: GUI button -- tare the sensors (e.g. force torque sensors).
 *         - OpenHands: GUI button
 *         - CloseHands: GUI button
 *         - Engage: GUI button
 *         - Disengage: GUI button
 *         - StopActions: GUI button
 *         - RecoverToClosestState: GUI button
 *
 *         The class does not subscribe to any events, but rather relies on updates through a function call.
 */
class MonitorWidgetBase : public QWidget
{
  Q_OBJECT

public:
  /*!
   *  \brief MonitorWidgetBase provides the QWidget GUI for monitoring and manipulating all sensor cues and
   *         MonitorRequest. For each sensor cue the current value is visualized and can also be overwritten
   *         manually. All current MonitorRequest are displayed including details on the involved conditions.
   *         The can be triggered manually from the GUI.
   *         In addition, a few buttons provide access to basic functionality:
   *         - Confirmations can be set to automatically confirm. This is very useful for debugging and simulation.
   *         - Auto evaluate: if disabled, no MonitorRequests will trigger even if their conditions are all
   *           fulfilled. Useful for debugging but also for pausing the system.
   *         Additional functionality is provided by GUI elements.
   */
  MonitorWidgetBase(EntityBase* parent, bool enableSensorEdit = false, bool autoPublish = true, bool autoConfirm_in = false);
  ~MonitorWidgetBase();

  /*!
   *  \brief Update all GUI elements for sensor signals with new sensor reading. If they are marked as GUI-driven
   *         then read back the value set in the GUI instead and use it to overwrite the real sensor value.
   *  \param[in] sig      Hashmap with pairs of sensor signal name and value
   */
  void syncSensorSignals(std::map<std::string, double>& sig);

  /*!
   *  \brief Updates list of MonitorRequestWidget in the GUI to add new and remove old MonitorRequest.
   *
   *  \param[in] addReq         List of MonitorRequest to add
   *  \param[in] removeReq      List of MonitorRequest to remove
   */
  void updateRequests(std::vector<MonitorRequest> addReq, std::vector<MonitorRequest> removeReq);

  /*!
   *  \brief Returns true if fulfilled MonitorRequests can be published for the next single evaluation.
   *
   *  \returns        Boolean
   */
  bool getPublishOnce();

  /*!
   *  \brief Returns true if fulfilled MonitorRequest can be published automatically.
   *
   *  \returns        Boolean
   */
  bool getAutoPublish();

  /*!
   *  \brief Returns true if MonitorRequest with topic 'confirmation' should be published automatically.
   *
   *  \returns        Boolean
   */
  bool getAutoAcceptConfirmation();

  /*!
   *  \brief Helper function to create the widget.
   */
  static void* monitorGuiBase(void* arg);

  /*!
   *  \brief Helper function to create the widget.
   */
  static int create(EntityBase* entityBase, bool enableSensorEdit, bool autoPublish, bool autoConfirm);


private slots:
  void displayAct();
  void evaluateRequests();
  void onAutoEvaluateCheckBoxChanged(int state);
  void onAutoAcceptConfirmationCheckBoxChanged(int state);
  void goToStart();
  void tareSensors();
  void openHands();
  void closeHands();
  void engageObject();
  void disengageObject();
  void stopMovement();
  void recoverToState();
  void onSliderTtcChanged(int value);
  void onValueBoxTtcEditingFinished();
  void onSetTtc();

private:

  void lock();
  void unlock();

  void onSetTtcEvent(double ttcIn);

  QFrame* createLine(bool horizontal = true);
  EntityBase* entity;

  QTimer* _timer;
  pthread_mutex_t mutex;

  std::map<std::string, double> sensorSignals;
  std::vector<std::pair<std::string, double> > signalUpdates;
  std::map<std::string, QLabel*> sensorLabels;
  std::map<std::string, SensorSignalWidget*> sensorSignalWidgets;
  std::vector<MonitorRequest> requests;

  QCheckBox* autoEvaluateCheckbox;
  QPushButton* evaluateButton;

  QPushButton* m_button;
  QLineEdit* valueBoxTTC;
  QLabel* labelCurrentTTC;
  QSlider* sliderTTC;
  double ttc;
  double ttcNew;

  QGroupBox* gbox_monitorRequests;
  QBoxLayout* requestLayout;
  QGroupBox* gbox_monitorIntentions;
  QBoxLayout* intentionLayout;
  QGroupBox* gbox_monitorConfirmation;
  QBoxLayout* confirmationLayout;
  QGroupBox* gbox_monitorProgress;
  QBoxLayout* progressLayout;
  QVBoxLayout* sensorLayout;
  std::vector<QPushButton*> requestButtons;
  std::vector<MonitorRequest> addRequests;
  std::vector<MonitorRequest> removeRequests;
  std::map<std::string, MonitorRequestWidget*> requestWidgets;

  bool publishOnce;
  bool autoPublish;
  bool autoConfirm;
  bool sensorEditEnabled;
};

}

#endif //RCS_MONITORWIDGETBASE_H
