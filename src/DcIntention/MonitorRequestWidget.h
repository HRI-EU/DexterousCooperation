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

#ifndef DC_MONITORREQUESTWIDGET_H
#define DC_MONITORREQUESTWIDGET_H

#include "MonitorRequest.h"

#include <QGroupBox>
#include <QLineEdit>
#include <QLabel>
#include <QSlider>
#include <QCheckBox>
#include <QPushButton>
#include <QProgressBar>

#include <pthread.h>


namespace Dc
{

class MonitorRequest;
class EntityBase;


/*!
 *  \brief MonitorRequestWidget provides a GUI widget to represent and interact with a MonitorRequest.
 *         The associated conditions are shown and the request can be triggered manually.
 */
class MonitorRequestWidget : public QGroupBox//Widget
{
  Q_OBJECT

public:

  /*!
   *  \brief MonitorRequestWidget provides a GUI widget to represent and interact with a MonitorRequest.
   *         The associated conditions are shown and the request can be triggered manually.
   */
  MonitorRequestWidget(EntityBase* parent, MonitorRequest req);
  ~MonitorRequestWidget();

  /*!
   *  \brief Update all conditions and the activation time based on the provided sensor signals.
   *
   *  \param[in] sig        hashmap with sensor signal name and value
   *  \param[in] time       current time
   */
  void update(std::map<std::string, double> sig, double time);

private slots:
  void click();

private:
  EntityBase* entity;
  MonitorRequest request;
  std::vector<QLabel*> signalLabels;
  QLabel* durationLabel;
  QPushButton* confirmRequestButton;
  double lastTimeNotFulfilled;
  QLabel* startStopLabel;
  QString groupboxDefaultStyle = "";
  QProgressBar* progress;
};

/*!
 *  \brief SensorSignalWidget provides a GUI widget to represent and interact with a sensor signal.
 *         The current value is shown, but can also be overwritten.
 */
class SensorSignalWidget : public QWidget //QGroupBox
{
  Q_OBJECT

public:
  /*!
   *  \brief SensorSignalWidget provides a GUI widget to represent and interact with a sensor signal.
   *         The current value is shown, but can also be overwritten.
   */
  SensorSignalWidget(std::string id, double value, bool enableEdit, double minVal = -50.0, double maxVal = 50.0);
  ~SensorSignalWidget();

  /*!
   *  \brief Update sensor value if not overwritten.
   *
   *  \param[in] id        id of sensor signal
   *  \param[in] value     sensor value
   */
  void update(std::string id, double value);

  /*!
   *  \brief Returns the sensor value shown in the GUI.
   *
   *  \returns             sensor value
   */
  double getValue();

  /*!
   *  \brief Returns the sensor value is being overwritten in the GUI.
   *
   *  \returns             Boolean, true if overwritten in GUI
   */
  bool editEnabled();

private slots:
  void onSliderChanged(int value);
  void onCheckBoxStateChanged(int state);
  void onValueBoxEditingFinished();

private:

  void lock();
  void unlock();

  double sensorValue;
  bool sensorEditEnabled;

  pthread_mutex_t mutex;
  QCheckBox* enableEditCheckBox;
  QLineEdit* valueBox;
  QLabel* nameLabel;
  QSlider* slider;
};

}

#endif // DC_MONITORREQUESTWIDGET_H
