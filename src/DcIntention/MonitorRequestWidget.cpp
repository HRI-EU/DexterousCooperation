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

#include "MonitorRequestWidget.h"
#include "EntityBase.h"

#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QDoubleValidator>



namespace Rcs
{
MonitorRequestWidget::MonitorRequestWidget(EntityBase* parent, MonitorRequest req) : QGroupBox(), //QWidget(),
  entity(parent), request(req), durationLabel(NULL)
{
  //  request.activationDurationThreshold = 5.0; //just for debugging
  setMaximumWidth(320);

  double currentTime = entity->getTime();

  lastTimeNotFulfilled = currentTime;//Timer_getSystemTime();

  // Box for the _dim task lines
  QVBoxLayout* mainGrid = new QVBoxLayout();
  mainGrid->setMargin(5);
  mainGrid->setSpacing(5);

  setLayout(mainGrid);

  groupboxDefaultStyle = "";
  groupboxDefaultStyle.append("QGroupBox { "
                              "    font-weight: bold; "
                              "    margin-top: 0.5em; "
                              "    border-radius: 10; "
                              "    border: 1px solid black;"
                              "} ");

  groupboxDefaultStyle.append("QGroupBox::title { "
                              "    subcontrol-origin: margin; "
                              //"    subcontrol-position: top left; /* position at the top center */"
                              "    padding: 5 0px; "
                              "    left: 5px; "
                              //"    top: 10px; "
                              "} ");

  setStyleSheet(groupboxDefaultStyle);

  confirmRequestButton = new QPushButton((std::to_string(request.id) + ": " + request.description).c_str());
  confirmRequestButton->setToolTip((" (id:" + std::to_string(request.id) + ",seq:" + std::to_string(request.sequenceNum) + ")").c_str());
  mainGrid->addWidget(confirmRequestButton);

  connect(confirmRequestButton, SIGNAL(clicked()), this, SLOT(click()));


  //  RLOG(0, "Activation Duration: %5.2f sec", req.activationDurationThreshold);
  if (request.activationDurationThreshold > 0.0)
  {
    char txt[100];
    snprintf(txt, 100, "Activation Duration: %5.2f / %5.2f sec", request.activationDuration, request.activationDurationThreshold);

    durationLabel = new QLabel(txt);
    mainGrid->addWidget(durationLabel);
  }

  for (size_t i = 0; i < request.sensorSignals.size(); i++)
  {
    char txt[100];
    snprintf(txt, 100,
             "%s %s % 5.2f",
             request.sensorSignals[i].signal.c_str(),
             (request.sensorSignals[i].lesserThan ? "<" : ">"),
             request.sensorSignals[i].threshold);
    //    txt.append(req.sensorSignals[i].signal + ": " + (req.sensorSignals[i].lesserThan ? "< ":"> ") + std::to_string(req.sensorSignals[i].threshold));

    QLabel* label = new QLabel(txt);
    signalLabels.push_back(label);

    mainGrid->addWidget(label);
  }

  progress = new QProgressBar();
  progress->setFixedHeight(15);
  progress->setValue(0);
  mainGrid->addWidget(progress);


  if (request.maxTime != 0.0 || request.maxTime != 0.0)
  {
    double timeSinceCreation = (currentTime - request.creationTime);
    if (fabs(request.creationTime) < 0.1)
    {
      //TODO: just a fix for now
      timeSinceCreation = 0.0;
    }

    char txt[100];
    snprintf(txt, 100,
             "Time: %5.2f < %5.2f < %5.2f",
             request.minTime, timeSinceCreation, request.maxTime);
    startStopLabel = new QLabel(txt);
    mainGrid->addWidget(startStopLabel);
  }
  else
  {
    startStopLabel = NULL;
  }
}

MonitorRequestWidget::~MonitorRequestWidget()
{

}

void MonitorRequestWidget::click()
{
  entity->publish<int, int>(request.topic, request.id, request.sequenceNum);
}

void MonitorRequestWidget::update(std::map<std::string, double> sig, double time)
{
  bool fulfilled = true;
  double currentTime = entity->getTime();

  double activeTotal = 0.0;

  for (size_t i = 0; i < request.sensorSignals.size(); i++)
  {
    double active = request.evaluate((int) i, sig);
    activeTotal += active;

    //    RLOG(0, "Update %s: %d %d", request.topic.c_str(), (int) i, active);

    if (active >= 1.0)
    {
      signalLabels[i]->setStyleSheet("QLabel { color: green}");
    }
    else
    {
      signalLabels[i]->setStyleSheet("QLabel { color: red}");
      fulfilled = false;
    }
  }
  int percentComplete = activeTotal * 100.0 / request.sensorSignals.size();

  percentComplete = std::max(percentComplete, 0);
  percentComplete = std::min(percentComplete, 100);

  progress->setValue(percentComplete);

  if (startStopLabel != NULL)
  {
    double timeSinceCreation = (currentTime - request.creationTime);
    if (fabs(request.creationTime) < 0.1)
    {
      //TODO: just a fix for now
      timeSinceCreation = 0.0;
    }

    if ((timeSinceCreation < request.minTime) || ((timeSinceCreation > request.maxTime) && request.maxTime!=0.0))
    {
      startStopLabel->setStyleSheet("QLabel { color: red}");
      confirmRequestButton->setEnabled(false);
    }
    else
    {
      startStopLabel->setStyleSheet("QLabel { color: green}");
      confirmRequestButton->setEnabled(true);
    }

    char txt[100];
    snprintf(txt, 100,
             "Time: %5.2f < %5.2f < %5.2f",
             request.minTime, timeSinceCreation, request.maxTime);
    startStopLabel->setText(txt);

  }

  if (request.activationDurationThreshold > 0.0 && durationLabel != NULL)
  {
    if (fulfilled)
    {
      request.activationDuration = fmin(
                                     currentTime - lastTimeNotFulfilled,
                                     request.activationDurationThreshold
                                   );
      //      request.activationDuration = fmin(
      //          getEntity()->getTime() - lastTimeNotFulfilled,
      //          request.activationDurationThreshold
      //      );

    }
    else
    {
      request.activationDuration = 0.0;
      lastTimeNotFulfilled = currentTime;
    }

    if (request.activationDurationThreshold == request.activationDuration)
    {
      durationLabel->setStyleSheet("QLabel { color: green}");
    }
    else if (request.activationDuration == 0.0)
    {
      durationLabel->setStyleSheet("QLabel { color: black}");
      fulfilled = false;
    }
    else
    {
      durationLabel->setStyleSheet("QLabel { color: orange}");
      fulfilled = false;
    }

    char txt[100];
    snprintf(txt, 100, "Activation Duration: %5.2f / %5.2f sec", request.activationDuration, request.activationDurationThreshold);
    durationLabel->setText(txt);

  }

  if (fulfilled)
  {
    //    RLOG(0, "Fulfilled: %s", request.toString().c_str());
    QString gStyle = groupboxDefaultStyle;
    gStyle.append("QGroupBox {background-color: rgb(137, 255, 128)}");

    this->setStyleSheet(gStyle);
  }
  else
  {
    //    RLOG(0, "Not fulfilled: %s", request.toString().c_str());
    this->setStyleSheet(groupboxDefaultStyle);
  }
}





SensorSignalWidget::SensorSignalWidget(std::string id, double value, bool enableEdit, double minVal, double maxVal) :
  QWidget(), sensorEditEnabled(enableEdit)
{
  pthread_mutex_init(&mutex, NULL);
  //  setMinimumHeight(45);
  setFixedSize(320, 60);

  //  QVBoxLayout* mainGrid = new QVBoxLayout();
  //  setLayout(mainGrid);
  //
  //  QHBoxLayout* labelGrid = new QHBoxLayout();
  //  labelGrid->setMargin(0);
  //  labelGrid->setSpacing(0);
  //
  //  QGroupBox* labelBox = new QGroupBox();
  //  labelBox->setLayout(labelGrid);




  //  QString groupboxStyle = "";
  //  groupboxStyle.append("QGroupBox { "
  //                       "    font-weight: bold; "
  ////                       "    margin-top: 0.5em; "
  ////                       "    border-radius: 10; "
  //                       "    border: 1px solid black;"
  //                       "} ");

  //  groupboxStyle.append("QGroupBox::title { "
  //                       "    subcontrol-origin: margin; "
  //                       //"    subcontrol-position: top left; /* position at the top center */"
  //                       "    padding: 5 0px; "
  //                       "    left: 5px; "
  //                       //"    top: 10px; "
  //                       "} ");

  //  setStyleSheet(groupboxStyle);

  //  QPushButton *but = new QPushButton((req.description).c_str());
  //  but->setToolTip((" (id:" + std::to_string(req.id) + ",seq:" + std::to_string(req.sequenceNum) + ")").c_str());

  enableEditCheckBox = new QCheckBox(this);
  enableEditCheckBox->setGeometry(0, 2, 22, 22);
  enableEditCheckBox->setCheckable(true);
  enableEditCheckBox->setChecked(sensorEditEnabled);



  nameLabel = new QLabel(this);
  //  nameLabel->setFixedSize(200, 20); //nameLabel->size().height());
  nameLabel->setGeometry(25, 4, 195, 17);

  valueBox = new QLineEdit(this);
  valueBox->setAlignment(Qt::AlignRight);
  //  valueBox->setFixedSize(100, 20); //valueBox->height());
  valueBox->setGeometry(220, 0, 100, 27);
  valueBox->setValidator(new QDoubleValidator(minVal, maxVal, 2, this));
  valueBox->setToolTip(("Valid value range: " + std::to_string(minVal) + " to " + std::to_string(maxVal)).c_str());

  slider = new QSlider(Qt::Horizontal, this);
  slider->setGeometry(0, 30, 320, 25);
  slider->setMinimum((int)(minVal * 100.0));
  slider->setMaximum((int)(maxVal * 100.0));
  slider->setTickPosition(QSlider::TicksBelow);
  //  slider->setEnabled(sensorEditEnabled);

  if (sensorEditEnabled)
  {
    slider->setEnabled(true);
    valueBox->setEnabled(true);
  }
  else
  {
    slider->setEnabled(false);
    valueBox->setEnabled(false);
  }

  int numTicks = 10;
  int tickInterval = (int)(((maxVal - minVal) * 100.0) / numTicks);
  slider->setTickInterval(tickInterval);

  connect(slider, SIGNAL(valueChanged(int)),
          this, SLOT(onSliderChanged(int)));

  connect(enableEditCheckBox, SIGNAL(stateChanged(int)),
          this, SLOT(onCheckBoxStateChanged(int)));

  connect(valueBox, SIGNAL(editingFinished()),
          this, SLOT(onValueBoxEditingFinished()));

  update(id, value);
}

SensorSignalWidget::~SensorSignalWidget()
{
  pthread_mutex_destroy(&mutex);
}

void SensorSignalWidget::update(std::string id, double value)
{
  lock();
  sensorValue = value;
  unlock();

  nameLabel->setText((id + ":").c_str());
  char valTxt[30];
  snprintf(valTxt, 30, "% .3f", value);
  valueBox->setText(valTxt);

  if (!slider->isEnabled())
  {
    slider->setValue((int)(value * 100.0));
  }
}

double SensorSignalWidget::getValue()
{
  double result;

  lock();
  result = sensorValue;
  unlock();

  return result;
}

bool SensorSignalWidget::editEnabled()
{
  return sensorEditEnabled;
}

void SensorSignalWidget::onSliderChanged(int value)
{
  double val = (double) value / 100.0;

  lock();
  sensorValue = val;
  unlock();

  char valTxt[30];
  snprintf(valTxt, 30, "% .3f", val);
  valueBox->setText(valTxt);
}

void SensorSignalWidget::onCheckBoxStateChanged(int state)
{
  if (state == Qt::Checked)
  {
    sensorEditEnabled = true;
    slider->setEnabled(true);
    valueBox->setEnabled(true);
  }
  else
  {
    sensorEditEnabled = false;
    slider->setEnabled(false);
    valueBox->setEnabled(false);
  }
}

void SensorSignalWidget::onValueBoxEditingFinished()
{
  double val = valueBox->text().toDouble();

  lock();
  sensorValue = val;
  unlock();

  slider->setValue((int)(val * 100.0));
}

void SensorSignalWidget::lock()
{
  pthread_mutex_lock(&this->mutex);
}

void SensorSignalWidget::unlock()
{
  pthread_mutex_unlock(&this->mutex);
}


} // end namespace Rcs
