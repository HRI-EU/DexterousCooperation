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

#include "MonitorWidgetBase.h"
#include "MonitorRequestWidget.h"
#include "MonitorRequest.h"
#include "EntityBase.h"

#include <QGridLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QDoubleValidator>

#include <Rcs_guiFactory.h>
#include <LcdSlider.h>
#include <Rcs_timer.h>
#include <algorithm>

#include <Rcs_macros.h>

namespace Dc
{

typedef struct
{
  void* ptr[10];
} VoidPointerList;

MonitorWidgetBase::MonitorWidgetBase(EntityBase* parent,
                                     bool enableSensorEdit,
                                     bool autoPublish_in,
                                     bool autoConfirm_in) :
  QWidget(), entity(parent), autoPublish(autoPublish_in), autoConfirm(autoConfirm_in)
{
  pthread_mutex_init(&mutex, NULL);
  publishOnce = false;
  sensorEditEnabled = enableSensorEdit;
  ttc = 6.0;
  ttcNew = ttc;

  setWindowTitle("Monitor Request GUI");

  // remove redundant frame
  //  setFrameShape(QFrame::NoFrame);

  QString groupboxStyle = "";
  groupboxStyle.append("QGroupBox { "
                       "    font-weight: bold; "
                       //"    margin-top: 0.5em; "
                       //"    border-radius: 10; "
                       //"    border: 1px solid black;"
                       "} ");

  groupboxStyle.append("QGroupBox::title { "
                       "    subcontrol-origin: margin; "
                       //"    subcontrol-position: top left; /* position at the top center */"
                       "    padding: 5 0px; "
                       "    left: 5px; "
                       //"    top: 10px; "
                       "} ");

  int widthLeftSide = 350;
  int buttonHeight = 28;
  //  int vertSpace = 7;
  int setupHeight = 190;
  int widthMonitorSide = 340;


  //===========================================================================

  QWidget* gbox_leftSide = new QWidget();
  gbox_leftSide->setStyleSheet(groupboxStyle);
  gbox_leftSide->setGeometry(0, 0, widthLeftSide, 800);
  gbox_leftSide->setMinimumSize(widthLeftSide + 5, 600);
  gbox_leftSide->setMaximumWidth(widthLeftSide + 5);

  QVBoxLayout* leftLayout = new QVBoxLayout();
  gbox_leftSide->setLayout(leftLayout);
  leftLayout->setAlignment(Qt::AlignTop);
  leftLayout->setContentsMargins(0, 0, 0, 0);

  //  QGroupBox* gbox_setup = new QGroupBox(tr("Setup:"), gbox_leftSide);
  QWidget* gbox_setup = new QWidget(gbox_leftSide);
  gbox_setup->setStyleSheet(groupboxStyle);
  gbox_setup->setMinimumSize(widthLeftSide, setupHeight);
  gbox_setup->setGeometry(0, 0, widthLeftSide, setupHeight);

  QPushButton* goToStartButton = new QPushButton("Go to start", gbox_setup);
  goToStartButton->setGeometry(0, 0, widthLeftSide - 80, buttonHeight);
  connect(goToStartButton, SIGNAL(clicked()), this, SLOT(goToStart()));

  QPushButton* tareSensorsButton = new QPushButton("Tare", gbox_setup);
  tareSensorsButton->setGeometry(widthLeftSide - 80 + 5, 0, 75, buttonHeight);
  connect(tareSensorsButton, SIGNAL(clicked()), this, SLOT(tareSensors()));

  QPushButton* stopMovementButton = new QPushButton("Stop", this);
  connect(stopMovementButton, SIGNAL(clicked()), this, SLOT(stopMovement()));

  QPushButton* recoverToStateButton = new QPushButton("Recover", this);
  connect(recoverToStateButton, SIGNAL(clicked()), this, SLOT(recoverToState()));

  QWidget* gbox_stop = new QWidget(gbox_setup);
  gbox_stop->setGeometry(0, 20, widthLeftSide, 40);
  gbox_stop->setStyleSheet(groupboxStyle);
  QHBoxLayout* stopLayout = new QHBoxLayout();
  stopLayout->setMargin(0);
  stopLayout->setSpacing(0);

  gbox_stop->setLayout(stopLayout);
  stopLayout->addWidget(stopMovementButton);
  stopLayout->addWidget(recoverToStateButton);

  //===========================================================================

  QWidget* gbox_setTTC = new QWidget(gbox_setup);
  gbox_setTTC->setStyleSheet(groupboxStyle);
  //  gbox_setTTC->setFixedSize(320, 60);
  gbox_setTTC->setGeometry(0, 65, widthLeftSide, 60);

  QPushButton* setTtcButton = new QPushButton("Set TTC", gbox_setTTC);
  setTtcButton->setGeometry(0, 4, 80, buttonHeight);

  char valTxt[30];
  snprintf(valTxt, 30, "%.2f", this->ttc);

  labelCurrentTTC = new QLabel(valTxt, gbox_setTTC);
  labelCurrentTTC->setGeometry(widthLeftSide - 120, 9, 40, buttonHeight);
  labelCurrentTTC->setAlignment(Qt::AlignRight);
  labelCurrentTTC->setToolTip("Current TTC value");

  double minTTC = 0.5;
  double maxTTC = 20.0;

  valueBoxTTC = new QLineEdit(valTxt, gbox_setTTC);
  valueBoxTTC->setAlignment(Qt::AlignRight);
  valueBoxTTC->setGeometry(widthLeftSide - 70, 4, 70, buttonHeight);
  valueBoxTTC->setValidator(new QDoubleValidator(minTTC, maxTTC, 2, gbox_setTTC));
  valueBoxTTC->setToolTip(("Valid value range: " + std::to_string(minTTC) + " to " + std::to_string(maxTTC)).c_str());

  sliderTTC = new QSlider(Qt::Horizontal, gbox_setTTC);
  sliderTTC->setGeometry(0, 30, widthLeftSide, 25);
  sliderTTC->setMinimum((int)(minTTC * 100.0));
  sliderTTC->setMaximum((int)(maxTTC * 100.0));
  sliderTTC->setTickPosition(QSlider::TicksBelow);
  sliderTTC->setValue((int)(ttcNew * 100.0));

  int tickInterval = 100;
  sliderTTC->setTickInterval(tickInterval);

  connect(sliderTTC, SIGNAL(valueChanged(int)), this, SLOT(onSliderTtcChanged(int)));
  connect(valueBoxTTC, SIGNAL(editingFinished()), this, SLOT(onValueBoxTtcEditingFinished()));
  connect(setTtcButton, SIGNAL(clicked()), this, SLOT(onSetTtc()));

  //===========================================================================

  QWidget* gbox_hands = new QWidget(gbox_setup);
  gbox_hands->setGeometry(0, 125, widthLeftSide, 120);

  QPushButton* engageButton = new QPushButton("Engage", gbox_hands);
  engageButton->setGeometry(0, 0, (widthLeftSide / 2 - 10), buttonHeight);
  connect(engageButton, SIGNAL(clicked()), this, SLOT(engageObject()));

  QPushButton* disengageButton = new QPushButton("Disengage", gbox_hands);
  disengageButton->setGeometry((widthLeftSide / 2 + 10), 0, (widthLeftSide / 2 - 10), buttonHeight);
  connect(disengageButton, SIGNAL(clicked()), this, SLOT(disengageObject()));

  QPushButton* closeHandsButton = new QPushButton("Close Hands", gbox_hands);
  closeHandsButton->setGeometry(0, 30, (widthLeftSide / 2 - 10), buttonHeight);
  connect(closeHandsButton, SIGNAL(clicked()), this, SLOT(closeHands()));

  QPushButton* openHandsButton = new QPushButton("Open Hands", gbox_hands);
  openHandsButton->setGeometry((widthLeftSide / 2 + 10), 30, (widthLeftSide / 2 - 10), buttonHeight);
  connect(openHandsButton, SIGNAL(clicked()), this, SLOT(openHands()));




  // Widget for sensor signal related sliders and some information
  QGroupBox* gbox_sensorSignals = new QGroupBox(tr("Sensor signals:"));
  //gbox_sensorSignals->setGeometry(0, setupHeight + vertSpace, widthLeftSide, 800);
  gbox_sensorSignals->setStyleSheet(groupboxStyle);

  sensorLayout = new QVBoxLayout();
  sensorLayout->setMargin(2);
  sensorLayout->setSpacing(1);
  sensorLayout->addStretch();
  sensorLayout->setAlignment(Qt::AlignTop);

  gbox_sensorSignals->setLayout(sensorLayout);

  QWidget* gbox_evaluate = new QWidget();
  gbox_evaluate->setFixedSize(320, 60);

  QCheckBox* autoAcceptConfirmationCheckbox = new QCheckBox("Auto accept ALL confirmation requests", gbox_evaluate);
  autoAcceptConfirmationCheckbox->setGeometry(0, 5, 300, 22);
  autoAcceptConfirmationCheckbox->setChecked(autoConfirm);
  connect(autoAcceptConfirmationCheckbox,
          SIGNAL(stateChanged(int)),
          this,
          SLOT(onAutoAcceptConfirmationCheckBoxChanged(int)));

  autoEvaluateCheckbox = new QCheckBox("Auto evaluate", gbox_evaluate);
  autoEvaluateCheckbox->setGeometry(0, 30, 140, 22);
  autoEvaluateCheckbox->setChecked(autoPublish);
  connect(autoEvaluateCheckbox, SIGNAL(stateChanged(int)), this, SLOT(onAutoEvaluateCheckBoxChanged(int)));

  evaluateButton = new QPushButton("Evaluate Requests", gbox_evaluate);
  evaluateButton->setGeometry(160, 28, 160, 27);
  connect(evaluateButton, SIGNAL(clicked()), this, SLOT(evaluateRequests()));
  if (autoPublish)
  {
    evaluateButton->setEnabled(false);
  }

  sensorLayout->addWidget(gbox_evaluate);

  QScrollArea* sensorScrollArea = new QScrollArea();//gbox_leftSide);
  //  sensorScrollArea->setGeometry(0, setupHeight + vertSpace, widthLeftSide, 700);

  sensorScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  sensorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  sensorScrollArea->setWidgetResizable(true);
  sensorScrollArea->setFrameShape(QFrame::NoFrame);
  //  sensorScrollArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::);
  sensorScrollArea->setWidget(gbox_sensorSignals);

  leftLayout->addWidget(gbox_setup);
  leftLayout->addWidget(sensorScrollArea);


  //===========================================================================
  //===========================================================================

  requestLayout = new QBoxLayout(QBoxLayout::BottomToTop);
  requestLayout->setMargin(5);
  requestLayout->setSpacing(1);
  requestLayout->addStretch();

  gbox_monitorRequests = new QGroupBox(tr("Other Active Requests:"));
  gbox_monitorRequests->setStyleSheet(groupboxStyle);
  gbox_monitorRequests->setLayout(requestLayout);

  intentionLayout = new QBoxLayout(QBoxLayout::BottomToTop);
  intentionLayout->setMargin(5);
  intentionLayout->setSpacing(1);
  intentionLayout->addStretch();

  gbox_monitorIntentions = new QGroupBox(tr("Intention Requests:"));
  gbox_monitorIntentions->setStyleSheet(groupboxStyle);
  gbox_monitorIntentions->setLayout(intentionLayout);
  //  gbox_monitorIntentions->setMinimumSize(300, 200);

  confirmationLayout = new QBoxLayout(QBoxLayout::BottomToTop);
  confirmationLayout->setMargin(5);
  confirmationLayout->setSpacing(1);
  confirmationLayout->addStretch();

  gbox_monitorConfirmation = new QGroupBox(tr("Confirmation Requests:"));
  gbox_monitorConfirmation->setStyleSheet(groupboxStyle);
  gbox_monitorConfirmation->setLayout(confirmationLayout);

  progressLayout = new QBoxLayout(QBoxLayout::BottomToTop);
  progressLayout->setMargin(5);
  progressLayout->setSpacing(1);
  progressLayout->addStretch();

  gbox_monitorProgress = new QGroupBox(tr("Progress Requests:"));
  gbox_monitorProgress->setStyleSheet(groupboxStyle);
  gbox_monitorProgress->setLayout(progressLayout);

  // The layout for the overall task widget
  QVBoxLayout* monitorLayout = new QVBoxLayout();
  monitorLayout->setAlignment(Qt::AlignTop);
  monitorLayout->addWidget(createLine());
  monitorLayout->addWidget(gbox_monitorIntentions);
  monitorLayout->addWidget(createLine());
  monitorLayout->addWidget(gbox_monitorConfirmation);
  monitorLayout->addWidget(createLine());
  monitorLayout->addWidget(gbox_monitorProgress);
  monitorLayout->addWidget(createLine());
  monitorLayout->addWidget(gbox_monitorRequests);
  //  monitorLayout->addStretch();

  QGroupBox* gbox_monitors = new QGroupBox(tr("Active Requests:"));
  gbox_monitors->setStyleSheet(groupboxStyle);
  gbox_monitors->setLayout(monitorLayout);
  gbox_monitors->setMinimumWidth(widthMonitorSide + 5);

  QScrollArea* monitorScrollArea = new QScrollArea();
  monitorScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  monitorScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  monitorScrollArea->setWidgetResizable(true);
  monitorScrollArea->setFrameShape(QFrame::NoFrame);
  monitorScrollArea->setWidget(gbox_monitors);

  //===========================================================================

  QHBoxLayout* guiLayout = new QHBoxLayout();
  guiLayout->setAlignment(Qt::AlignLeft);
  setSizePolicy(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
  guiLayout->addWidget(gbox_leftSide);
  guiLayout->addWidget(createLine(false));
  guiLayout->addWidget(monitorScrollArea);
  guiLayout->addWidget(createLine(false));
  //  guiLayout->addStretch();

  this->setLayout(guiLayout);

  resize(760, 950);

  //  for (int i = 0; i < 17; i++)
  //  {
  //    MonitorRequest mr("Intention", i, i, "Huihuihui All of it", 2.0);
  //    mr.addSignal("test", 0.4, false);
  //    mr.addSignal("test", 0.4, false);
  //    mr.addSignal("test", 0.4, false);
  //    mr.addSignal("test", 0.4, false);
  //    MonitorRequestWidget* tmp = new MonitorRequestWidget(entity, mr);
  //
  //
  //    intentionLayout->addWidget(tmp);
  //  }


  entity->subscribe("SetTTC", &MonitorWidgetBase::onSetTtcEvent, this);


  //
  // 25 Hz timer callback
  //
  _timer = new QTimer(this);
  connect(_timer, SIGNAL(timeout()), SLOT(displayAct()));
  _timer->start(40);

}

MonitorWidgetBase::~MonitorWidgetBase()
{
  _timer->stop();
  _timer->deleteLater();
  sensorSignals.clear();

  for (std::map<std::string, QLabel*>::iterator it = sensorLabels.begin(); it != sensorLabels.end(); ++it)
  {
    it->second->deleteLater();
  }

  sensorLabels.clear();
  pthread_mutex_destroy(&mutex);
}

QFrame* MonitorWidgetBase::createLine(bool horizontal)
{
  QFrame* line;
  line = new QFrame();
  if (horizontal)
  {
    line->setFrameShape(QFrame::HLine);
  }
  else
  {
    line->setFrameShape(QFrame::VLine);
  }
  line->setFrameShadow(QFrame::Sunken);
  return line;
}

void MonitorWidgetBase::onSliderTtcChanged(int value)
{
  this->ttcNew = (double) value / 100.0;
  char valTxt[30];
  snprintf(valTxt, 30, "%.2f", this->ttcNew);
  valueBoxTTC->setText(valTxt);
}

void MonitorWidgetBase::onValueBoxTtcEditingFinished()
{
  this->ttcNew = valueBoxTTC->text().toDouble();
  sliderTTC->setValue((int)(this->ttcNew * 100.0));
  onSetTtc();
}

void MonitorWidgetBase::onSetTtc()
{
  RLOG(0, "Sending 'SetTTC' with value %5.2f", this->ttcNew);
  entity->publish("SetTTC", this->ttcNew);
}

void MonitorWidgetBase::onSetTtcEvent(double ttcIn)
{
  this->ttc = ttcIn;
  char valTxt[30];
  snprintf(valTxt, 30, "%.2f", this->ttc);
  labelCurrentTTC->setText(valTxt);
  sliderTTC->setValue((int)(this->ttc * 100.0));
}

void MonitorWidgetBase::displayAct()
{
  lock();
  std::map<std::string, double> sig(sensorSignals);
  unlock();

  // read back sensor values controlled by GUI
  std::vector<std::pair<std::string, double> > editedSignals;

  auto it3 = sensorSignalWidgets.begin();
  while (it3 != sensorSignalWidgets.end())
  {
    if (it3->second->editEnabled())
    {
      // add to list of changed signals
      std::pair<std::string, double> tmp(it3->first, it3->second->getValue());
      editedSignals.push_back(tmp);

      // update in current signal list
      sig[it3->first] = it3->second->getValue();
    }
    it3++;
  }

  lock();
  this->signalUpdates = editedSignals;
  unlock();

  auto it = sig.begin();
  while (it != sig.end())
  {
    auto it2 = sensorSignalWidgets.find(it->first);
    if (it2 != sensorSignalWidgets.end())
    {
      //      if (!sensorEditEnabled || (sensorEditEnabled != it2->second->editEnabled()))
      if (!it2->second->editEnabled())
      {
        it2->second->update(it->first, it->second);
      }
    }
    else
    {
      RLOG(5, "Adding sensor: %s, enabled: %d", it->first.c_str(), sensorEditEnabled);
      //TODO: use sensorEditEnabled
      sensorSignalWidgets[it->first] = new SensorSignalWidget(it->first, it->second, sensorEditEnabled);
      sensorLayout->addWidget(sensorSignalWidgets[it->first]);
    }
    it++;
  }

  lock();
  std::vector<MonitorRequest> add(addRequests);
  addRequests.clear();
  std::vector<MonitorRequest> remove(removeRequests);
  removeRequests.clear();
  unlock();

  // process all requests to be removed

  auto itReq2 = remove.begin();
  while (itReq2 != remove.end())
  {
    std::string ident = itReq2->toString();
    auto it2 = requestWidgets.find(ident);
    if (it2 != requestWidgets.end())
    {
      //      RLOG(0, "RequestWidget already exist...");
      requestLayout->removeWidget(it2->second);
      it2->second->deleteLater();
      requestWidgets.erase(ident);
      continue;
    }
    else
    {
      //check if it is still in the add queue
      auto itAdd = add.begin();
      while (itAdd != add.end())
      {
        if (ident == itAdd->toString())
        {
          add.erase(itAdd);
          continue;
        }
        itAdd++;
      }
    }
    itReq2++;
  }

  // process all requests to be added

  //  RLOG(0, "%d new requests to add, %d to be deleted", (int) add.size(), (int) remove.size());

  auto itReq = add.begin();
  while (itReq != add.end())
  {
    std::string ident = itReq->toString();
    auto it2 = requestWidgets.find(ident);
    if (it2 != requestWidgets.end())
    {
      RLOG(0, "RequestWidget already exist...");
    }
    else
    {
      MonitorRequestWidget* tmp = new MonitorRequestWidget(entity, *itReq);
      requestWidgets[ident] = tmp;

      if (itReq->topic == "Intention")
      {
        intentionLayout->addWidget(tmp);
      }
      else if (itReq->topic == "Confirmation")
      {
        confirmationLayout->addWidget(tmp);
      }
      else if (itReq->topic == "Progress")
      {
        progressLayout->addWidget(tmp);
      }
      else
      {
        requestLayout->addWidget(tmp);
      }
    }
    itReq++;
  }

  //update values for all existing requests
  auto itReq3 = requestWidgets.begin();
  while (itReq3 != requestWidgets.end())
  {
    itReq3->second->update(sig, 0.0); //TODO: we need to get access to time here

    itReq3++;
  }

}

void MonitorWidgetBase::goToStart()
{
  entity->publish("MoveToStartPose", 3.0);
}

void MonitorWidgetBase::tareSensors()
{
  entity->publish("Tare");
}

void MonitorWidgetBase::openHands()
{
  entity->publish("OpenHands", 3.0);
}

void MonitorWidgetBase::closeHands()
{
  entity->publish("CloseHands", 3.0);
}

void MonitorWidgetBase::engageObject()
{
  entity->publish("Engage", 6.0);
}

void MonitorWidgetBase::disengageObject()
{
  entity->publish("Disengage", 6.0);
}

void MonitorWidgetBase::stopMovement()
{
  entity->publish("StopActions");
}

void MonitorWidgetBase::recoverToState()
{
  entity->publish("RecoverToClosestState");
}

void MonitorWidgetBase::syncSensorSignals(std::map<std::string, double>& sig)
{
  lock();
  sensorSignals = sig;

  for (size_t i = 0; i < this->signalUpdates.size(); i++)
  {
    sig[this->signalUpdates[i].first] = this->signalUpdates[i].second;
  }

  unlock();
}

void MonitorWidgetBase::updateRequests(std::vector<MonitorRequest> addReq, std::vector<MonitorRequest> removeReq)
{
  lock();
  //  if (!addReq.empty())
  //  {
  //    RLOG(0, "Add requests: %d, new: %d", (int) addRequests.size(), (int) addReq.size() );
  //  }
  addRequests.insert(addRequests.end(), addReq.begin(), addReq.end());
  //
  //  if (!addReq.empty())
  //  {
  //    RLOG(0, "New total adds: %d", (int) addRequests.size() );
  //    for (int i = 0; i < (int) addRequests.size(); i++)
  //    {
  //      RLOG(0, "%d:  %2d - %s %d", i, addRequests[i].id, addRequests[i].toString().c_str(), addRequests[i].sequenceNum);
  //    }
  //  }
  //
  //  if (!removeReq.empty())
  //  {
  //    RLOG(0, "Remove requests: %d, new: %d", (int) removeRequests.size(), (int) removeReq.size() );
  //  }
  removeRequests.insert(removeRequests.end(), removeReq.begin(), removeReq.end());

  //  if (!removeReq.empty())
  //  {
  //    RLOG(0, "New total dels: %d", (int) removeRequests.size() );
  //    for (int i = 0; i < (int) removeRequests.size(); i++)
  //    {
  //      RLOG(0, "%d:  %2d - %s %d", i, removeRequests[i].id, removeRequests[i].toString().c_str(), removeRequests[i].sequenceNum);
  //    }
  //  }
  unlock();
}

bool MonitorWidgetBase::getPublishOnce()
{
  bool result = publishOnce;
  publishOnce = false;
  return result;
}

bool MonitorWidgetBase::getAutoPublish()
{
  return autoPublish;
}

bool MonitorWidgetBase::getAutoAcceptConfirmation()
{
  return autoConfirm;
}

void MonitorWidgetBase::evaluateRequests()
{
  publishOnce = true;
}

void MonitorWidgetBase::onAutoEvaluateCheckBoxChanged(int state)
{
  if (state == Qt::Checked)
  {
    evaluateButton->setEnabled(false);
    autoPublish = true;
    entity->publish("StartLogging");
  }
  else
  {
    evaluateButton->setEnabled(true);
    autoPublish = false;
    entity->publish("StopLogging");
  }
}

void MonitorWidgetBase::onAutoAcceptConfirmationCheckBoxChanged(int state)
{
  if (state == Qt::Checked)
  {
    //    autoEvaluateCheckbox->setEnabled(false);
    //    evaluateButton->setEnabled(false);

    autoConfirm = true;
  }
  else
  {
    //    autoEvaluateCheckbox->setEnabled(true);
    //    if (!autoEvaluateCheckbox->isChecked())
    //    {
    //      evaluateButton->setEnabled(true);
    //    }
    autoConfirm = false;
  }
}


/*****************************************************************************
  \brief Thread function.
*****************************************************************************/
void* MonitorWidgetBase::monitorGuiBase(void* arg)
{
  VoidPointerList* p = (VoidPointerList*) arg;
  RCHECK(arg);

  EntityBase* entity = (EntityBase*) p->ptr[0];
  bool enableSensorEdit = (bool) p->ptr[1]; //TODO: does this have to be bool* instead?
  bool autoPublish = (bool) p->ptr[2];
  bool autoConfirm = (bool) p->ptr[3];
  //  Rcs::ControllerBase* cntrl = (Rcs::ControllerBase*) p->ptr[0];
  //  MatNd* a_des                 = (MatNd*) p->ptr[1];
  //  MatNd* x_des                 = (MatNd*) p->ptr[2];
  //  const MatNd* x_curr          = (MatNd*) p->ptr[3];
  //  pthread_mutex_t* mutex       = (pthread_mutex_t*) p->ptr[4];
  //  bool* showOnly               = (bool*) p->ptr[5];
  //  MatNd* a_curr                = (MatNd*) p->ptr[6];
  //
  //  delete p;

  MonitorWidgetBase* widget = new MonitorWidgetBase(entity, enableSensorEdit, autoPublish, autoConfirm);
  widget->show();

  //  delete showOnly;

  return widget;
}


/*****************************************************************************
  \brief Static instantiation method.
*****************************************************************************/
int MonitorWidgetBase::create(EntityBase* entity, bool enableSensorEdit, bool autoPublish, bool autoConfirm)
{
  VoidPointerList* p = new VoidPointerList;
  p->ptr[0] = (void*) entity;
  p->ptr[1] = (void*) enableSensorEdit;
  p->ptr[2] = (void*) autoPublish;
  p->ptr[3] = (void*) autoConfirm;

  int handle = RcsGuiFactory_requestGUI(MonitorWidgetBase::monitorGuiBase, (void*) p);

  return handle;
}


void MonitorWidgetBase::lock()
{
  pthread_mutex_lock(&this->mutex);
}

void MonitorWidgetBase::unlock()
{
  pthread_mutex_unlock(&this->mutex);
}


}
