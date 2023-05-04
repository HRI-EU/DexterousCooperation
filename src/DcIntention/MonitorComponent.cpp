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

#include "MonitorComponent.h"
#include "MonitorRequestWidget.h"
#include "ObjectModel.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_timer.h>
#include <Rcs_guiFactory.h>
#include <Rcs_Vec3d.h>


namespace Dc
{

MonitorComponent::MonitorComponent(EntityBase* entity, const RcsGraph* graph_,
                                   std::shared_ptr<ObjectModel> model, bool useRealSensors, bool showGui,
                                   bool autoPublish_, bool autoConfirm_) :
  ComponentBase(entity), gui(showGui), realSensors(useRealSensors), autoPublish(autoPublish_),
  autoConfirm(autoConfirm_), publishOnce(false), monitorWidgetBase(NULL), objModel(model)
{
  this->graph = RcsGraph_clone(graph_);

  newTime = getEntity()->getTime();
  lastTime = getEntity()->getTime();

  //  getEntity()->subscribe("UpdateGraph", &MonitorComponent::updateGraph, this);
  getEntity()->subscribe("PostUpdateGraph", &MonitorComponent::postUpdateGraph, this);
  getEntity()->subscribe("Start", &MonitorComponent::start, this);
  getEntity()->subscribe("Stop", &MonitorComponent::stop, this);

  getEntity()->subscribe("RequestMonitor", &MonitorComponent::addMonitorRequest, this);
  getEntity()->subscribe("ResetAndRequestMonitor", &MonitorComponent::resetAndAddMonitorRequest, this);
  getEntity()->subscribe("ClearMonitors", &MonitorComponent::clearMonitorRequests, this);

  publishOnce = false;
}

MonitorComponent::~MonitorComponent()
{
  RcsGraph_destroy(this->graph);
}

void MonitorComponent::start()
{
  RLOG(0, "Start::start()");

  if (gui)
  {
    monitorWidgetBaseHandle = MonitorWidgetBase::create(getEntity(), !realSensors, autoPublish, autoConfirm);

    monitorWidgetBase = (MonitorWidgetBase*)RcsGuiFactory_getPointer(monitorWidgetBaseHandle);
    monitorWidgetBase->syncSensorSignals(sensorData);

    for (size_t i = 0; i < requests.size(); i++)
    {
      addedRequests.push_back(requests[i]);
    }
  }
}

void MonitorComponent::stop()
{
  if (gui && monitorWidgetBase)
  {
    bool success = RcsGuiFactory_destroyGUI(monitorWidgetBaseHandle);

    if (success)
    {
      RLOG(0, "Stop::stop()");
    }
    else
    {
      RLOG(0, "Stop::stop() MonitorWidgetBase NOT deleted.");
    }

    monitorWidgetBase = NULL;
    monitorWidgetBaseHandle = -1;
  }
}

void MonitorComponent::postUpdateGraph(RcsGraph* desiredGraph, RcsGraph* currentGraph)
{
  if (objModel)
  {
    sensorData = objModel->computeSensorSignals(desiredGraph, currentGraph);
  }
  else
  {
    RLOG(0, "No object model available: No sensor signals processed.");
  }

  int countIntentions = 0;
  int countConfirmation = 0;
  int countProgress = 0;

  for (size_t i = 0; i < requests.size(); i++)
  {
    if (requests[i].topic == "Intention")
    {
      countIntentions++;
    }
    else if (requests[i].topic == "Confirmation")
    {
      countConfirmation++;
    }
    else if (requests[i].topic == "Progress")
    {
      countProgress++;
    }
  }

  char text2[256];
  snprintf(text2, 256, "Requests: %d (%d, %d, %d)", (int)requests.size(), countIntentions, countConfirmation,
           countProgress);
  getEntity()->publish<std::string>("SetTextLine", std::string(text2), 4);

  syncGui();

  processRequests();
}

void MonitorComponent::syncGui()
{
  if (gui && monitorWidgetBase != NULL)
  {
    // send updated values to the GUI
    //    monitorWidgetBase->writeSensorSignals(sensorData);

    // sync sensor signals with GUI (send and receive updates)
    monitorWidgetBase->syncSensorSignals(sensorData);

    //    // fetch values from GUI
    //    std::vector<std::pair<std::string, double> > tmp = monitorWidgetBase->readSensorSignals();
    //
    //    for (size_t i = 0; i < tmp.size(); i++)
    //    {
    //      sensorData[tmp[i].first] = tmp[i].second;
    //    }

    if (monitorWidgetBase->getPublishOnce())
    {
      publishOnce = true;
    }
  }

  if (gui && monitorWidgetBase != NULL)
  {
    //synchronize data with widgets
    monitorWidgetBase->updateRequests(addedRequests, removedRequests);
    addedRequests.clear();
    removedRequests.clear();
    if (monitorWidgetBase->getPublishOnce())
    {
      publishOnce = true;
    }
    autoPublish = monitorWidgetBase->getAutoPublish();
    autoConfirm = monitorWidgetBase->getAutoAcceptConfirmation();
  }
}

void MonitorComponent::processRequests()
{
  this->newTime = getEntity()->getTime();

  int count = 0;
  // check all requests
  std::vector<MonitorRequest>::iterator itReq = requests.begin();
  while (itReq != requests.end())
  {
    count++;

    double timeSinceCreation = (newTime - itReq->creationTime);
    if (fabs(itReq->creationTime) < 0.1)
    {
      //TODO: just a fix for now
      timeSinceCreation = 0.0;
    }

    // only check once min time reached
    if ((timeSinceCreation) >= itReq->minTime)
    {
      if (itReq->maxTime != 0.0 && (timeSinceCreation > itReq->maxTime))
      {
        RLOG(0, "Maximum time for request ('%s') exceeded: discarding request.", itReq->topic.c_str());
        removeMonitorRequest(itReq);
        continue;
      }

      double satisfied = itReq->evaluate2(sensorData, newTime);
      //TODO: mad haxx
      getEntity()->publish("MonitorStatusChanged", itReq->topic, itReq->id, itReq->sequenceNum, satisfied);

      if (autoConfirm && itReq->topic == "Confirmation")
      {
        satisfied = 1.0;
      }
      else if (!autoPublish && !publishOnce)
      {
        satisfied = 0.0;
      }

      if (satisfied < 0.0)
      {
        RWARNING(0, "Request ('%s') with unknown sensor data. Discarding.",
                 (itReq->topic + " " + itReq->description).c_str());
        removeMonitorRequest(itReq);
        continue;
      }
      else if (satisfied >= 1.0)
      {
        RLOG(0, "All conditions for request '%s:%d - %s' met. Emitting signal.",
             itReq->topic.c_str(), itReq->id, itReq->description.c_str());
        getEntity()->publish<int, int>(itReq->topic, itReq->id, itReq->sequenceNum);
        itReq->printStatus(sensorData, newTime);
        removeMonitorRequest(itReq);
        continue;
      }
    }

    //    printf("Request %d/%d: \n", count, (int) requests.size());
    //    int satisfied = itReq->evaluate(sensorData, dt);

    itReq++;
  }
  publishOnce = false;
}

void MonitorComponent::addMonitorRequest(MonitorRequest req)
{
  //TODO: not sure if this is good
  if (req.creationTime == 0.0)
  {
    //    req.creationTime = Timer_getSystemTime();
    req.creationTime = getEntity()->getTime();
    req.lastDeactivationTime = req.creationTime;
  }

  //  RLOG(0, "Received new '%s' monitor request (%d signal): ID '%d', Seq: '%d'.",
  // req.topic.c_str(), (int) req.sensorSignals.size(), req.id, req.sequenceNum);
  //  RLOG(0, "Creation time: %5.2f, entity time: %5.2f, system time: %5.2f ",
  // req.creationTime, getEntity()->getTime(), Timer_getSystemTime());
  requests.push_back(req);
  if (gui)
  {
    addedRequests.push_back(req);
  }
}


//TODO: input string for selective clear
void MonitorComponent::clearMonitorRequests(std::string type)
{
  //  RLOG(0, "Clearing all monitor requests. Type: '%s'", type.c_str());

  //  int count = 0;
  std::vector<MonitorRequest>::iterator itReq = requests.begin();
  while (itReq != requests.end())
  {
    //    RLOG(0, "Request #%d of type '%s' and ID '%d', Seq: '%d'",
    //++count, itReq->topic.c_str(), itReq->id, itReq->sequenceNum);
    if (type == "" || type == "all" || itReq->topic == type)
    {
      removeMonitorRequest(itReq);
    }
    else
    {
      itReq++;
    }
  }
}

void MonitorComponent::resetAndAddMonitorRequest(MonitorRequest req)
{
  clearMonitorRequests();

  //  RLOG(0, "Received new monitor request for %d signal(s) with reply on '%s'.",
  //req.sensorSignals.size(), req.topic.c_str());
  addMonitorRequest(req);
}

void MonitorComponent::removeMonitorRequest(std::vector<MonitorRequest>::iterator it)
{
  if (gui)
  {
    removedRequests.push_back(*it);
  }
  requests.erase(it);
}


}   // namespace