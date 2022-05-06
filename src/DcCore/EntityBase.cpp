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

#include "EntityBase.h"

#include <Rcs_macros.h>

#include <algorithm>


namespace Rcs
{

EntityBase::EntityBase() : dt(0.05), pause(false), timeFrozen(false),
  eStop(false), maxQueueSize(0)
{
  pthread_mutex_init(&mutex, NULL);
  this->time = 0.0;
  subscribe<>("TogglePause", &EntityBase::onTogglePause, this);
  subscribe<>("ToggleTimeFreeze", &EntityBase::onToggleTimeFrozen, this);
  subscribe<>("Print", &EntityBase::onPrint, this);
  subscribe<>("EmergencyStop", &EntityBase::onEmergencyStop, this);
  subscribe<>("EmergencyRecover", &EntityBase::onEmergencyRecover, this);
}

EntityBase::~EntityBase()
{
  pthread_mutex_destroy(&mutex);
}

double EntityBase::getDt() const
{
  pthread_mutex_lock(&mutex);
  double deltaT = this->dt;
  pthread_mutex_unlock(&mutex);

  return deltaT;
}

void EntityBase::setDt(double dt_)
{
  pthread_mutex_lock(&mutex);
  this->dt = dt_;
  pthread_mutex_unlock(&mutex);
}

double EntityBase::getTime() const
{
  pthread_mutex_lock(&mutex);
  double retTime = this->time;
  pthread_mutex_unlock(&mutex);

  return retTime;
}

void EntityBase::setTime(double time_)
{
  pthread_mutex_lock(&mutex);
  this->time = time_;
  pthread_mutex_unlock(&mutex);
}

void EntityBase::stepTime()
{

  if (!this->getTimeFrozen())
  {
    pthread_mutex_lock(&mutex);
    this->time += dt;
    pthread_mutex_unlock(&mutex);
  }
}

void EntityBase::process()
{
  maxQueueSize = (std::max)(maxQueueSize, queueSize());
  ES::EventSystem::process();

  if (this->pause==true)
  {
    RPAUSE();
  }
}

size_t EntityBase::queueSize() const
{
  return dynamicQueue.size();
}

size_t EntityBase::getMaxQueueSize() const
{
  return maxQueueSize;
}

bool EntityBase::getTimeFrozen() const
{
  return timeFrozen;
}

void EntityBase::onTogglePause()
{
  this->pause = !this->pause;
}

void EntityBase::onToggleTimeFrozen()
{
  this->timeFrozen = !this->timeFrozen;
  RLOG(0, "Time %s", timeFrozen ? "frozen" : "unfrozen");
}

void EntityBase::onPrint()
{
  RLOG_CPP(0, "Dynamic queue has size " << dynamicQueue.size()
           << " max. was " << maxQueueSize);
  auto copyOfMap = getRegisteredEvents();
  size_t count = 1;
  for (auto& entry : copyOfMap)
  {
    std::string eventName = entry.first;
    RLOG_CPP(0, "Event " << count++ << ": " << eventName);
  }

  print(std::cout);
}

void EntityBase::onEmergencyStop()
{
  this->eStop = true;
}

void EntityBase::onEmergencyRecover()
{
  this->eStop = false;
}

bool EntityBase::getEmergencyStopFlag() const
{
  return this->eStop;
}

bool EntityBase::initialize(RcsGraph* graph)
{
  RPAUSE_MSG_DL(1, "Start");
  publish("Start");
  int nIter = processUntilEmpty(10);
  RLOG_CPP(1, "Start took " << nIter << " process() calls, queue is "
           << queueSize());

  // Initialization sequence
  publish("Render");
  nIter = processUntilEmpty(10);
  RLOG_CPP(1, "Render took " << nIter << " process() calls, queue is "
           << queueSize());

  RPAUSE_MSG_DL(1, "UpdateGraph");
  publish<RcsGraph*>("UpdateGraph", graph);
  nIter = processUntilEmpty(10);
  RLOG_CPP(1, "updateGraph took " << nIter << " process() calls, queue is "
           << queueSize());

  RPAUSE_MSG_DL(1, "ComputeKinematics");
  publish<RcsGraph*>("ComputeKinematics", graph);
  processUntilEmpty(10);
  publish("Render");
  processUntilEmpty(10);

  publish<std::string>("ChangeObjectShape", "Box");
  processUntilEmpty(10);


  RPAUSE_MSG_DL(1, "UpdateGraph");
  publish<RcsGraph*>("UpdateGraph", graph);
  nIter = processUntilEmpty(10);
  RLOG_CPP(1, "updateGraph took " << nIter << " process() calls, queue is "
           << queueSize());

  RPAUSE_MSG_DL(1, "ComputeKinematics");
  publish<RcsGraph*>("ComputeKinematics", graph);
  processUntilEmpty(10);
  publish("Render");
  processUntilEmpty(10);

  RPAUSE_MSG_DL(1, "InitFromState");
  publish<const RcsGraph*>("InitFromState", graph);
  publish("Render");
  processUntilEmpty(10);

  RPAUSE_MSG_DL(1, "EnableCommands");
  publish("EnableCommands");
  nIter = processUntilEmpty(10);
  RLOG_CPP(1, "InitFromState++ took " << nIter << " process() calls, queue is "
           << queueSize());

  RPAUSE_MSG_DL(1, "Enter runLoop");

  return true;
}


}   // namespace Rcs
