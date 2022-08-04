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

#include "TaskGuiComponent.h"

#include <ControllerWidgetBase.h>
#include <TaskWidget.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>



namespace Rcs
{

class TaskUpdateCallback : public TaskWidget::TaskChangeCallback
{
public:
  TaskUpdateCallback(TaskGuiComponent* gui_) : gui(gui_)
  {
  }

  virtual void callback()
  {
    gui->guiCallback();
  };

  TaskGuiComponent* gui;
};


TaskGuiComponent::TaskGuiComponent(EntityBase* parent,
                                   const ControllerBase* controller_) :
  Rcs::ComponentBase(parent), controller(*controller_), a_des(NULL),
  x_des(NULL), x_des_filt(NULL), x_curr(NULL), filt(NULL), passive(false)
{
  this->a_des   = MatNd_create(controller.getNumberOfTasks(), 1);
  this->x_curr  = MatNd_create(controller.getTaskDim(), 1);
  controller_->readActivationsFromXML(this->a_des);
  controller.computeX(this->x_curr);
  this->x_des = MatNd_clone(this->x_curr);
  this->x_des_filt = MatNd_clone(this->x_curr);

  const double tmc = 0.25;
  const double vmax = 0.4;
  this->filt = new RampFilterND(this->x_curr->ele, tmc, vmax, parent->getDt(),
                                controller.getTaskDim());

  pthread_mutex_init(&this->mtx, NULL);

  subscribe("Start", &TaskGuiComponent::start);
  subscribe("Stop", &TaskGuiComponent::stop);
  subscribe("ComputeKinematics", &TaskGuiComponent::setState);
  subscribe("InitFromState", &TaskGuiComponent::onInitFromState);
}

TaskGuiComponent::~TaskGuiComponent()
{
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
  MatNd_destroy(this->x_des_filt);
  MatNd_destroy(this->x_curr);
  delete this->filt;
  pthread_mutex_destroy(&this->mtx);
}

void TaskGuiComponent::setPassive(bool value)
{
  this->passive = value;
}

void TaskGuiComponent::start()
{
  RLOG(0, "Start::start()");
  if (this->passive)
  {
    subscribe<const MatNd*, const MatNd*>("SetTaskCommand",
                                          &TaskGuiComponent::onTaskCommand);
  }

  int guiHandle =
    ControllerWidgetBase::create(&this->controller, this->a_des, this->x_des,
                                 this->x_curr, &this->mtx, this->passive);

  this->handle.push_back(guiHandle);

  void* ptr = RcsGuiFactory_getPointer(guiHandle);
  ControllerWidgetBase* widget = static_cast<ControllerWidgetBase*>(ptr);
  TaskUpdateCallback* jcb = new TaskUpdateCallback(this);
  widget->registerCallback(jcb);

  RLOG(0, "Start::start() with Gui handle %d", guiHandle);
}

void TaskGuiComponent::stop()
{
  size_t numDestroyed = 0;
  for (size_t i=0; i<this->handle.size(); ++i)
  {
    bool success = RcsGuiFactory_destroyGUI(this->handle[i]);
    numDestroyed += success ? 1 : 0;
  }

  RLOG_CPP(0, "Stop::stop() "
           << (numDestroyed == handle.size() ? "SUCCEEDED" : "FAILED") << " ("
           << numDestroyed << " from " << this->handle.size() << " deleted)");

  this->handle.clear();
}

void TaskGuiComponent::guiCallback()
{
  RLOG(5, "TaskGuiComponent::guiCallback()");
  pthread_mutex_lock(&this->mtx);
  filt->setTarget(this->x_des->ele);
  pthread_mutex_unlock(&this->mtx);
}

/*******************************************************************************
 * This is only subscribed if the Gui is passive. We then write x_des and a_des
 * into the internal arrays so that the Guimakes them visible.
 ******************************************************************************/
void TaskGuiComponent::onTaskCommand(const MatNd* a, const MatNd* x)
{
  RLOG(5, "SetTaskCommand::setTaskCommand(MatNd*,MatNd*)");
  if (this->passive)
  {
    pthread_mutex_lock(&this->mtx);
    MatNd_copy(this->a_des, a);
    MatNd_copy(this->x_des, x);
    pthread_mutex_unlock(&this->mtx);
  }
}

const MatNd* TaskGuiComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* TaskGuiComponent::getTaskCommandPtr() const
{
  return this->x_des_filt;
}

void TaskGuiComponent::setState(RcsGraph* from)
{
  RLOG(5, "ComputeKinematics::setState()");
  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(controller.getGraph(), from->q, NULL);
  filt->iterate();
  filt->getPosition(x_des_filt->ele);
  controller.computeX(this->x_curr);
  pthread_mutex_unlock(&this->mtx);
}

void TaskGuiComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "TaskGuiComponent::onInitFromState()");

  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(controller.getGraph(), target->q, target->q_dot);

  MatNd* x_target = MatNd_create(controller.getTaskDim(), 1);
  controller.computeX(x_target);

  MatNd_setZero(this->a_des);
  MatNd_copy(this->x_des, x_target);
  MatNd_copy(this->x_des_filt, x_target);
  MatNd_copy(this->x_curr, x_target);

  filt->init(x_target->ele);
  pthread_mutex_unlock(&this->mtx);

  for (size_t i=0; i<this->handle.size(); ++i)
  {
    void* ptr = RcsGuiFactory_getPointer(this->handle[i]);
    ControllerWidgetBase* widget = static_cast<ControllerWidgetBase*>(ptr);
    widget->reset(this->a_des, x_target);
  }

  MatNd_destroy(x_target);
}

}   // namespace Rcs
