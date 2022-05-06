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

#include "JointGuiComponent.h"

#include <JointWidget.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_guiFactory.h>



namespace Rcs
{

class JointUpdateCallback : public JointWidget::JointChangeCallback
{
public:
  JointUpdateCallback(JointGuiComponent* gui_) : gui(gui_)
  {
  }

  virtual void callback()
  {
    gui->guiCallback();
  };

  JointGuiComponent* gui;
};


JointGuiComponent::JointGuiComponent(EntityBase* parent, const RcsGraph* g,
                                     double tmc, double vmax) :
  ComponentBase(parent),
  graph(NULL),
  q_des(NULL),
  q_curr(NULL),
  q_des_filt(NULL),
  filt(NULL)
{
  RCHECK(tmc>=0.0);
  RCHECK(vmax>=0.0);
  this->graph = RcsGraph_clone(g);
  this->q_des = MatNd_clone(g->q);
  this->q_des_filt = MatNd_clone(g->q);
  this->q_curr = MatNd_clone(g->q);

  this->filt = new RampFilterND(q_curr->ele, tmc, vmax, parent->getDt(), g->dof);
  pthread_mutex_init(&this->mtx, NULL);

  subscribeAll();
}

JointGuiComponent::~JointGuiComponent()
{
  unsubscribe();
  RcsGraph_destroy(this->graph);
  MatNd_destroy(this->q_des);
  MatNd_destroy(this->q_des_filt);
  MatNd_destroy(this->q_curr);
  delete this->filt;
  pthread_mutex_destroy(&this->mtx);
}

void JointGuiComponent::subscribeAll()
{
  subscribe("Start", &JointGuiComponent::onStart);
  subscribe("Stop", &JointGuiComponent::onStop);
  subscribe<const RcsGraph*>("InitFromState", &JointGuiComponent::onInitialize);
  subscribe("ComputeKinematics", &JointGuiComponent::onFilterAndUpdateGui);
  subscribe("EmergencyStop", &JointGuiComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &JointGuiComponent::onEmergencyRecover);
}

void JointGuiComponent::onStart()
{
  RLOG(1, "Start::start()");
  MatNd_copy(this->q_des, graph->q);
  int guiHandle = JointWidget::create(this->graph, &this->mtx,
                                      this->q_des, this->q_curr);

  this->handle.push_back(guiHandle);

  void* ptr = RcsGuiFactory_getPointer(guiHandle);
  JointWidget* widget = static_cast<JointWidget*>(ptr);

  JointUpdateCallback* jcb = new JointUpdateCallback(this);
  widget->registerCallback(jcb);
}

void JointGuiComponent::guiCallback()
{
  pthread_mutex_lock(&this->mtx);
  filt->setTarget(this->q_des->ele);
  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::setGoalPose(const MatNd* q_goal)
{
  if ((q_goal->m!=this->q_des->m) || (q_goal->n!=this->q_des->n))
  {
    RLOG(1, "Mismatch in setGoalPose(): q_goal is [%d x %d], but state vector "
         "should be [%d x %d] - skipping q_goal", q_goal->m, q_goal->n,
         this->q_des->m, this->q_des->n);
    return;
  }

  pthread_mutex_lock(&this->mtx);
  filt->setTarget(q_goal->ele);

  for (size_t i=0; i<this->handle.size(); ++i)
  {
    void* ptr = RcsGuiFactory_getPointer(handle[i]);
    JointWidget* widget = static_cast<JointWidget*>(ptr);
    widget->reset(q_goal);
  }

  pthread_mutex_unlock(&this->mtx);
}

const RcsGraph* JointGuiComponent::getGraph() const
{
  return this->graph;
}

void JointGuiComponent::onFilterAndUpdateGui(RcsGraph* from)
{
  RLOG(5, "ComputeKinematics::setState()");
  pthread_mutex_lock(&this->mtx);
  MatNd_copy(this->q_curr, from->q);
  filt->iterate();
  filt->getPosition(q_des_filt->ele);
  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::onEmergencyStop()
{
  RLOG(1, "EmergencyStop");
  MatNd_copy(graph->q, this->q_curr);
  onInitialize(this->graph);
}

void JointGuiComponent::onEmergencyRecover()
{
  RLOG(1, "EmergencyRecover");
  MatNd_copy(graph->q, this->q_curr);
  onInitialize(this->graph);
}

void JointGuiComponent::onInitialize(const RcsGraph* target)
{
  RLOG(1, "Initialize::initialize()");
  pthread_mutex_lock(&this->mtx);

  MatNd_copy(this->q_curr, target->q);
  MatNd_copy(this->q_des, target->q);
  MatNd_copy(this->q_des_filt, target->q);
  filt->init(target->q->ele);

  for (size_t i=0; i<this->handle.size(); ++i)
  {
    void* ptr = RcsGuiFactory_getPointer(handle[i]);
    JointWidget* widget = static_cast<JointWidget*>(ptr);
    widget->reset(target->q);
  }

  pthread_mutex_unlock(&this->mtx);
}

void JointGuiComponent::onStop()
{
  size_t numDestroyed = 0;
  for (size_t i=0; i<this->handle.size(); ++i)
  {
    bool success = RcsGuiFactory_destroyGUI(this->handle[i]);
    numDestroyed += success ? 1 : 0;
  }

  RLOG(1, "Stop::stop() %s (%d from %d deleted)",
       (numDestroyed==this->handle.size()) ? "SUCCEEDED" : "FAILED",
       (int) numDestroyed, (int) this->handle.size());

  this->handle.clear();
}

const MatNd* JointGuiComponent::getJointCommandPtr() const
{
  return this->q_des_filt;
}

}   // namespace Rcs
