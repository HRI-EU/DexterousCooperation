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

#include "TrajectoryComponent.h"
#include "TrajectoryPredictor.h"

#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>


namespace Dc
{

TrajectoryComponent::TrajectoryComponent(EntityBase* parent,
                                         const Rcs::ControllerBase* controller_,
                                         bool via,
                                         double horizon,
                                         bool checkTrajectory_) :
  ComponentBase(parent), tc(NULL), motionEndTime(0.0),
  lastMotionEndTime(0.0), a_des(NULL), x_des(NULL),
  qPred(NULL), animationGraph(NULL), animationTic(0),
  enableTrajectoryCheck(checkTrajectory_), enableDbgRendering(true),
  eStop(false)

{
  Rcs::ControllerBase* controller = new Rcs::ControllerBase(*controller_);
  this->a_des   = MatNd_create((int) controller->getNumberOfTasks(), 1);
  this->x_des   = MatNd_create((int) controller->getTaskDim(), 1);
  controller->computeX(this->x_des);

  this->animationGraph = RcsGraph_clone(controller->getGraph());

  if (via)
  {
    tc = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(controller, horizon);
  }
  else
  {
    tc = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(controller, horizon);
  }

  //tc->readActivationsFromXML();
  controller->readActivationsFromXML(this->a_des);
  tc->takeControllerOwnership(true);
  for (unsigned int i=0; i<a_des->m; ++i)
  {
    tc->setActivation(i, (a_des->ele[i]==0.0) ? false : true);
  }

  subscribe<>("ClearTrajectory", &TrajectoryComponent::onClearTrajectory);
  subscribe<>("EmergencyStop", &TrajectoryComponent::onEmergencyStop);
  subscribe<>("EmergencyRecover", &TrajectoryComponent::onEmergencyRecover);
  subscribe<const RcsGraph*>("InitFromState", &TrajectoryComponent::onInitFromState);
  subscribe<RcsGraph*>("ComputeTrajectory", &TrajectoryComponent::stepTrajectory);
  subscribe("SetTrajectory", &TrajectoryComponent::onApplyTrajectory);
  subscribe("SetDebugRendering", &TrajectoryComponent::enableDebugRendering);
  subscribe("SetTrajectoryCheck", &TrajectoryComponent::onEnableTrajectoryCheck);

  if (enableTrajectoryCheck)
  {
    subscribe("Render", &TrajectoryComponent::onRender);
  }
}

TrajectoryComponent::~TrajectoryComponent()
{
  // TrajectoryController owns controller, so it does not need to be deleted
  delete this->tc;
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
  MatNd_destroy(this->qPred);
  RcsGraph_destroy(this->animationGraph);
}

void TrajectoryComponent::stepTrajectory(RcsGraph* from)
{
  RcsGraph_setState(tc->getInternalController()->getGraph(), from->q, from->q_dot);

  this->lastMotionEndTime = motionEndTime;
  this->motionEndTime = tc->step(getEntity()->getDt());
  tc->getPosition(0.0, this->x_des);
  tc->getActivation(this->a_des);

  if ((lastMotionEndTime > 0.0) && (motionEndTime == 0.0))
  {
    getEntity()->publish("TrajectoryMoving", false);
  }
  else if ((lastMotionEndTime == 0.0) && (motionEndTime > 0.0))
  {
    getEntity()->publish("TrajectoryMoving", true);
  }

  double blending = tc->computeBlending();
  getEntity()->publish("SetBlending", blending);
}

void TrajectoryComponent::onClearTrajectory()
{
  RLOG(0, "TrajectoryComponent::clearTrajectory()");
  tc->clear();
}

const MatNd* TrajectoryComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* TrajectoryComponent::getTaskCommandPtr() const
{
  return this->x_des;
}

void TrajectoryComponent::onEmergencyStop()
{
  RLOG(0, "TrajectoryComponent::EmergencyStop");
  tc->clear();
  this->eStop = true;
}

void TrajectoryComponent::onEmergencyRecover()
{
  RLOG(0, "TrajectoryComponent::EmergencyRecover");
  this->eStop = false;
}

void TrajectoryComponent::onEnableTrajectoryCheck(bool enable)
{
  RLOG(0, "TrajectoryComponent::SetTrajectoryCheck");
  this->enableTrajectoryCheck = enable;
}

void TrajectoryComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "TrajectoryComponent::onInitFromState()");
  RcsGraph_setState(tc->getInternalController()->getGraph(), target->q, target->q_dot);
  tc->getController()->computeX(this->x_des);
  tc->clear();
  tc->init();
}

void TrajectoryComponent::onApplyTrajectory(tropic::TCS_sptr tSet)
{
  if (this->eStop==true)
  {
    RLOG(0, "Skipping to apply trajectory - E-Stop triggered");
    return;
  }

  if (enableTrajectoryCheck)
  {
    bool trajOk = checkTrajectory(tSet);

    if (!trajOk)
    {
      RLOG(0, "Trajectory invalid - skipping");
      return;
    }

    RLOG(0, "Trajectory is OK");
  }
  else
  {
    RLOG(0, "Skipping trajectory check");
  }

  tc->addAndApply(tSet);

  animationTic = 0;
}

bool TrajectoryComponent::checkTrajectory(tropic::TCS_sptr tSet)
{
  if (!enableTrajectoryCheck)
  {
    return true;
  }

  TrajectoryPredictor pred(tc);
  pred.setTrajectory(tSet);   // also clears it
  bool success = pred.predict(getEntity()->getDt());

  MatNd_destroy(this->qPred);
  this->qPred = MatNd_clone(pred.qStack);

  return success;
}

void TrajectoryComponent::enableDebugRendering(bool enable)
{
  if ((enable==false) && (this->enableDbgRendering != enable))
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "Prediction",
                                                  "erase");
  }

  this->enableDbgRendering = enable;
}

void TrajectoryComponent::onRender()
{
  if ((this->qPred==NULL) || (this->enableDbgRendering==false))
  {
    animationTic = 0;
    return;
  }

  MatNd qi = MatNd_getRowViewTranspose(qPred, animationTic);
  RcsGraph_setState(animationGraph, &qi, NULL);
  getEntity()->publish<std::string,std::string>("RenderCommand", "Prediction",
                                                "setGhostMode");
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "Prediction",
                                                    animationGraph);
  animationTic+=10;
  if (animationTic >= qPred->m)
  {
    animationTic = qPred->m-1;
    animationTic = 0;
  }
}

}   // namespace Rcs
