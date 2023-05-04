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

#include "IKComponent.h"

#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_joint.h>

#include <cfloat>


namespace Dc
{

IKComponent::IKComponent(EntityBase* parent,
                         const Rcs::ControllerBase* controller_,
                         IkSolverType ik) :
  ComponentBase(parent), controller(*controller_), ikSolver(NULL),
  eStop(false), alphaMax(0.05), alpha(0.0), lambda(1.0e-6), blending(1.0),
  renderSolid(false), speedLimitCheck(true), jointLimitCheck(true),
  collisionCheck(true)
{
  switch (ik)
  {
    case RMR:
      ikSolver = new Rcs::IkSolverRMR(&controller);
      break;

    case ConstraintRMR:
      ikSolver = new Rcs::IkSolverConstraintRMR(&controller);
      break;

    default:
      RFATAL("Unknown IkSolverType: %d", ik);
  }

  subscribeAll();
}

IKComponent::~IKComponent()
{
  delete this->ikSolver;
}

void IKComponent::subscribeAll()
{
  subscribe("SetTaskCommand", &IKComponent::onTaskCommand);
  subscribe("SetBlending", &IKComponent::onSetBlending);
  subscribe("EmergencyStop", &IKComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &IKComponent::onEmergencyRecover);
  subscribe("Render", &IKComponent::onRender);
  subscribe("InitFromState", &IKComponent::onInitFromState);
  subscribe("TriggerInitFromDesiredState", &IKComponent::onTriggerInitFromDesiredState);
  subscribe("Print", &IKComponent::print);
}

void IKComponent::onTaskCommand(const MatNd* a, const MatNd* x)
{
  if (this->eStop == true)
  {
    return;
  }

  MatNd* dx_des = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* dH_ca = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* dq_des = MatNd_create(controller.getGraph()->dof, 1);


  // Compute null space gradients
  controller.computeCollisionModel();
  controller.getCollisionGradient(dH_ca);
  RcsGraph_limitJointSpeeds(controller.getGraph(), dH_ca,
                            getEntity()->getDt(), RcsStateIK);

  // Inverse kinematics
  controller.computeDX(dx_des, x);
  controller.computeJointlimitGradient(dH);
  MatNd_addSelf(dH, dH_ca);
  MatNd_constMulSelf(dH, this->blending*this->alpha);
  double det = ikSolver->solveRightInverse(dq_des, dx_des, dH, a, lambda);

  if (det==0.0)
  {
    RLOG(0, "Singular Inverse Kinematics");
    if (eStop==false)
    {
      getEntity()->call("EmergencyStop");
    }
  }

  double scale = RcsGraph_limitJointSpeeds(controller.getGraph(), dq_des,
                                           getEntity()->getDt(), RcsStateFull);

  if (scale<1.0)
  {
    RLOG(4, "Scaling down joint speeds by factor %f", scale);
    MatNd_constMulSelf(dq_des, 0.99999);
  }

  MatNd_addSelf(controller.getGraph()->q, dq_des);

  // Forward kinematics including velocities
  MatNd_constMulSelf(dq_des, 1.0/getEntity()->getDt());
  RcsGraph_setState(controller.getGraph(), NULL, dq_des);

  // Update collision model
  // controller.computeCollisionModel();

#if 0

  bool checkPassed = controller.checkLimits(jointLimitCheck, collisionCheck, speedLimitCheck,
                                            0.0, 0.0, 0.001, 0.0, 0.0);

#else

  // Speed limit check
  if (this->speedLimitCheck == true)
  {
    double scaling = RcsGraph_limitJointSpeeds(controller.getGraph(), dq_des,
                                               1.0, RcsStateFull);
    if (scaling < 1.0)
    {
      RLOG(0, "Joint speed limit violation - triggering emergency stop");
      if (eStop==false)
      {
        getEntity()->call("EmergencyStop");
      }
      RCSGRAPH_TRAVERSE_JOINTS(controller.getGraph())
      {
        if ((!JNT->constrained) &&
            (fabs(dq_des->ele[JNT->jointIndex])>=JNT->speedLimit))
        {
          double sf = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
          RLOG(0, "%s: q_dot=%f   limit=%f [%s]",JNT->name,
               sf*dq_des->ele[JNT->jointIndex], sf*JNT->speedLimit,
               sf==1.0 ? "m/sec" : "deg/sec");
        }
      }
    }
  }

  // Joint limit check
  if (this->jointLimitCheck == true)
  {
    unsigned int aor = RcsGraph_numJointLimitsViolated(controller.getGraph(), 0.0, 0.0, true);
    if (aor > 0)
    {
      RLOG(0, "%d joint limit violations - triggering emergency stop", aor);
      if (eStop==false)
      {
        getEntity()->call("EmergencyStop");
      }
      RcsGraph_printState(controller.getGraph(), controller.getGraph()->q);
    }
  }

  // Collision check
  if (this->collisionCheck == true && controller.getCollisionMdl())
  {
    const double distLimit = 0.001;
    double minDist = RcsCollisionMdl_getMinDist(controller.getCollisionMdl());
    if (minDist < distLimit)
    {
      RLOG(0, "Found collision distance of %f (must be >%f) - triggering emergency stop",
           minDist, distLimit);
      RcsCollisionModel_fprintCollisions(stdout, controller.getCollisionMdl(), distLimit);
      if (eStop==false)
      {
        getEntity()->call("EmergencyStop");
      }
    }
  }

#endif

  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dH_ca);
  MatNd_destroy(dq_des);

  // Gradually activate null space so that it takes 1 second from 0 to alphaMax
  this->alpha += getEntity()->getDt()*alphaMax;
  if (this->alpha > this->alphaMax)
  {
    this->alpha = this->alphaMax;
  }
}

const MatNd* IKComponent::getJointCommandPtr() const
{
  return controller.getGraph()->q;
}

const RcsGraph* IKComponent::getGraph() const
{
  return controller.getGraph();
}

RcsGraph* IKComponent::getGraph()
{
  return controller.getGraph();
}

void IKComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  this->eStop = true;
  this->alpha = 0.0;
}

void IKComponent::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
}

void IKComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "IKComponent::onInitFromState()");
  RcsGraph_setState(controller.getGraph(), target->q, target->q_dot);
}

void IKComponent::onTriggerInitFromDesiredState()
{
  RLOG(0, "IKComponent::onTriggerInitFromDesiredState()");
  getEntity()->publish<const RcsGraph*>("InitFromState", controller.getGraph());
}

void IKComponent::onRender()
{
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "IK", controller.getGraph());
  if (controller.getCollisionMdl())
  {
    getEntity()->publish<const MatNd*>("RenderLines", controller.getCollisionMdl()->cp);
  }

  if (this->renderSolid == false)
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "IK", "setGhostMode");
    this->renderSolid = true;
  }
}

void IKComponent::setSpeedLimitCheck(bool enable)
{
  this->speedLimitCheck = enable;
}

void IKComponent::setJointLimitCheck(bool enable)
{
  this->jointLimitCheck = enable;
}

void IKComponent::setCollisionCheck(bool enable)
{
  this->collisionCheck = enable;
}

void IKComponent::print() const
{
  if (controller.getCollisionMdl())
  {
    RcsCollisionModel_fprintCollisions(stdout, controller.getCollisionMdl(), 1000.0);
  }

  RcsGraph_fprintModelState(stdout, controller.getGraph(), controller.getGraph()->q, NULL, 0);
}

void IKComponent::onSetBlending(double value)
{
  this->blending = value;
  // std::string atext = "Alpha: " + std::to_string(blending*alpha);
  // getEntity()->publish("SetTextLine", atext, 1);
}

void IKComponent::setAlpha(double value)
{
  this->alphaMax = value;
}

double IKComponent::getAlpha() const
{
  return this->alphaMax;
}

void IKComponent::setLambda(double value)
{
  this->lambda = value;
}

double IKComponent::getLambda() const
{
  return this->lambda;
}

void IKComponent::renderSolidModel()
{
  this->renderSolid = true;
}

}   // namespace Dc
