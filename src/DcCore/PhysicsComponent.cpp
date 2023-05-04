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

#include "PhysicsComponent.h"
#include "PhysicsFactory.h"

#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_timer.h>
#include <Rcs_math.h>



namespace Dc
{

/*******************************************************************************
 * Constructor
 ******************************************************************************/
PhysicsComponent::PhysicsComponent(EntityBase* parent,
                                   RcsGraph* graph_,
                                   const char* engine,
                                   const char* cfgFile,
                                   bool threaded_) :
  ComponentBase(parent),
  PeriodicCallback(),
  dtSim(0.0),
  tStart(Timer_getSystemTime()),
  sim(NULL),
  ffwd(false),
  eStop(false),
  eRecover(false),
  threaded(threaded_),
  renderingInitialized(false),
  enableCommands(false)
{
  setClassName("PhysicsComponent");
  setUpdateFrequency(200.0);
  this->sim = Rcs::PhysicsFactory::create(engine, graph_, cfgFile);
  this->q_curr = MatNd_clone(graph_->q);
  this->q_dot_curr = MatNd_clone(graph_->q_dot);

  subscribeAll(this->sim);

  if (this->sim)
  {
    // sim->disableCollisions();
    // sim->setJointLimits(false);
  }
  else
  {
    RLOG(0, "Can't create engine %s", engine);
    //RFATAL("EXIT");
  }

}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
void PhysicsComponent::subscribeAll(Rcs::PhysicsBase* physics)
{
  subscribe("SetRandomPose", &PhysicsComponent::onRandomPose);
  subscribe("EnableCommands", &PhysicsComponent::onEnableCommands);
  subscribe<const RcsGraph*>("InitFromState", &PhysicsComponent::onInitFromState);
  subscribe("SetJointCommand", &PhysicsComponent::setPositionCommand);

  if (physics)
  {
    subscribe("Start", &PhysicsComponent::start);
    subscribe("Stop", &PhysicsComponent::stop);
    subscribe("EmergencyStop", &PhysicsComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &PhysicsComponent::onEmergencyRecover);
    subscribe<std::string,bool>("SetObjectActivation", &PhysicsComponent::onObjectActivation);
    subscribe<RcsGraph*>("UpdateGraph", &PhysicsComponent::updateGraph);
    subscribe("ResetRigidBodies", &PhysicsComponent::onResetRigidBodies);
  }
  else
  {
    subscribe<RcsGraph*>("UpdateGraph", &PhysicsComponent::updateGraphWithoutPhysics);
  }
}

/*******************************************************************************
 * Destructor
 ******************************************************************************/
PhysicsComponent::~PhysicsComponent()
{
  stop();
  unsubscribe();
  delete this->sim;
  MatNd_destroy(this->q_curr);
  MatNd_destroy(this->q_dot_curr);
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::callback()
{
  double tmp = Timer_getSystemTime();

  if (this->ffwd==true)
  {
    MatNd* q_old = NULL;
    MatNd_clone2(q_old, this->q_curr);

    simMtx.lock();
    sim->getLastPositionCommand(this->q_curr);
    simMtx.unlock();

    const double dt = getEntity()->getDt();

    if (dt>0.0 && !this->eRecover)
    {
      // compute joint velocities using finite differences.
      MatNd_sub(this->q_dot_curr, this->q_curr, q_old);
      MatNd_constMulSelf(this->q_dot_curr, 1 / dt);
    }
    else
    {
      // no velocities if dt is 0 or we're recovering from an emergency (avoids jumps)
      MatNd_setZero(this->q_dot_curr);
    }
    MatNd_destroy(q_old);
  }
  else
  {
    const double dt = getEntity()->getDt();

    if (dt>0.0)
    {
      simMtx.lock();
      sim->simulate(dt, this->q_curr, this->q_dot_curr, NULL, NULL, this->enableCommands);
      simMtx.unlock();
    }
    else
    {
      RLOG(1, "Simulation time step not >0: %g", dt);
    }
  }

  this->eRecover = false;
  this->dtSim = Timer_getSystemTime() - tmp;
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::updateGraph(RcsGraph* graph)
{
  if (this->threaded==false)
  {
    callback();
  }

  updateMtx.lock();
  MatNd_copy(graph->q, this->q_curr);
  MatNd_copy(graph->q_dot, this->q_dot_curr);

  if (sim->getGraph()->nSensors==graph->nSensors)
  {

    for (unsigned int i=0; i<graph->nSensors; ++i)
    {
      const RcsSensor* srcSensorPtr = &sim->getGraph()->sensors[i];
      RcsSensor* dstSensorPtr = &graph->sensors[i];
      MatNd_copy(dstSensorPtr->rawData, srcSensorPtr->rawData);
    }
  }
  else
  {
    RLOG(1, "Sensor update failed: Different number of sensors in graphs");
  }

  updateMtx.unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::updateGraphWithoutPhysics(RcsGraph* graph)
{
  updateMtx.lock();
  MatNd_copy(graph->q, this->q_curr);
  MatNd_copy(graph->q_dot, this->q_dot_curr);
  updateMtx.unlock();
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  MatNd* zeroVec = MatNd_create(this->q_curr->m, 1);
  sim->setControlInput(this->q_curr, zeroVec, zeroVec);
  MatNd_destroy(zeroVec);
  this->eStop = true;
  //this->enableCommands = false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
  this->eRecover = true;
  //this->enableCommands = true;
  setPositionCommand(this->q_curr);
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::setPositionCommand(const MatNd* q_des)
{
  if (this->eStop)
  {
    return;
  }

  if (this->enableCommands==false)
  {
    return;
  }

  if (sim)
  {
    MatNd* zeroVec = MatNd_create(q_des->m, 1);
    sim->setControlInput(q_des, zeroVec, zeroVec);
    MatNd_destroy(zeroVec);
  }
  else
  {
    MatNd_copy(this->q_curr, q_des);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::tare()
{
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string PhysicsComponent::getName() const
{
  return std::string("PhysicsSimulation");
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::getLastPositionCommand(MatNd* q_des) const
{
  sim->getLastPositionCommand(q_des);
}

/*******************************************************************************
 *
 ******************************************************************************/
Rcs::PhysicsBase* PhysicsComponent::getPhysicsSimulation() const
{
  return this->sim;
}

/*******************************************************************************
 *
 ******************************************************************************/
int PhysicsComponent::sprint(char* str, size_t size) const
{
  return snprintf(str, size, "Simulation time: %.3f (%.3f)\nStep took %.1f msec\n",
                  sim->time(), Timer_getSystemTime()-this->tStart, dtSim*1.0e3);
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::setFeedForward(bool ffwd_)
{
  this->ffwd = ffwd_;
}

/*******************************************************************************
 *
 ******************************************************************************/
double PhysicsComponent::getStartTime() const
{
  return this->tStart;
}

/*******************************************************************************
 *
 ******************************************************************************/
const RcsGraph* PhysicsComponent::getGraph() const
{
  return this->sim->getGraph();
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::start()
{
  if (this->threaded==true)
  {
    PeriodicCallback::start();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::stop()
{
  if (this->threaded==true)
  {
    PeriodicCallback::stop();
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onRandomPose()
{
  RLOG(0, "Setting random pose");
  MatNd* q_rnd = MatNd_create(q_curr->m, 1);
  MatNd_setRandom(q_rnd, -RCS_DEG2RAD(10.0), RCS_DEG2RAD(10.0));

  if (this->sim)
  {
    simMtx.lock();
    RcsGraph* graph = sim->getGraph();
    RcsGraph_setDefaultState(graph);
    MatNd_addSelf(graph->q, q_rnd);
    RcsGraph_limitJoints(graph, graph->q, RcsStateFull);
    updateMtx.lock();
    MatNd_copy(this->q_curr, graph->q);
    MatNd_setZero(this->q_dot_curr);
    updateMtx.unlock();
    RcsGraph_setState(graph, NULL, NULL);
    sim->reset();
    simMtx.unlock();
  }
  else
  {
    MatNd_copy(this->q_curr, q_rnd);
  }

  MatNd_destroy(q_rnd);
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "PhysicsComponent::onInitFromState()");
  updateMtx.lock();
  MatNd_copy(this->q_curr, target->q);
  MatNd_setZero(this->q_dot_curr);
  updateMtx.unlock();

  if (sim)
  {
    RcsGraph_setState(sim->getGraph(), target->q, target->q_dot);
    simMtx.lock();
    sim->reset();
    simMtx.unlock();
  }
}

/*******************************************************************************
 * The simulation locks the mutex inside the activate / deactivate methods.
 ******************************************************************************/
void PhysicsComponent::onObjectActivation(std::string objectName, bool enable)
{
  RLOG(1, "PhysicsComponent::onObjectActivation(%s,%s)",
       objectName.c_str(), enable ? "true" : "false");

  if (enable ==false)
  {
    sim->deactivateBody(objectName.c_str());
  }
  else
  {
    sim->activateBody(objectName.c_str());
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::setThreadedMode(bool enable)
{
  this->threaded = enable;
}

/*******************************************************************************
 * The simulation locks the mutex inside the activate / deactivate methods.
 ******************************************************************************/
bool PhysicsComponent::getThreadedMode() const
{
  return this->threaded;
}

/*******************************************************************************
 * Applies the impulse F to the given object at point p
 ******************************************************************************/
void PhysicsComponent::applyImpulse(const char* objName, const double F[3],
                                    const double p[3])
{
  if (!sim)
  {
    RLOG(4, "Can't apply impulse without physics simulation");
    return;
  }

  sim->applyImpulse(RcsGraph_getBodyByName(getGraph(), objName), F, p);
}

/*******************************************************************************
 * Applies the impulse F to the given object's center of mass
 ******************************************************************************/
void PhysicsComponent::applyImpulse(const char* objName, const double F[3])
{
  if (!sim)
  {
    RLOG(4, "Can't apply impulse without physics simulation");
    return;
  }

  sim->applyImpulse(RcsGraph_getBodyByName(getGraph(), objName), F, NULL);
}

/*******************************************************************************
 * Adds the given force F to the given object at point p
 ******************************************************************************/
void PhysicsComponent::addForce(const char* objName, const double F[3],
                                const double r[3])
{
  if (!sim)
  {
    RLOG(4, "Can't add force without physics simulation");
    return;
  }

  sim->setForce(RcsGraph_getBodyByName(getGraph(), objName), F, r);
}

/*******************************************************************************
 * Adds the given force F to the given object's center of mass
 ******************************************************************************/
void PhysicsComponent::addForce(const char* objName, const double F[3])
{
  if (!sim)
  {
    RLOG(4, "Can't add force without physics simulation");
    return;
  }

  sim->setForce(RcsGraph_getBodyByName(getGraph(), objName), F, NULL);
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onEnableCommands()
{
  RLOG(1, "PhysicsComponent::onEnableCommands()");
  this->enableCommands = true;
}

/*******************************************************************************
 *
 ******************************************************************************/
void PhysicsComponent::onResetRigidBodies()
{
  RLOG(1, "PhysicsComponent::onEnableCommands()");

  if (!sim)
  {
    RLOG(4, "Can't reset rigid bodies without physics simulation");
    return;
  }

  simMtx.lock();
  sim->resetRigidBodies();
  simMtx.unlock();

}

}   // namespace Dc
