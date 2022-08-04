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

#include "WheelPlannerComponent.h"
#include "EntityBase.h"

#include <Constraint1D.h>
#include <AStar.h>
#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <thread>
#include <algorithm>
#include <iterator>

using namespace tropic;

namespace Rcs
{


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v)
{
  if (!v.empty())
  {
    out << '[';
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

void WheelPlannerComponent::plannerThread(std::vector<int> from,
                                          std::vector<int> to,
                                          bool execute,
                                          double ttc)
{
  REXEC(0)
  {
    RMSG("plannerThread starts (execute: %s):", (execute ? "true" : "false"));
    RMSG_CPP("Start: " << from);
    RMSG_CPP("Goal: " << to);
  }

  wheelExplorer.setGoal(to);
  double startTime = Timer_getSystemTime();
  std::vector<std::vector<int>> solutionPath = Gras::Astar::search(wheelExplorer, from);
  bool goalReached = !solutionPath.empty();
  double endTime = Timer_getSystemTime();
  Gras::Astar::printSolution(wheelExplorer, solutionPath);
  RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
  RLOG(0, "Time needed: %4.3f", endTime - startTime);

  tSet->pruneSolution(solutionPath);

  if (execute)
  {
    if (goalReached)
    {
      getEntity()->publish("ExecutePlan", solutionPath, ttc);
    }
  }
  else
  {
    getEntity()->publish("PlanResult", solutionPath);
  }

}





WheelPlannerComponent::WheelPlannerComponent(EntityBase* parent,
                                             const ControllerBase* controller_,
                                             bool via,
                                             double horizon) :
  ComponentBase(parent), tc(NULL), tSet(NULL), motionEndTime(0.0),
  a_des(NULL), x_des(NULL), eStop(false), wheelExplorer(controller_->getGraph())
{
  ControllerBase* controller = new ControllerBase(*controller_);
  this->a_des   = MatNd_create((int) controller->getNumberOfTasks(), 1);
  this->x_des   = MatNd_create((int) controller->getTaskDim(), 1);
  controller->computeX(this->x_des);

  if (via)
  {
    this->tc = new TrajectoryController<ViaPointTrajectory1D>(controller, horizon);
  }
  else
  {
    this->tc = new TrajectoryController<ZigZagTrajectory1D>(controller, horizon);
  }
  //tc->readActivationsFromXML();
  tc->takeControllerOwnership(true);   // Deleted by tc destructor
  controller->readActivationsFromXML(this->a_des);
  for (unsigned int i=0; i<a_des->m; ++i)
  {
    tc->setActivation(i, (a_des->ele[i]==0.0) ? false : true);
  }

  this->tSet = new WheelConstraint(this->tc, &wheelExplorer, this->getEntity());

  this->searchState = wheelExplorer.getState(tc->getController()->getGraph());

  getEntity()->registerEvent<std::vector< std::vector<int> > >("PlanResult");
  subscribe<std::vector<int>,std::vector<int>, double>("PlanAndExecute", &WheelPlannerComponent::planAndExecute);
  subscribe<std::vector<int>,std::vector<int>, double>("Plan", &WheelPlannerComponent::plan);
  subscribe<int, int>("PlanToFlipAndExecute", &WheelPlannerComponent::onPlantoAngleAndExecute);
  subscribe<std::string, double>("MoveToState", &WheelPlannerComponent::onMoveToState);

  subscribe<double>("MoveToStartPose", &WheelPlannerComponent::moveToStartPose);
  subscribe<std::vector<int>, double>("MoveTo", &WheelPlannerComponent::onMoveTo);
  subscribe<std::vector<std::vector<int> >, double>("ExecutePlan", &WheelPlannerComponent::executePlan);
  subscribe("ClearTrajectory", &WheelPlannerComponent::onClearTrajectory);
  subscribe("EmergencyStop", &WheelPlannerComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &WheelPlannerComponent::onEmergencyRecover);
  subscribe<const RcsGraph*>("InitFromState", &WheelPlannerComponent::onInitFromState);
  subscribe<RcsGraph*>("ComputeTrajectory", &WheelPlannerComponent::stepTrajectory);
  subscribe("Print", &WheelPlannerComponent::print);
  subscribe<double>("InitializeMovement", &WheelPlannerComponent::onInitializeMovement);
  subscribe<double>("OpenHands", &WheelPlannerComponent::onOpenFingers);
  subscribe<double>("CloseHands", &WheelPlannerComponent::onCloseFingers);

  subscribe<double>("Disengage", &WheelPlannerComponent::onDisengage);
  subscribe<double>("Engage", &WheelPlannerComponent::onEngage);
  subscribe("Print", &WheelPlannerComponent::print);
}

WheelPlannerComponent::~WheelPlannerComponent()
{
  // delete this->tc->getController(); // now owned by tc
  delete this->tc;
  delete this->tSet;
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
}

void WheelPlannerComponent::print() const
{
  RLOG_CPP(0, "TrajectoryController has " << tc->getNumberOfSetConstraints() << " sets with "
           << tc->getNumberOfSetConstraints() << " constraints (Overall "
           << Constraint1D::getNumConstraints() << " constraints)");
}

std::string WheelPlannerComponent::getName() const
{
  return std::string("WheelPlannerComponent");
}

void WheelPlannerComponent::planAndExecute(std::vector<int> from,
                                           std::vector<int> to,
                                           double ttc)
{
  RLOG(0, "Plan::plan()");

  bool startValid = wheelExplorer.checkStateFeasible(from[0], from[1], from[2]);
  bool goalValid =  wheelExplorer.checkStateFeasible(to[0], to[1], to[2]);


  if (startValid && goalValid)
  {
    std::thread t1(&WheelPlannerComponent::plannerThread,
                   this, from, to, true, ttc);
    t1.detach();
  }
  else
  {
    RLOG(0, "Start state is %s, goal state is %s",
         startValid ? "VALID" : "INVALID", goalValid ? "VALID" : "INVALID");

    for (size_t i=0; i<from.size(); ++i)
    {
      std::cout << from[i] << " ";
    }
    std::cout << std::endl;

    for (size_t i=0; i<to.size(); ++i)
    {
      std::cout << to[i] << " ";
    }
    std::cout << std::endl;

    std::vector<std::vector<int> > solutionPath;


    getEntity()->publish("PlanResult", solutionPath);
  }
}

void WheelPlannerComponent::plan(std::vector<int> from,
                                 std::vector<int> to, double ttc)
{
  RLOG(0, "Plan::plan()");

  bool startValid = wheelExplorer.checkStateFeasible(from[0], from[1], from[2]);
  bool goalValid =  wheelExplorer.checkStateFeasible(to[0], to[1], to[2]);

  if (startValid && goalValid)
  {
    std::thread t1(&WheelPlannerComponent::plannerThread,
                   this, from, to, false, ttc);
    t1.detach();
  }
  else
  {
    RLOG(0, "Start state is %s, goal state is %s",
         startValid ? "VALID" : "INVALID", goalValid ? "VALID" : "INVALID");

    for (size_t i=0; i<from.size(); ++i)
    {
      std::cout << from[i] << " ";
    }
    std::cout << std::endl;

    for (size_t i=0; i<to.size(); ++i)
    {
      std::cout << to[i] << " ";
    }
    std::cout << std::endl;

    std::vector<std::vector<int> > solutionPath;
    getEntity()->publish("PlanResult", solutionPath);
  }
}



void WheelPlannerComponent::onPlantoAngleAndExecute(int s, int flip)
{
  RLOG(0, "WheelPlannerComponent::onPlanAndExecute()");

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping plan",
         motionEndTime);
    return;
  }

  std::vector<int> from = wheelExplorer.getState(tc->getController()->getGraph());
  std::vector<int> to = from;
  to[0] = s;
  to[1] = flip;

  std::thread t1(&WheelPlannerComponent::plannerThread,
                 this, from, to, true, 5.0);
  t1.detach();
}


void WheelPlannerComponent::onMoveToState(std::string to, double ttc)
{
  RLOG(0, "WheelPlannerComponent::onMoveToState(%s)", to.c_str());

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping new move",
         motionEndTime);
    return;
  }

  double dTo[WheelStrategy7D::StateMaxIndex];
  bool success = String_toDoubleArray_l(to.c_str(), dTo, WheelStrategy7D::StateMaxIndex);
  RCHECK(success);

  std::vector<int> vecTo;
  for (size_t i = 0; i<WheelStrategy7D::StateMaxIndex; ++i)
  {
    vecTo.push_back((int) lround(dTo[i]));
  }

  bool goalValid = wheelExplorer.checkState(vecTo, false);
  if (goalValid == false)
  {

    REXEC(0)
    {
      printf("Size: %5.2f %d\n", dTo[6], (int) vecTo.size());
      for (size_t i = 0; i < vecTo.size(); i++)
      {
        std::cout << vecTo[i] << " ";
      }
      std::cout << std::endl;
      RLOG(0, "Goal state is INVALID - skipping move");
    }
    return;
  }

  tc->clear();
  auto moveSet = tSet->moveTo(vecTo, ttc);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}


void WheelPlannerComponent::onMoveTo(std::vector<int> state, double ttc)
{
  tc->clear();
  auto moveSet = tSet->moveTo(state, ttc);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}


void WheelPlannerComponent::moveToStartPose(double ttc)
{
  RLOG(0, "MoveToStartPose::moveToStartPose");

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Moving for %g sec more - skipping moveToStartPose",
         motionEndTime);
    return;
  }

  std::vector<int> currentState = wheelExplorer.getState(tc->getController()->getGraph());
  RLOG(0, "Current state: %s", Gras::vecToStr(currentState).c_str());

  ttc = 10.0;
  RLOG(0, "Overwriting ttc for moving to start: %5.2f sec", ttc);
  onInitializeMovement(ttc);
}

void WheelPlannerComponent::executePlan(std::vector<std::vector<int> > solutionPath, double ttc)
{

  if (solutionPath.empty())
  {
    RLOG(0, "Couldn't find solution");
  }
  else
  {
    //    RcsGraph* g = RcsGraph_clone(tc->getController()->getGraph());
    //    broadcastHoloPlan(solutionPath, g, this, this->explorer);

    // Publish plan to trajectory generators
    RLOG_CPP(5, "Set has " << tc->getNumberOfSetConstraints()
             << " constraints before clearing it");
    tc->clear();
    double dtAdd = Timer_getSystemTime();
    RLOG(5, "Adding solution with %d steps", (int) solutionPath.size());

    auto sln = tSet->addSolution(solutionPath, ttc);

    if (!sln)
    {
      RLOG(1, "Failed to generate trajectory");
      return;
    }

    tc->add(sln);

    dtAdd = Timer_getSystemTime() - dtAdd;
    RLOG(5, "... addSolution took %.2f msec", 1.0e3*dtAdd);
    dtAdd = Timer_getSystemTime();
    sln->apply(tc->getTrajectoriesRef());
    dtAdd = Timer_getSystemTime() - dtAdd;
    RLOG(5, "... done, apply took %.2f msec, now rootSet has %d sets and %d "
         "constraints. ", 1.0e3*dtAdd, tc->getNumberOfSets(),
         tc->getNumberOfSetConstraints());
  }
}
void WheelPlannerComponent::stepTrajectory(RcsGraph* from)
{
  RcsGraph_setState(tc->getInternalController()->getGraph(), from->q, from->q_dot);
  double lastMotionEndTime = this->motionEndTime;
  this->motionEndTime = tc->step(getEntity()->getDt());

  tc->getPosition(0.0, x_des);
  tc->getActivation(a_des);

  char text[256];
  std::vector<int> stateValues = wheelExplorer.getState(tc->getInternalController()->getGraph());
  snprintf(text, 256, "Motion time: %5.2f, State: %s",   this->motionEndTime,
           Gras::vecToStr(stateValues).c_str());
  getEntity()->publish("SetTextLine", std::string(text), 2);

  this->searchState = wheelExplorer.getState(tc->getController()->getGraph());
  double currentMotionEndTime = this->motionEndTime;

  if (lastMotionEndTime <= 0.00001 && currentMotionEndTime > 0.0001)
  {
    getEntity()->publish("MotionState", true);
  }
  else if (lastMotionEndTime > 0.0001 && currentMotionEndTime <= 0.0001)
  {
    getEntity()->publish("MotionState", false);
  }

}

void WheelPlannerComponent::onClearTrajectory()
{
  RLOG(0, "WheelPlannerComponent::clearTrajectory()");
  tc->clear();
}

const MatNd* WheelPlannerComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* WheelPlannerComponent::getTaskCommandPtr() const
{
  return this->x_des;
}

void WheelPlannerComponent::onEmergencyStop()
{
  if (this->eStop == false)
  {
    RLOG(0, "WheelPlannerComponent::EmergencyStop");
    tc->clear();
  }

  this->eStop = true;
}

void WheelPlannerComponent::onEmergencyRecover()
{
  RLOG(0, "WheelPlannerComponent::EmergencyRecover");
  this->eStop = false;
}

void WheelPlannerComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "WheelPlannerComponent::onInitFromState()");
  RcsGraph_setState(tc->getInternalController()->getGraph(), target->q, target->q_dot);
  tc->init();
  tc->clear();
}

void WheelPlannerComponent::onInitializeMovement(double ttc)
{
  RMSGS("Initializing movement");
  auto moveSet = tSet->createInitMove(ttc);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}

void WheelPlannerComponent::onOpenFingers(double ttc)
{
  RMSGS("Opening hands");
  auto moveSet = tSet->createFingersOpenMove(ttc);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}

void WheelPlannerComponent::onCloseFingers(double ttc)
{
  RMSGS("Closing hands");
  auto moveSet = tSet->createFingersClosedMove(ttc);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}

void WheelPlannerComponent::onEngage(double ttc)
{
  RMSGS("Engaging");
  auto moveSet = tSet->createEngageMove(ttc, 0.0);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}

void WheelPlannerComponent::onDisengage(double ttc)
{
  RMSGS("Disengaging");
  auto moveSet = tSet->createEngageMove(ttc, 0.1);
  moveSet->apply(tc->getTrajectoriesRef());
  tc->add(moveSet);
}

double WheelPlannerComponent::getMotionEndTime() const
{
  return this->motionEndTime;
}

std::vector<int> WheelPlannerComponent::getState() const
{
  return this->searchState;
}

std::vector<double> WheelPlannerComponent::getTrajectoryVel(const std::string& trajName) const
{
  const TrajectoryND* traj = tc->getTrajectory(trajName);
  std::vector<double> vel;
  RCHECK(traj);

  for (size_t i = 0; i < traj->getDim(); ++i)
  {
    double x, xd, xdd;
    traj->getTrajectory1D(i)->computeTrajectoryPoint(x, xd, xdd, 0.0);
    vel.push_back(xd);
  }

  return vel;
}

std::vector<double> WheelPlannerComponent::getComplianceWrench() const
{
  std::vector<double> xd_r = getTrajectoryVel("RobotRight");
  std::vector<double> xd_l = getTrajectoryVel("RobotLeft");

  std::vector<double> complianceHigh(12, 0.0);
  complianceHigh[0] = 300.0;
  complianceHigh[1] = 300.0;
  complianceHigh[2] = 900.0;
  complianceHigh[3] = 30.0;
  complianceHigh[4] = 30.0;
  complianceHigh[5] = 30.0;


  std::vector<double> complianceLow(12, 0.0);
  complianceLow[0] = 5000.0;
  complianceLow[1] = 5000.0;
  complianceLow[2] = 5000.0;
  complianceLow[3] = 300.0;
  complianceLow[4] = 300.0;
  complianceLow[5] = 300.0;

  for (int i = 0; i < 6; i++)
  {
    complianceHigh[i+6] = complianceHigh[i];
    complianceLow[i+6]  = complianceLow[i];
  }

  double complianceVelocityMax = 0.08;

  double xd[3];
  Vec3d_add(xd, xd_r.data(), xd_l.data());
  double handTaskVel = Vec3d_sqrLength(xd);

  double ratio = (std::min)(handTaskVel, complianceVelocityMax) / complianceVelocityMax;

  std::vector<double> complianceWrench(12, 0.0);

  for (size_t i = 0; i < complianceWrench.size(); i++)
  {
    complianceWrench[i] = complianceLow[i]*ratio + complianceHigh[i]*(1.0-ratio);
  }

  return complianceWrench;
}

const WheelStrategy7D* WheelPlannerComponent::getStrategy() const
{
  return &wheelExplorer;
}


}   // namespace Rcs
