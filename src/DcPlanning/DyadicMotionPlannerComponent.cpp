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

#include "DyadicMotionPlannerComponent.h"
#include "EntityBase.h"

#include <MultiGoalConstraint.h>
#include <AStar.h>
#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <thread>

namespace Rcs
{

void DyadicMotionPlannerComponent::plannerThread(std::vector<int> from,
                                                 std::vector<int> to,
                                                 bool execute,
                                                 double ttc)
{
  REXEC(0)
  {
    RLOG(0, "plannerThread starts (execute: %s):", execute ? "true" : "false");

    for (size_t i = 0; i < from.size(); ++i)
    {
      std::cout << from[i] << " ";
    }
    std::cout << std::endl;

    for (size_t i = 0; i < to.size(); ++i)
    {
      std::cout << to[i] << " ";
    }

    std::cout << std::endl << "Delta phi is "
              << RCS_RAD2DEG(explorer.getDeltaPhi());
    std::cout << std::endl << "Friction angle is "
              << RCS_RAD2DEG(explorer.getFrictionAngle(0)) << std::endl;
  }

  RLOG(0, "Start planning");
  explorer.setGoal(to);
  double startTime = Timer_getSystemTime();
  std::vector<std::vector<int>> solutionPath = Gras::Astar::search(explorer, from);
  bool goalReached = !solutionPath.empty();
  double endTime = Timer_getSystemTime();
  Gras::Astar::printSolution(explorer, solutionPath);
  RLOG(0, "%s finishing planning", goalReached ? "SUCCESS" : "FAILURE");
  RLOG(0, "Time needed: %4.3f", endTime - startTime);

  if (execute)
  {
    if (goalReached)
    {
      getEntity()->publish("ExecutePlan", solutionPath, ttc);
    }
  }
  else
  {
    // Publish plan solution
    getEntity()->publish("PlanResult", solutionPath);
  }

}





DyadicMotionPlannerComponent::DyadicMotionPlannerComponent(EntityBase* parent,
                                                           const ControllerBase* controller_,
                                                           double deltaPhi,
                                                           bool via,
                                                           double horizon) :
  ComponentBase(parent), tc(NULL), tSet(NULL), explorer(BoxStrategy5D::Box, 1),
  motionEndTime(0.0),
  lastMotionEndTime(0.0), a_des(NULL), x_des(NULL), eStop(false)
{
  ControllerBase* controller = new ControllerBase(*controller_);
  this->a_des   = MatNd_create((int) controller->getNumberOfTasks(), 1);
  this->x_des   = MatNd_create((int) controller->getTaskDim(), 1);
  controller->computeX(this->x_des);

  if (via)
  {
    this->tc = new tropic::TrajectoryController<tropic::ViaPointTrajectory1D>(controller, horizon);
  }
  else
  {
    this->tc = new tropic::TrajectoryController<tropic::ZigZagTrajectory1D>(controller, horizon);
  }
  tc->readActivationsFromXML();
  for (unsigned int i=0; i<a_des->m; ++i)
  {
    tc->setActivation(i, (tc->getActivation(i)>0.0) ? true : false);
  }

  int nStepsAround = (int) lround(2.0*M_PI/deltaPhi);
  deltaPhi = 2.0*M_PI / (double)nStepsAround;
  explorer.setNumPhiDiscretizations(nStepsAround);
  explorer.setMinimumHandDistance(0.12);
  // this->explorer.setMaxLever(1.0);
  // this->explorer.setFrictionAngle(RCS_DEG2RAD(35.0));




  this->tSet = new DyadicTrajectorySet(this->tc, deltaPhi);
  tSet->setContactPoints(explorer.getRoboContacts3D(),
                         explorer.getPartnerContacts3D());
  //  tc->rootSet.add(tSet);


  getEntity()->registerEvent<std::vector< std::vector<int> > >("PlanResult");
  subscribe<std::vector<int>,std::vector<int>, double>("PlanAndExecute", &DyadicMotionPlannerComponent::planAndExecute);
  subscribe<double, double>("PlanToAngleAndExecute", &DyadicMotionPlannerComponent::planToAngleAndExecute);
  subscribe<std::vector<int>,std::vector<int>, double>("Plan", &DyadicMotionPlannerComponent::plan);
  subscribe<double, double>("PlanToAngle", &DyadicMotionPlannerComponent::planToAngle);
  subscribe<double>("MoveToStartPose", &DyadicMotionPlannerComponent::moveToStartPose);
  subscribe<std::vector<int>, double>("MoveTo", &DyadicMotionPlannerComponent::moveTo);
  subscribe<int, int, int, double>("MoveRobotTo", &DyadicMotionPlannerComponent::moveRobotTo);
  subscribe<std::vector<std::vector<int> >, double>("ExecutePlan", &DyadicMotionPlannerComponent::executePlan);
  subscribe<>("ClearTrajectory", &DyadicMotionPlannerComponent::onClearTrajectory);
  subscribe<double>("StopMovement", &DyadicMotionPlannerComponent::stopTrajectory);
  subscribe<>("EmergencyStop", &DyadicMotionPlannerComponent::onEmergencyStop);
  subscribe<>("EmergencyRecover", &DyadicMotionPlannerComponent::onEmergencyRecover);
  subscribe<const RcsGraph*>("InitFromState", &DyadicMotionPlannerComponent::onInitFromState);
  subscribe<RcsGraph*>("ComputeTrajectory", &DyadicMotionPlannerComponent::stepTrajectory);
  subscribe<>("Print", &DyadicMotionPlannerComponent::print);

  subscribe<std::string, std::string>("PlanFromTo", &DyadicMotionPlannerComponent::onPlanFromTo);
  subscribe<std::string, double>("PlanToStateAndExecute", &DyadicMotionPlannerComponent::onPlanToStateAndExecute);
  subscribe<>("ToggleSlideMode", &DyadicMotionPlannerComponent::onToggleSlideMode);
  subscribe<std::string, tropic::ViaPointTrajectoryPosition**>("ProvideTrajectory",
                                                               &DyadicMotionPlannerComponent::onProvideTrajectoryPtr);
  subscribe<std::string>("ChangeObjectShape", &DyadicMotionPlannerComponent::onSelectObject);
  subscribe<std::string, double>("MoveToState", &DyadicMotionPlannerComponent::onMoveToState);
}

DyadicMotionPlannerComponent::~DyadicMotionPlannerComponent()
{
  delete this->tc->getController();
  delete this->tc;
  delete this->tSet;
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
}

void DyadicMotionPlannerComponent::print() const
{
  //RcsGraph_fprintModelState(stdout, tc->getController()->getGraph(),
  //                          tc->getController()->getGraph()->q);

  RLOG(0, "End time: %f", this->motionEndTime);

}

std::string DyadicMotionPlannerComponent::getName() const
{
  return std::string("DyadicMotionPlannerComponent");
}

std::vector<std::vector<int>> DyadicMotionPlannerComponent::planSynchronous(std::vector<int> from, std::vector<int> to)
{
  RLOG(0, "Plan::planSynchronous()");
  explorer.setGoalCondition(BoxStrategy5D::FullState);

  bool startValid = explorer.checkState(from);
  bool goalValid = explorer.checkState(to);

  std::vector<std::vector<int> > solutionPath;

  if (startValid && goalValid)
  {
    RLOG(0, "Start planning");
    explorer.setGoal(to);
    solutionPath = Gras::Astar::search(explorer, from);
    bool goalReached = !solutionPath.empty();
    RLOG(0, "Finished planning");

    if (goalReached==false)
    {
      RLOG(0, "Couldn't find solution");
    }
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

  }

  return solutionPath;
}

void DyadicMotionPlannerComponent::onPlanFromTo(std::string from,
                                                std::string to)
{
  RLOG(0, "Planning from \"%s\" to \"%s\"", from.c_str(), to.c_str());

  double dFrom[5], dTo[5];
  bool success = String_toDoubleArray_l(from.c_str(), dFrom, 5);

  if (success==false)
  {
    RLOG(0, "Wrong start state: \"%s\" - did you specify 5 values?",
         from.c_str());
  }

  success = String_toDoubleArray_l(to.c_str(), dTo, 5);

  if (success==false)
  {
    RLOG(0, "Wrong goal state: \"%s\" - did you specify 5 values?",
         to.c_str());
  }

  std::vector<int> vecFrom, vecTo;
  for (size_t i=0; i<5; ++i)
  {
    vecFrom.push_back((int) lround(dFrom[i]));
    vecTo.push_back((int) lround(dTo[i]));
  }

  std::vector<std::vector<int>> sln = planSynchronous(vecFrom, vecTo);
}

void DyadicMotionPlannerComponent::plan(std::vector<int> from,
                                        std::vector<int> to, double ttc)
{
  RLOG(0, "Plan::plan()");
  explorer.setGoalCondition(BoxStrategy5D::FullState);

  bool startValid = explorer.checkState(from);
  bool goalValid = explorer.checkState(to);

  if (startValid && goalValid)
  {
    std::thread t1(&DyadicMotionPlannerComponent::plannerThread,
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

  }
}

void DyadicMotionPlannerComponent::planToAngle(double phi, double ttc)
{
  RLOG(0, "Planning to angle: %.2f", phi);
  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping plan",
         motionEndTime);
    return;
  }

  std::vector<int> from = tSet->getState();
  std::vector<int> to = from;
  to[0] = explorer.getPhiFromAngle(phi);

  RLOG(0, "PlanToAngle::planToAngle(%.1f deg, %.1f s)", RCS_RAD2DEG(phi), ttc);
  explorer.setGoalCondition(BoxStrategy5D::RotationWithMaxStableHold);
  std::thread t1(&DyadicMotionPlannerComponent::plannerThread,
                 this, from, to, false, ttc);
  t1.detach();
}

void DyadicMotionPlannerComponent::planAndExecute(std::vector<int> from,
                                                  std::vector<int> to,
                                                  double ttc)
{
  RLOG(0, "Plan::plan()");
  explorer.setGoalCondition(BoxStrategy5D::FullState);

  bool startValid = explorer.checkState(from);
  bool goalValid = explorer.checkState(to);

  if (startValid && goalValid)
  {
    std::thread t1(&DyadicMotionPlannerComponent::plannerThread,
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

  }
}

void DyadicMotionPlannerComponent::planToAngleAndExecute(double phi, double ttc)
{
  RLOG(0, "DyadicMotionPlannerComponent::planToAngleAndExecute(%.2f deg)", RCS_RAD2DEG(phi));

  RLOG(0, "Constraints: %d (%d)", (int)tc->getNumberOfSetConstraints(),
       (int)tropic::Constraint1D::getNumConstraints());


  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping plan",
         motionEndTime);
    return;
  }

  std::vector<int> from = tSet->getState();
  std::vector<int> to = from;
  to[0] = (int) lround(phi/explorer.getDeltaPhi());

  RLOG(0, "PlanToAngle::planToAngleAndExecute(%.1f deg, %.1f s)",
       RCS_RAD2DEG(phi), ttc);
  explorer.setGoalCondition(BoxStrategy5D::RotationOnly);//RotationWithMaxStableHold);
  std::thread t1(&DyadicMotionPlannerComponent::plannerThread,
                 this, from, to, true, ttc);
  t1.detach();
}

void DyadicMotionPlannerComponent::onPlanToStateAndExecute(std::string to,
                                                           double ttc)
{
  RLOG(0, "DyadicMotionPlannerComponent::planToStateAndExecute(%s)",
       to.c_str());

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping plan",
         motionEndTime);
    return;
  }

  std::vector<int> from = tSet->getState();
  double dTo[5];
  bool success = String_toDoubleArray_l(to.c_str(), dTo, 5);
  RCHECK(success);

  std::vector<int> vecTo;
  for (size_t i = 0; i<5; ++i)
  {
    vecTo.push_back((int) lround(dTo[i]));
  }

  bool goalValid = explorer.checkState(vecTo);
  if (goalValid == false)
  {

    REXEC(0)
    {
      for (size_t i=0; i<vecTo.size(); ++i)
      {
        std::cout << vecTo[i] << " ";
      }
      std::cout << std::endl;
      RLOG(0, "Goal state is INVALID - skipping plan");
    }
    return;
  }

  explorer.setGoalCondition(BoxStrategy5D::FullState);
  std::thread t1(&DyadicMotionPlannerComponent::plannerThread,
                 this, from, vecTo, true, ttc);
  t1.detach();
}

void DyadicMotionPlannerComponent::moveToStartPose(double ttc)
{
  RLOG(0, "MoveToStartPose::moveToStartPose");

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Moving for more %g sec - skipping moveToStartPose",
         motionEndTime);
    return;
  }


  // TODO: this needs to be checked for others shapes than boxes...
  std::vector<int> currentState = this->getCurrentState();
  RLOG(0, "Current state: %d, %d, %d, %d, %d",
       currentState[0], currentState[1], currentState[2], currentState[3], currentState[4]);
  if (currentState[0] != 0)
  {
    RMSGS("Moving to closest canonical box pose with support");

    int canonPhi = (int) lround(getNextCanonicalAngle(currentState[0] * getDeltaPhi(), 0) / getDeltaPhi());
    std::vector<int> goalState = getStableHandSupports(canonPhi);

    //tSet->moveTo(goalState[0], goalState[1], goalState[2], goalState[3], goalState[4], ttc);
    auto moveSet = tSet->createMoveToSet(goalState[0], goalState[1], goalState[2], goalState[3], goalState[4], ttc);
    if (moveSet)
    {
      tc->add(moveSet);
      moveSet->apply(tc->getTrajectoriesRef());
    }

    return;
  }

  switch (explorer.getObjectType())
  {
    case BoxStrategy5D::ObjectType::Box:
    {
      RLOG(0, "Moving to initial box pose");
      //tSet->moveTo(0, 14, 10, 10, 14, ttc);
      auto moveSet = tSet->createMoveToSet(0, 14, 10, 10, 14, ttc);
      if (moveSet)
      {
        tc->add(moveSet);
        moveSet->apply(tc->getTrajectoriesRef());
      }
    }
    break;

    case BoxStrategy5D::ObjectType::LShape:
    {
      RLOG(0, "Moving to initial L-shape pose");
      //tSet->moveTo(0, 15, 12, 12, 15, ttc);
      auto moveSet = tSet->createMoveToSet(0, 15, 12, 12, 15, ttc);
      if (moveSet)
      {
        tc->add(moveSet);
        moveSet->apply(tc->getTrajectoriesRef());
      }
    }
    break;

    case BoxStrategy5D::ObjectType::Cylinder:
    {
      RLOG(0, "Moving to initial cylinder pose");
      //tSet->moveTo(0, 14, 11, 11, 14, ttc);
      auto moveSet = tSet->createMoveToSet(0, 14, 11, 11, 14, ttc);
      if (moveSet)
      {
        tc->add(moveSet);
        moveSet->apply(tc->getTrajectoriesRef());
      }
    }
    break;

    default:
      RLOG(0, "Unknown object type: %d", explorer.getObjectType());
  }
}

void DyadicMotionPlannerComponent::moveTo(std::vector<int> to, double ttc)
{
  if (to.size() < 5)
  {
    RLOG(0, "State has too few elements %d, expected 5.", (int) to.size());
    return;
  }

  auto moveSet = tSet->createMoveToSet(to[0], to[1], to[2], to[3], to[4], ttc);
  if (moveSet)
  {
    tc->clear();
    tc->add(moveSet);
    moveSet->apply(tc->getTrajectoriesRef());
  }

}

void DyadicMotionPlannerComponent::moveRobotTo(int phi, int ra, int la,
                                               double ttc)
{
  auto moveSet = tSet->createMoveRobotToSet(phi, ra, la, ttc);
  if (moveSet)
  {
    moveSet->apply(tc->getTrajectoriesRef());
    tc->add(moveSet);
  }
}

void DyadicMotionPlannerComponent::executePlan(std::vector<std::vector<int> > solutionPath, double ttc)
{
  if (solutionPath.empty())
  {
    RLOG(0, "Couldn't find solution");
  }
  else
  {
    // Publish plan to trajectory generators
    RLOG(5, "Set has %d constraints before clearing it",
         (int) tc->getNumberOfSetConstraints());
    tc->clear();
    double dtAdd = Timer_getSystemTime();
    RLOG(5, "Adding solution with %d steps", (int) solutionPath.size());
    auto moveSet = tSet->createSolutionSet(solutionPath, ttc);

    if (!moveSet)
    {
      RLOG(1, "Failed to generate trajectory - giving up");
      return;
    }

    dtAdd = Timer_getSystemTime() - dtAdd;
    RLOG(5, "... addSolution took %.2f msec", 1.0e3*dtAdd);
    dtAdd = Timer_getSystemTime();

    REXEC(5)
    {
      moveSet->toXML("ConstraintSet.xml");
    }

    auto tVec = tc->getTrajectoriesRef();
    moveSet->apply(tVec);
    dtAdd = Timer_getSystemTime() - dtAdd;
    tc->add(moveSet);
    RLOG(0, "... done, apply took %.2f msec, now rootSet has %d sets and %d "
         "constraints.", 1.0e3*dtAdd,
         (int)tc->getNumberOfSets(),
         (int)tc->getNumberOfSetConstraints());
  }
}

void DyadicMotionPlannerComponent::computeControl(MatNd* a, MatNd* x)
{
  const double dt_step = getEntity()->getDt();

  this->motionEndTime = tc->step(dt_step);
  tc->getPosition(0.0, x);
  tc->getActivation(a);

  char text[256];
  std::vector<int> stateValues = getCurrentState();
  snprintf(text, 256, "Motion time: %5.2f, State: %s\n"
           "Constraints: Static count: %d, TrajND: %d %d",
           this->motionEndTime,
           Gras::vecToStr(stateValues).c_str(),
           (int)tc->getNumberOfSetConstraints(),
           (int)tropic::Constraint1D::getNumConstraints(),
           (int)tc->getNumberOfConstraints());
  getEntity()->publish("SetTextLine", std::string(text), 2);

  double currentMotionEndTime = this->motionEndTime;

  //TODO: maybe manage MotionState inside tSet
  if (lastMotionEndTime <= 0.00001 && currentMotionEndTime > 0.0001)
  {
    getEntity()->publish("MotionState", true);
  }
  else if (lastMotionEndTime > 0.0001 && currentMotionEndTime <= 0.0001)
  {
    getEntity()->publish("MotionState", false);
  }

  this->lastMotionEndTime = currentMotionEndTime;
}

const ControllerBase* DyadicMotionPlannerComponent::getController() const
{
  return tc->getController();
}

void DyadicMotionPlannerComponent::onClearTrajectory()
{
  RLOG(0, "DyadicMotionPlannerComponent::clearTrajectory()");
  tc->clear();
}

void DyadicMotionPlannerComponent::onStopAtNextGoal()
{
  RLOG(0, "DyadicMotionPlannerComponent::stopTrajectory()");
  std::vector<tropic::TrajectoryND*> tvec = tc->getTrajectoriesRef();
  for (size_t i = 0; i < tvec.size(); ++i)
  {
    tvec[i]->removeConstraintsAfterFirstGoal();
  }
  tc->clearConstraintSet();
}

void DyadicMotionPlannerComponent::stopTrajectory(double dt)
{
  RLOG(0, "DyadicMotionPlannerComponent::stopTrajectory()");
  onStopAtNextGoal();
  return;

  MatNd* stopPos = MatNd_create(tc->getDim(), 1);
  double endTime = std::min(dt, this->motionEndTime);
  tc->getPosition(endTime, stopPos);
  tc->clear();

  std::vector<tropic::TrajectoryND*> tvec = tc->getTrajectoriesRef();
  for (size_t i = 0; i < tvec.size(); ++i)
  {
    tvec[i]->clear();
  }

  auto mc = std::make_shared<tropic::MultiGoalConstraint>(dt, stopPos,
                                                          tc->getTrajectoriesRef());

  tc->add(mc);
  mc->apply(tc->getTrajectoriesRef());

  MatNd_destroy(stopPos);
}

int DyadicMotionPlannerComponent::getNumPhiDiscretizations() const
{
  return this->explorer.getNumPhiDiscretizations();
}

double DyadicMotionPlannerComponent::getDeltaPhi() const
{
  return this->explorer.getDeltaPhi();
}

HTr DyadicMotionPlannerComponent::getRoboHandPose(unsigned int idx) const
{
  std::vector<HTr> contacts = this->explorer.getRoboContacts3D();

  if (idx >= contacts.size())
  {
    //TODO: maybe better handling here?
    RFATAL("Invalid contact location idx requested: %d", idx);
  }

  return contacts[idx];
}

double DyadicMotionPlannerComponent::getNextCanonicalAngle(double startAngle,
                                                           int direction) const
{
  // get nearest 90-degree multiple of object orientation
  // \todo: 'canonical' or 'likely goal/sub-goal' states probably need to be
  //        defined on a per-object basis, and should be informed through human
  //        demonstrations.
  unsigned int numCanonicalStates = 4;

  //  int anglePhi = (int) floor(startAngle/getDeltaPhi());
  //  RLOG(0, "======================");
  //  RLOG(1, "Get next canonic state for angle %5.2f (%d) in direction %d",
  //       startAngle, anglePhi, direction);

  //  if (direction==0)
  //  {
  //    RLOG(0, "This should never happen.");
  //    return startAngle;
  //  }

  std::vector<double> canocicalAngles;
  canocicalAngles.reserve(numCanonicalStates);
  double stepSize = 2.0 * M_PI / numCanonicalStates;
  for (unsigned int i = 0; i < numCanonicalStates; i++)
  {
    canocicalAngles.push_back(i * stepSize);
  }

  // current angle between 0 and 360
  double baseAngle = fmod(startAngle, 2.0*M_PI);//(startAngle - rotations * 2 * M_PI);
  if (baseAngle < 0.0)
  {
    baseAngle += 2.0*M_PI;
  }

  // current number of rotations
  int rotations = (int) lround(floor(startAngle / (2.0*M_PI)));

  //  RLOG(0, "Rotations: %d (%5.2f, %5.2f), base: %5.2f",
  //rotations, (startAngle) / (2*M_PI), floor((startAngle) / (2*M_PI)), baseAngle);

  // determine closest canonicalState (index)
  int minIdx = -1;
  double minDist = fabs(baseAngle - (2.0 * M_PI));

  for (int i = 0; i < (int) canocicalAngles.size(); i++)
  {
    double dist = fabs(baseAngle - canocicalAngles[i]);
    if (dist < minDist)
    {
      minDist = dist;
      minIdx = i;
    }
  }
  if (minIdx == -1)
  {
    minIdx = 0;
    rotations += 1;
  }

  int closestCanonicState = minIdx;

  //  double closestCanonicalStateAngle = rotations * 2 * M_PI + canocicalAngles[closestCanonicState];
  //  RLOG(0, "   Closest canonical state is %5.2f (%d)",
  //closestCanonicalStateAngle, (int) floor(closestCanonicalStateAngle/getDeltaPhi()));

  if (direction != 0)
  {
    // adjust state idx in desired direction
    closestCanonicState += direction;

    while (closestCanonicState < 0)
    {
      closestCanonicState += (int) canocicalAngles.size();
      rotations--;
    }
    while (closestCanonicState >= (int) canocicalAngles.size())
    {
      closestCanonicState -= (int) canocicalAngles.size();
      rotations++;
    }
  }

  double nextCanonicalStateAngle = rotations * 2 * M_PI + canocicalAngles[closestCanonicState];

  //  RLOG(0, "   Idx: %d, min: %d", closestCanonicState, minIdx);
  //  RLOG(0, "   Next canonical state is    %5.2f (%d)",
  //nextCanonicalStateAngle, (int) floor(nextCanonicalStateAngle/getDeltaPhi()));

  return nextCanonicalStateAngle;
}

/*******************************************************************************
 * Stop current movement along current trajectory after specified time.
 ******************************************************************************/
void DyadicMotionPlannerComponent::stopMove(double deltaTime)
{
  RWARNING(0, "Not implemented...");
}

bool DyadicMotionPlannerComponent::isStateValid(std::vector<int> state) const
{
  return explorer.checkState(state);
}

std::vector<int> DyadicMotionPlannerComponent::getCurrentState() const
{
  return tSet->getState();
}

const MatNd* DyadicMotionPlannerComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* DyadicMotionPlannerComponent::getTaskCommandPtr() const
{
  return this->x_des;
}

void DyadicMotionPlannerComponent::stepTrajectory(RcsGraph* from)
{
  RcsGraph_setState(tc->getController()->getGraph(), from->q, from->q_dot);
  computeControl(this->a_des, this->x_des);
}

void DyadicMotionPlannerComponent::onEmergencyStop()
{
  RLOG(0, "DyadicMotionPlannerComponent::EmergencyStop");
  tc->clear();
  this->eStop = true;
}

void DyadicMotionPlannerComponent::onEmergencyRecover()
{
  RLOG(0, "DyadicMotionPlannerComponent::EmergencyRecover");
  this->eStop = false;
}

void DyadicMotionPlannerComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "DyadicMotionPlannerComponent::onInitFromState()");
  RcsGraph_setState(tc->getController()->getGraph(), target->q, target->q_dot);
  tc->init();
  tc->clear();
}

void DyadicMotionPlannerComponent::onToggleSlideMode()
{
  RLOG(0, "setting slideMode %s", tSet->getSlideMode() ? "OFF" : "ON");
  tSet->setSlideMode(!tSet->getSlideMode());
}

void DyadicMotionPlannerComponent::onProvideTrajectoryPtr(std::string taskName,
                                                          tropic::ViaPointTrajectoryPosition** dst)
{
  tropic::ViaPointTrajectoryPosition* ti = dynamic_cast<tropic::ViaPointTrajectoryPosition*>(tc->getTrajectory(taskName));

  if (ti != NULL)
  {
    *dst = ti;
  }

}

std::vector<int> DyadicMotionPlannerComponent::getStableHandSupports(int phi) const
{
  return this->explorer.getStableHandSupports(phi);
}

void DyadicMotionPlannerComponent::onSelectObject(std::string objectName)
{
  RLOG(0, "PhysicsComponent::onSelectObject(%s)", objectName.c_str());

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Moving for more %g sec - skipping graph change", motionEndTime);
    return;
  }

  if (objectName == "Box")
  {
    bool success = explorer.setObjectType(BoxStrategy5D::ObjectType::Box);
    if (success)
    {
      tSet->setContactPoints(explorer.getRoboContacts3D(),
                             explorer.getPartnerContacts3D());
    }
  }
  else if (objectName == "Cylinder")
  {
    bool success = explorer.setObjectType(BoxStrategy5D::ObjectType::Cylinder);
    if (success)
    {
      tSet->setContactPoints(explorer.getRoboContacts3D(),
                             explorer.getPartnerContacts3D());
    }
  }
  else if (objectName == "LShape")
  {
    bool success = explorer.setObjectType(BoxStrategy5D::ObjectType::LShape);
    if (success)
    {
      tSet->setContactPoints(explorer.getRoboContacts3D(),
                             explorer.getPartnerContacts3D());
    }
  }
  else
  {
    RLOG(0, "Object type \"%s\" not supported: only Box, Cylinder and LShape",
         objectName.c_str());
    return;
  }

  getEntity()->publish("ObjectModelChanged", explorer.getRoboContacts3D(), explorer.getPartnerContacts3D());

}

void DyadicMotionPlannerComponent::onMoveToState(std::string to, double ttc)
{
  RLOG(0, "WheelPlannerComponent::onMoveToState(%s)", to.c_str());

  if (this->motionEndTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    RLOG(0, "Currently moving (for more %g sec) - skipping new move",
         motionEndTime);
    return;
  }

  double dTo[5];
  bool success = String_toDoubleArray_l(to.c_str(), dTo, 5);
  RCHECK(success);

  std::vector<int> vecTo;
  for (size_t i = 0; i<5; ++i)
  {
    vecTo.push_back((int) lround(dTo[i]));
  }

  auto moveSet = tSet->createMoveToSet(vecTo[0], vecTo[1], vecTo[2], vecTo[3],
                                       vecTo[4], ttc);
  if (moveSet)
  {
    tc->clear();
    tc->add(moveSet);
    moveSet->apply(tc->getTrajectoriesRef());
  }

}

std::shared_ptr<tropic::ConstraintSet> DyadicMotionPlannerComponent::createExampleTrajectory()
{
  std::shared_ptr<tropic::ConstraintSet> moveSet = std::make_shared<tropic::ConstraintSet>();
  RCHECK(moveSet);
  std::vector<int> currentState = this->getCurrentState();
  long int canonPhi = lround(getNextCanonicalAngle(currentState[0]*getDeltaPhi(), 0) / getDeltaPhi());

  // Move to closest initial start pose
  const double ttc0 = 5.0;
  std::vector<int> s0 = getStableHandSupports(canonPhi);
  auto move0 = tSet->createMoveToSet(s0[0], s0[1], s0[2], s0[3], s0[4], ttc0);

  if (!move0)
  {
    RLOG(0, "Failed to create example move: move0");
    return moveSet;
  }

  moveSet->add(move0);

  // Flip to 180 degrees
  const double ttc1 = 6.0;
  const double phi = M_PI;
  std::vector<int> from = s0;
  std::vector<int> to = s0;
  to[0] = (int) lround(phi/explorer.getDeltaPhi());
  explorer.setGoalCondition(BoxStrategy5D::RotationWithMaxStableHold);
  explorer.setGoal(to);
  std::vector<std::vector<int>> solutionPath = Gras::Astar::search(explorer, from);
  bool goalReached = !solutionPath.empty();
  if (!goalReached)
  {
    RLOG(0, "Failed to plan solution for flip to +180 degrees");
    return std::make_shared<tropic::ConstraintSet>();
  }
  Gras::Astar::printSolution(explorer, solutionPath);
  auto move1 = tSet->createSolutionSet(solutionPath, ttc1);
  RCHECK(move1);
  move1->shiftTime(ttc0);
  moveSet->add(move1);


  return moveSet;
}


}   // namespace Rcs
