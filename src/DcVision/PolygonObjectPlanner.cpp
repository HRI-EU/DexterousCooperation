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

#include "PolygonObjectPlanner.h"
#include "DyadicPolygonObjectConstraint.h"
#include "BoxStrategy5D.h"

#include <AStar.h>
#include <Rcs_typedef.h>
#include <Rcs_shape.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>

#include <SphereNode.h>
#include <TextNode3D.h>

#include <thread>
#include <cmath>

#define MIN_HAND_DIST (0.2)
#define MAX_HAND_ANGLE (0.95*M_PI)

namespace Rcs
{

BoxStrategy5D getExplorer(int nStepsAround, const PolygonObjectModel& poly)
{
  BoxStrategy5D explorer = BoxStrategy5D(BoxStrategy5D::None, nStepsAround);
  explorer.setMinimumHandDistance(MIN_HAND_DIST);
  explorer.setGoalCondition(BoxStrategy5D::RotationOnly);
  explorer.setMaxAngleBetweenHands(MAX_HAND_ANGLE);
  explorer.roboContacts.clear();
  explorer.partnerContacts.clear();

  std::vector<PolygonContactPoint2D> c2d = poly.getContacts2d();

  for (size_t i=0; i<c2d.size(); ++i)
  {
    auto cpi = BoxStrategy5D::ContactPoint2D(c2d[i].getX(),
                                             c2d[i].getY(),
                                             c2d[i].getNormalAngle(),
                                             c2d[i].getFrictionAngle(),
                                             c2d[i].getMaxLever());
    explorer.roboContacts.push_back(cpi);
  }

  explorer.partnerContacts = explorer.roboContacts;
  BoxStrategy5D::mirrorContacts(explorer.partnerContacts);

  return explorer;
}

void PolygonObjectPlanner::plannerThread(std::vector<int> from,
                                         std::vector<int> to,
                                         double ttc)
{
  if (from.empty())
  {
    RLOG(1, "\"from\"-state not assigned: No object model assigned"
         " - skipping search");
    return;
  }

  int nStepsAround = (int) lround(2.0*M_PI/this->deltaPhi);
  this->deltaPhi = 2.0*M_PI / (double)nStepsAround;
  auto explorer = getExplorer(nStepsAround, objectModel);

  RLOG(0, "Start planning");
  explorer.setGoal(to);
  std::vector<std::vector<int>> solutionPath = Gras::Astar::search(explorer, from);
  RLOG(0, "%s finishing planning", solutionPath.empty() ? "FAILURE" : "SUCCESS");

  if (solutionPath.empty())
  {
    return;
  }

  if (this->eStop == true)
  {
    RLOG(0, "Skipping to publish trajectory - E-Stop triggered");
    return;
  }

  const double dHeight = 0.1;
  DyadicPolygonObjectConstraint c(this->deltaPhi, dHeight);

  std::vector<HTr> rc3d = objectModel.getContacts3D();
  std::vector<HTr> pc3d = objectModel.getMirroredContacts3D();

  c.setContactPoints(rc3d, pc3d);
  auto tSet = c.createSolutionSet(solutionPath, ttc);

  if (!tSet)
  {
    RLOG(0, "Can't create trajectory - skipping");
    return;
  }

  getEntity()->publish("SetTrajectory", tSet);
}

PolygonObjectPlanner::PolygonObjectPlanner(EntityBase* parent,
                                           const ControllerBase* controller_,
                                           double deltaPhi_) :
  ComponentBase(parent), objectModel(), controller(*controller_),
  deltaPhi(deltaPhi_), eStop(false), isMoving(false)
{
  this->searchState = getCurrentState();
  subscribeAll();
}

void PolygonObjectPlanner::subscribeAll()
{
  subscribe<double, double>("PlanToAngleAndExecute", &PolygonObjectPlanner::onPlanner);
  subscribe("EmergencyStop", &PolygonObjectPlanner::onEmergencyStop);
  subscribe("EmergencyRecover", &PolygonObjectPlanner::onEmergencyRecover);
  subscribe<const RcsGraph*>("InitFromState", &PolygonObjectPlanner::onInitFromState);

  // Initializes the planner with a new polygon geometry
  subscribe<PolygonObjectModel>("SetPolygon", &PolygonObjectPlanner::onSetPolygon);
  subscribe<std::vector<int>, double>("MoveTo", &PolygonObjectPlanner::onMoveToState);
  subscribe("MoveToInit", &PolygonObjectPlanner::onMoveToInitialState);
  subscribe<std::string, double>("MoveToState", &PolygonObjectPlanner::onMoveToStateStr);
  subscribe<int>("TurnObject", &PolygonObjectPlanner::onTurnObject);
  subscribe<double>("DriveHome", &PolygonObjectPlanner::onDriveHome);
  subscribe("SetJointCommand", &PolygonObjectPlanner::onJointCommand);
  subscribe("TrajectoryMoving", &PolygonObjectPlanner::onUpdateMovingState);
}

void PolygonObjectPlanner::startEndlessTest()
{
  bool init = false;

  if (init == false)
  {
    init = true;
    subscribe("TrajectoryMoving", &PolygonObjectPlanner::onEternalTest);
  }

}

bool PolygonObjectPlanner::checkState(std::vector<int> state) const
{
  int nStepsAround = (int) lround(2.0*M_PI/this->deltaPhi);
  auto explorer = getExplorer(nStepsAround, objectModel);
  return explorer.checkState(state);
}

void PolygonObjectPlanner::onJointCommand(const MatNd* q_des)
{
  bool stateChanged = false;
  RcsGraph_setState(controller.getGraph(), q_des, NULL);

  std::vector<int> newState = getCurrentState();

  if (newState != this->searchState)
  {
    stateChanged = true;
  }

  this->searchState = newState;

  std::string hudText;

  hudText = "Time: " + std::to_string(getEntity()->getTime());

  if (this->isMoving)
  {
    hudText += " Robo moves - ";
  }
  else
  {
    hudText += " Robo ready - ";
  }


  // Update text line
  hudText += "State: ";
  for (size_t i=0; i<searchState.size(); ++i)
  {
    hudText += std::to_string(searchState[i]) + " ";
  }

  if (stateChanged)
  {
    hudText += "State is ";
    if (checkState(searchState))
    {
      hudText += "VALID";
    }
    else
    {
      hudText += "INVALID";
    }

  }

  getEntity()->publish("SetTextLine", hudText, 0);
}

void PolygonObjectPlanner::onPlanner(double phi, double ttc)
{
  if (searchState.empty())
  {
    RLOG(1, "No object model assigned - skipping planning");
    return;
  }

  if (isMoving)
  {
    RLOG(1, "Robot is still in motion - skipping planning");
    return;
  }

  RLOG(0, "PolygonObjectPlanner::onPlanner(%.2f deg)",
       RCS_RAD2DEG(phi));

  std::vector<int> from = this->searchState;
  std::vector<int> to = from;
  to[0] = (int) lround(phi/deltaPhi);

  std::thread t1(&PolygonObjectPlanner::plannerThread, this, from, to, ttc);
  t1.detach();
}

void PolygonObjectPlanner::onEmergencyStop()
{
  // The eStop flag is checked before publishing a planned trajectory. In
  // case it is true, the trajectory will be abandonned.
  RLOG(0, "PolygonObjectPlanner::EmergencyStop");
  this->eStop = true;
}

void PolygonObjectPlanner::onEmergencyRecover()
{
  RLOG(0, "PolygonObjectPlanner::EmergencyRecover");
  this->eStop = false;
}

void PolygonObjectPlanner::onInitFromState(const RcsGraph* target)
{
  RLOG(0, "PolygonObjectPlanner::onInitFromState()");
  RcsGraph_setState(controller.getGraph(), target->q, target->q_dot);
  this->searchState = getCurrentState();
}

void PolygonObjectPlanner::onSetPolygon(PolygonObjectModel model)
{
  RLOG(1, "PolygonObjectPlanner::onSetPolygon(%s)", model.getName().c_str());
  this->objectModel = model;
  showContacts(objectModel.getContacts3D(), "POP_c1", "Box_v");
  showContacts(objectModel.getMirroredContacts3D(), "POP_c2", "Partner_Box_v");
}

void PolygonObjectPlanner::showContacts(const std::vector<HTr>& contacts,
                                        const std::string& nodeName,
                                        const std::string& parent)
{
  osg::ref_ptr<osg::Group> grp = new osg::Group();
  grp->setName(nodeName);

  for (size_t i=0; i<contacts.size(); ++i)
  {
    osg::ref_ptr<SphereNode> cn = new SphereNode(contacts[i].org, 0.01);
    cn->setMaterial("GREEN");
    cn->addChild(new TextNode3D(std::to_string(i)));
    grp->addChild(cn.get());
  }

  getEntity()->publish<std::string, std::string>("RemoveNode", "IK", nodeName);
  getEntity()->publish<osg::ref_ptr<osg::Node>, std::string, std::string>("AddChildNode", grp, "IK", parent);
}

std::vector<int> PolygonObjectPlanner::getCurrentState() const
{
  if (!objectModel.valid())
  {
    RLOG_CPP(6, "Invalid model: " << objectModel.getName());
    return searchState;
  }

  const Task* taskPhi = controller.getTask("Phi_Box");
  RCHECK(taskPhi);
  const Task* taskPosR = controller.getTask("XYZ_R");
  RCHECK(taskPosR);
  const Task* taskPosL = controller.getTask("XYZ_L");
  RCHECK(taskPosL);
  const Task* taskPartnerPosR = controller.getTask("Partner_XYZ_R");
  RCHECK(taskPartnerPosR);
  const Task* taskPartnerPosL = controller.getTask("Partner_XYZ_L");
  RCHECK(taskPartnerPosL);

  std::vector<int> state(5);


  double x[3];

  // Box rotation
  taskPhi->computeX(x);
  state[0] = (int) lround(x[2]/this->deltaPhi);



  // Right hand
  state[1] = 0;
  taskPosR->computeX(x);
  std::vector<HTr> contactPointsRobot = objectModel.getContacts3D();
  RCHECK(!contactPointsRobot.empty());
  double minDist = Vec3d_distance(x, contactPointsRobot[0].org);
  for (size_t i=1; i<contactPointsRobot.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPointsRobot[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[1] = (int) i;
    }
  }


  // Left hand
  state[2] = 0;
  taskPosL->computeX(x);
  minDist = Vec3d_distance(x, contactPointsRobot[0].org);
  for (size_t i=1; i<contactPointsRobot.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPointsRobot[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[2] = (int) i;
    }
  }


  // Right partner hand
  state[3] = 0;
  taskPartnerPosR->computeX(x);
  std::vector<HTr> contactPointsPartner = objectModel.getMirroredContacts3D();
  RCHECK(!contactPointsPartner.empty());
  minDist = Vec3d_distance(x, contactPointsPartner[0].org);
  for (size_t i=1; i<contactPointsPartner.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPointsPartner[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[3] = (int) i;
    }
  }

  // Left partner hand
  state[4] = 0;
  taskPartnerPosL->computeX(x);
  minDist = Vec3d_distance(x, contactPointsPartner[0].org);
  for (size_t i=1; i<contactPointsPartner.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPointsPartner[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[4] = (int) i;
    }
  }

  return state;
}

void PolygonObjectPlanner::onTurnObject(int phi)
{
  if (searchState.empty())
  {
    RLOG(1, "No object model assigned - skipping move command");
    return;
  }

  std::vector<int> iState = searchState;
  iState[0] = phi;
  onMoveToState(iState, 5.0);
}

void PolygonObjectPlanner::onMoveToInitialState(double ttc)
{
  if (searchState.empty())
  {
    RLOG(1, "No object model assigned - skipping move command");
    return;
  }

  std::vector<int> iState;
  const double phi = searchState[0]*getDeltaPhi();
  iState.push_back(searchState[0]);
  iState.push_back(objectModel.getRightMostRoboSupportPoint(phi));
  iState.push_back(objectModel.getLeftMostRoboSupportPoint(phi));
  iState.push_back(objectModel.getRightMostMirroredSupportPoint(phi));
  iState.push_back(objectModel.getLeftMostMirroredSupportPoint(phi));

  if (checkState(iState)==false)
  {
    RLOG(0, "Initial state %d %d %d %d %d invalid - skipping",
         iState[0], iState[1], iState[2], iState[3], iState[4]);
    return;
  }

  RLOG(0, "Moving to state %d %d %d %d %d with ttc %f",
       iState[0], iState[1], iState[2], iState[3], iState[4], ttc);

  const double dHeight = 0.1;
  DyadicPolygonObjectConstraint c(this->deltaPhi, dHeight);
  c.setContactPoints(objectModel.getContacts3D(),
                     objectModel.getMirroredContacts3D());

  auto moveSet = c.createMoveToSet(iState[0], iState[1], iState[2], iState[3], iState[4], ttc);

  if (!moveSet)
  {
    RLOG(1, "Couldn't create trajectory to initial state");
    return;
  }

  // Open and close fingers
  const double fngrOpenAngle = RCS_DEG2RAD(-60.0);
  moveSet->add(0.8*ttc, fngrOpenAngle, "Fingers_R 0");
  moveSet->add(0.8*ttc, fngrOpenAngle, "Fingers_R 4");
  moveSet->add(1.6*ttc, 0.0, "Fingers_R 0");
  moveSet->add(1.6*ttc, 0.0, "Fingers_R 4");

  moveSet->add(0.8*ttc, fngrOpenAngle, "Fingers_L 0");
  moveSet->add(0.8*ttc, fngrOpenAngle, "Fingers_L 4");
  moveSet->add(1.6*ttc, 0.0, "Fingers_L 0");
  moveSet->add(1.6*ttc, 0.0, "Fingers_L 4");

  if (moveSet && !this->eStop)
  {
    getEntity()->publish("SetTrajectory", moveSet);
  }

}

void PolygonObjectPlanner::onMoveToState(std::vector<int> to, double ttc)
{
  if (to.size() < 5)
  {
    RLOG(0, "State has too few elements %d, expected 5.", (int) to.size());
    return;
  }

  if (checkState(to)==false)
  {
    RLOG(0, "Goal state %d %d %d %d %d invalid - skipping",
         to[0], to[1], to[2], to[3], to[4]);
    return;
  }

  RLOG(0, "Moving to state %d %d %d %d %d with ttc %f",
       to[0], to[1], to[2], to[3], to[4], ttc);

  const double dHeight = 0.1;
  DyadicPolygonObjectConstraint c(this->deltaPhi, dHeight);
  c.setContactPoints(objectModel.getContacts3D(),
                     objectModel.getMirroredContacts3D());

  auto moveSet = c.createMoveToSet(to[0], to[1], to[2], to[3], to[4], ttc);
  if (moveSet && !this->eStop)
  {
    getEntity()->publish("SetTrajectory", moveSet);
  }

}

void PolygonObjectPlanner::onMoveToStateStr(std::string to, double ttc)
{
  double dTo[5];
  bool success = String_toDoubleArray_l(to.c_str(), dTo, 5);
  RCHECK(success);

  std::vector<int> vecTo;
  for (size_t i = 0; i<5; ++i)
  {
    vecTo.push_back((int) lround(dTo[i]));
  }

  onMoveToState(vecTo, ttc);
}

void PolygonObjectPlanner::onDriveHome(double ttc)
{
  if (this->eStop == true)
  {
    RLOG(0, "Skipping to publish trajectory - E-Stop triggered");
    return;
  }

  if (this->isMoving == true)
  {
    RLOG(0, "Skipping to publish trajectory - robot is still moving");
    return;
  }

  const double dHeight = 0.1;
  DyadicPolygonObjectConstraint c(this->deltaPhi, dHeight);
  auto tSet = c.driveHome(ttc);

  if (!tSet)
  {
    RLOG(0, "Failed to create DriveHome constraints");
    return;
  }

  RLOG(0, "Driving home in %f sec", ttc);
  getEntity()->publish("SetTrajectory", tSet);
}

double PolygonObjectPlanner::getDeltaPhi() const
{
  return this->deltaPhi;
}

void PolygonObjectPlanner::onEternalTest(bool started)
{
  if (started==true)
  {
    return;
  }

  RLOG(0, "eternalTestCallback()");
  static double cbSign = 1.0;
  cbSign *= -1.0;
  getEntity()->publish("PlanToAngleAndExecute", cbSign*M_PI_2, 6.0);
}

void PolygonObjectPlanner::onUpdateMovingState(bool moves)
{
  this->isMoving = moves;
}

}   // namespace Rcs
