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

#include "BoxObjectModel.h"
#include "ActionProgressState.h"
#include "ActionProgressTransition.h"

#include <DyadicTrajectorySet.h>
#include <Rcs_Vec3d.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>


namespace Dc
{



BoxObjectModel::BoxObjectModel(const Rcs::ControllerBase* controller_,
                               BoxStrategy5D::ObjectType object_,
                               int numPhiDiscretizations_,
                               EntityBase* entity_) :
  controller(*controller_),
  numPhiDiscretizations(numPhiDiscretizations_), entity(entity_)
{

  boxExplorer = std::make_shared<BoxStrategy5D>(object_, numPhiDiscretizations_);

  //TODO: remove this
  this->deltaPhi = boxExplorer->getDeltaPhi();

  roboContacts = &(boxExplorer->roboContacts);
  partnerContacts = &(boxExplorer->partnerContacts);

  contactPointsRobot = boxExplorer->getRoboContacts3D();
  contactPointsPartner = boxExplorer->getPartnerContacts3D();

  //this->controller = new ControllerBase(*controller_);

  this->taskPhi = controller.getTask("Phi_Box");
  RCHECK(this->taskPhi);
  this->taskPosR = controller.getTask("XYZ_R");
  RCHECK(this->taskPosR);
  this->taskPosL = controller.getTask("XYZ_L");
  RCHECK(this->taskPosL);
  this->taskPartnerPosR = controller.getTask("Partner_XYZ_R");
  RCHECK(this->taskPartnerPosR);
  this->taskPartnerPosL = controller.getTask("Partner_XYZ_L");
  RCHECK(this->taskPartnerPosL);

}

BoxObjectModel::~BoxObjectModel()
{
  // delete (this->controller);
}

int BoxObjectModel::getStateDimension() const
{
  return 5;
}

bool BoxObjectModel::isStateValid(std::vector<int> state) const
{
  bool valid = boxExplorer->checkState(state);
  RLOG(0, "Checking state: %d %d --> %s",
       state[0], state[1], valid ? "valid" : "invalid");
  return valid;
}

std::vector<int> BoxObjectModel::getEmptyState()
{
  std::vector<int> state(getStateDimension(), 0);
  return state;
}

std::vector<int> BoxObjectModel::getCurrentState(const RcsGraph* graph)
{
  std::vector< std::vector<double> > offsets;
  return getCurrentState(offsets, graph);
}

std::vector<int> BoxObjectModel::getCurrentState(std::vector< std::vector<double> >& offsets, const RcsGraph* graph)
{
  if (graph)
  {
    RcsGraph_setState(controller.getGraph(), graph->q, graph->q_dot);
  }

  std::vector<int> state(5);

  offsets.resize(5);
  offsets[0].resize(1, 0.0);
  offsets[1].resize(3, 0.0);
  offsets[2].resize(3, 0.0);
  offsets[3].resize(3, 0.0);
  offsets[4].resize(3, 0.0);


  double x[3];

  // Box rotation
  if (this->taskPhi)
  {
    this->taskPhi->computeX(x);
    state[0] = (int) lround(x[2]/this->deltaPhi);
  }
  else
  {
    Vec3d_setZero(x);
    state[0] = -1;
  }
  //RLOG(0, "DeltaPhi: %5.3f", this->deltaPhi);
  offsets[0][0] = (state[0] * this->deltaPhi) - x[2];
  //  RLOG(0, " XXAngle: %5.3f, State: %d, State-Angle: %5.3f, delta: %5.3f",
  //x[2], state[0], (state[0] * this->deltaPhi), offsets[0][0] );

  // Right hand
  state[1] = 0;
  RCHECK(!contactPointsRobot.empty());
  RCHECK(!contactPointsPartner.empty());
  taskPosR->computeX(x);
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
  for (size_t i = 0; i < 3; i++)
  {
    offsets[1][i] = contactPointsRobot[state[1]].org[i] - x[i];
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
  for (size_t i = 0; i < 3; i++)
  {
    offsets[2][i] = contactPointsRobot[state[2]].org[i] - x[i];
  }

  // Right partner hand
  state[3] = 0;
  taskPartnerPosR->computeX(x);
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
  for (size_t i = 0; i < 3; i++)
  {
    offsets[3][i] = contactPointsPartner[state[3]].org[i] - x[i];
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
  for (size_t i = 0; i < 3; i++)
  {
    offsets[4][i] = contactPointsPartner[state[4]].org[i] - x[i];
  }

  return state;
}

std::vector<int> BoxObjectModel::getGoalFromIntention(std::vector<int> currentState, int intention_id) const
{
  std::vector<int> nextGoalState(currentState);
  //  std::vector<int> currentClosestCanonicalState = getClosestCanonicalState(currentState);


  //TODO: object specific
  if (intention_id == (int) Intention::ROTATE_RIGHT)
  {
    RLOG(0, "'Rotate Right' command issued!");
    std::vector<int> endState(currentState);
    endState[0] -= 1;



    // if we can't tilt any further and the other hand is supporting the box, plan a re-grasp
    if (!isStateValid(endState))
    {
      RLOG(0, "No valid adjacent states to the right! Executing 90-degree turn.");
      /// TODO: Find next available canonical state instead of assuming 90-degrees
      double angle = getNextCanonicalAngle(currentState[0] * deltaPhi, -1);
      int phi = (int) lround(angle/this->deltaPhi);
      nextGoalState = getStableHandSupports(phi);

      //      getEntity()->publish<double, double>("PlanToAngle",
      //motionPlanner->getNextCanonicalAngle(currentState->getValue(0)*motionPlanner->getDeltaPhi(), +1), ttc);
      if (entity)
      {
        entity->publish<>("Prompt.confirm");
      }
      //      return nextGoalState;
    }
    else
    {
      // search for a maximal tilt angle in this direction, stopping if we hit a horizontal state

      while (true)
      {
        RLOG(0, "Checking next state... %d", endState[0]);

        if (!isStateValid(endState))
        {
          endState[0] += 1;
          nextGoalState = endState;

          break;
        }
        else if (positive_modulo(endState[0], numPhiDiscretizations / 4) == 0)
          //(endState[0] % (numPhiDiscretizations / 4) == 0)
        {
          nextGoalState = endState;
          //          planSequence(currentState, endState, ttc, false);
          //          interactionState = InteractionState::WAITING_FOR_PLANNER;
          break;
        }

        endState[0] -= 1;
      }

      if (positive_modulo(endState[0], numPhiDiscretizations / 4) != 0)
        //(endState[0] % (numPhiDiscretizations / 4) != 0)
      {
        if (entity)
        {
          entity->publish<>("Prompt.lean_right");
        }
      }
      else
      {
        if (entity)
        {
          entity->publish<>("Prompt.clear");
        }
      }
    }
  }
  else if (intention_id == (int) Intention::ROTATE_LEFT)
  {
    RLOG(0, "'Rotate Left' command issued!");
    std::vector<int> endState(currentState);
    endState[0] += 1;

    // if we can't tilt any further and the other hand is supporting the box, plan a re-grasp
    if (!isStateValid(endState))
    {
      RLOG(0, "No valid adjacent states to the right! Executing 90-degree turn.");
      /// TODO: Find next available canonical state instead of assuming 90-degrees

      double angle = getNextCanonicalAngle(currentState[0] * deltaPhi, +1);
      //      RLOG(0, "DeltaPhi: %5.3f, angle - curr: %5.3f, -next: %5.3f ", deltaPhi, currentState[0] * deltaPhi, angle);
      int phi = (int) lround(angle/this->deltaPhi);
      nextGoalState = getStableHandSupports(phi);

      //      getEntity()->publish<double, double>("PlanToAngle",
      //motionPlanner->getNextCanonicalAngle(currentState->getValue(0)*motionPlanner->getDeltaPhi(), +1), ttc);
      if (entity)
      {
        entity->publish<>("Prompt.confirm");
      }


    }
    else
    {
      // search for a maximal tilt angle in this direction, stopping if we hit a horizontal state

      while (true)
      {
        RLOG(0, "Checking next state... %d", endState[0]);

        if (!isStateValid(endState))
        {
          endState[0] -= 1;
          nextGoalState = endState;
          break;
        }
        else if (positive_modulo(endState[0], numPhiDiscretizations / 4) == 0)
          //(endState[0] % (numPhiDiscretizations / 4) == 0)
        {
          nextGoalState = endState;
          break;
        }

        endState[0] += 1;
      }

      if (positive_modulo(endState[0], numPhiDiscretizations / 4) != 0)
        //(endState[0] % (numPhiDiscretizations / 4) != 0)
      {
        if (entity)
        {
          entity->publish<>("Prompt.lean_left");
        }
      }
      else
      {
        if (entity)
        {
          entity->publish<>("Prompt.clear");
        }
      }
    }
  }
  else
  {
    RLOG(0, "Unknown intention: %d. Ignoring.", intention_id);
    return nextGoalState;
  }

  return nextGoalState;
}


std::vector<int> BoxObjectModel::getClosestCanonicalState(std::vector<int> state_in) const
{
  int canonPhi = lround(getNextCanonicalAngle(state_in[0] * deltaPhi, 0) / deltaPhi);
  std::vector<int> stableState = getStableHandSupports(canonPhi);
  return stableState;
}

//std::vector<int> BoxObjectModel::getStableHandSupports(int phi) const
//{
//  //TODO: this needs to be moved out of the exploration strategy into the object model
//  return this->explorer->getStableHandSupports(phi);
//}

double BoxObjectModel::getNextCanonicalAngle(double startAngle, int direction) const
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

double BoxObjectModel::getTtc(std::vector<int> stateFrom, std::vector<int> stateTo, double baseTtc) const
{
  double nextTtc = baseTtc;

  unsigned int type = getTransitionType(stateFrom, stateTo);
  if (type == TransitionType::PartnerRegraspLeft ||
      type == TransitionType::PartnerRegraspRight)
  {
    nextTtc = 1.0;
  }
  else if (type == TransitionType::BothRotateObject)
  {
    //TODO: this might need better method or adjustment
    nextTtc = fabs(stateTo[0] - stateFrom[0]) * deltaPhi *  baseTtc;
    if (nextTtc < 1.3)
    {
      RLOG(0, "Next ttc was too low (%5.3fs), increasing to 1.3s", nextTtc);
      nextTtc = 1.3;
    }
  }
  RLOG(0, "Action: %s TTC: %4.2f", transitionTypeToString(type).c_str(), nextTtc);

  return nextTtc;
}

bool BoxObjectModel::confirmationNeeded(std::vector<int> stateFrom, std::vector<int> stateTo) const
{
  unsigned int type = getTransitionType(stateFrom, stateTo);
  if (type == TransitionType::PartnerRegraspLeft ||
      type == TransitionType::PartnerRegraspRight)
  {
    return false;
  }

  return true;
}

void BoxObjectModel::triggerConfirmationVisualization(std::vector<int> state, int id) const
{
  if (entity)
  {
    entity->publish("DesiredStateChanged", -state[0] * deltaPhi);
  }
}

void BoxObjectModel::triggerExecutionVisualization(std::vector<int> from, std::vector<int> to, RcsGraph* graph) const
{
#if 0
  if (entity)
  {
    auto t = getTransitionType(from, to);
    if (t == TransitionType::PartnerRegraspRight)
    {
      ArrowRaySphereTweener ct("PartnerRegraspRightTargetNotify",
                               "PartnerBoxGrasp(mirror)_" + std::to_string(to[3]),
                               2.5, 0.15,
      {{1, 115, 140, 20}, {1, 115, 140, 255}},
      0.002, "HumanSuggestion");
      Tween<ArrowRaySphereTweener>::StartNew(this->entity, ct, 3);

      Sprite graspSprite("PartnerBoxGrasp(mirror)_" + std::to_string(to[3]) + "_sprite",
                         "PartnerBoxGrasp(mirror)_" + std::to_string(to[3]));
      graspSprite.topic = "HumanSuggestion";
      graspSprite.scale = 0.1;
      graspSprite.spriteName =  "icons/crosshairs/crosshair139";
      graspSprite.setColor({0, 205, 255, 255});
      this->entity->publish("HoloUpdateSprite", graspSprite);

      Sprite prevGraspSprite("PartnerBoxGrasp(mirror)_" + std::to_string(from[3]) + "_sprite",
                             "PartnerBoxGrasp(mirror)_" + std::to_string(from[3]));
      prevGraspSprite.topic = "HumanSuggestion";
      prevGraspSprite.scale = 0.05;
      prevGraspSprite.spriteName =  "icons/crosshairs/crosshair031";
      prevGraspSprite.setColor({128, 128, 128, 128});
      this->entity->publish("HoloUpdateSprite", prevGraspSprite);

      ArrowPlain arrow("PartnerGraspArrowR", "PartnerBoxGrasp(mirror)_" + std::to_string(to[3]), {1, 105, 120, 220});
      arrow.topic = "HumanSuggestion";
      arrow.targetTransform = "PartnerGrasp_R";
      this->entity->publish("HoloUpdateArrowPlain", arrow);
    }
    else if (t == TransitionType::PartnerRegraspLeft)
    {
      ArrowRaySphereTweener ct("PartnerRegraspLeftTargetNotify",
                               "PartnerBoxGrasp(mirror)_" + std::to_string(to[4]),
                               2.5, 0.15,
      {{116, 0, 55, 255}, {116, 0, 55, 0}},
      0.002, "HumanSuggestion");
      Tween<ArrowRaySphereTweener>::StartNew(this->entity, ct, 3);

      Sprite graspSprite("PartnerBoxGrasp(mirror)_" + std::to_string(to[4]) + "_sprite",
                         "PartnerBoxGrasp(mirror)_" + std::to_string(to[4]));
      graspSprite.topic = "HumanSuggestion";
      graspSprite.scale = 0.1;
      graspSprite.spriteName =  "icons/crosshairs/crosshair139";
      graspSprite.setColor({162, 51, 77, 255});
      this->entity->publish("HoloUpdateSprite", graspSprite);

      Sprite prevGraspSprite("PartnerBoxGrasp(mirror)_" + std::to_string(from[4]) + "_sprite",
                             "PartnerBoxGrasp(mirror)_" + std::to_string(from[4]));
      prevGraspSprite.topic = "HumanSuggestion";
      prevGraspSprite.scale = 0.05;
      prevGraspSprite.spriteName =  "icons/crosshairs/crosshair031";
      prevGraspSprite.setColor({128, 128, 128, 128});
      this->entity->publish("HoloUpdateSprite", prevGraspSprite);

      ArrowPlain arrow("PartnerGraspArrowL", "PartnerBoxGrasp(mirror)_" + std::to_string(to[4]), {66, 0, 15, 220});
      arrow.topic = "HumanSuggestion";
      arrow.targetTransform = "PartnerGrasp_L";
      this->entity->publish("HoloUpdateArrowPlain", arrow);
    }
    else if (t == TransitionType::RobotRegraspLeft)
    {
      Sprite graspSprite("RoboBoxGrasp_" + std::to_string(to[2]) + "_sprite", "RoboBoxGrasp_" + std::to_string(to[2]));
      graspSprite.topic = "RobotInfo";
      graspSprite.scale = 0.1;
      graspSprite.spriteName =  "icons/crosshairs/crosshair139";
      graspSprite.setColor({128, 255, 128, 255});
      this->entity->publish("HoloUpdateSprite", graspSprite);

      Sprite prevGraspSprite("RoboBoxGrasp_" + std::to_string(from[2]) + "_sprite",
                             "RoboBoxGrasp_" + std::to_string(from[2]));
      prevGraspSprite.topic = "RobotInfo";
      prevGraspSprite.scale = 0.05;
      prevGraspSprite.spriteName =  "icons/crosshairs/crosshair117";
      prevGraspSprite.setColor({128, 128, 128, 128});
      this->entity->publish("HoloUpdateSprite", prevGraspSprite);

      ArrowPlain arrow("RobotGraspArrowL", "PowerGrasp_L", {120, 120, 80, 180});
      arrow.topic = "RobotInfo";
      arrow.targetTransform = "RoboBoxGrasp_" + std::to_string(to[2]);
      this->entity->publish("HoloUpdateArrowPlain", arrow);

      Box b("BoxCage", "Box_real_center", {9, 232, 170, 120});
      b.topic = "RobotInfo";
      b.setExtents(0.63*0.9, 0.36*0.9, 0.96*0.9);
      ColorTweener<Box> ct(b, {{0, {0, 0, 0, 0}}, {0.5, {9, 232, 120, 255}}, {1.0, {0, 255, 0, 0}}});
      Tween<ColorTweener<Box>>::StartNew(this->entity, ct, 5);

      Box br("BoxGeom", "Vicon box", {9, 169, 232, 120});
      br.topic = "RobotInfo";
      br.setExtents(0.63*0.9, 0.36*0.9, 0.96*0.9);
      ColorTweener<Box> ctr(br, {{0, {0, 0, 0, 0}}, {0.5, {140, 7, 56, 255}}, {1.0, {0, 0, 0, 0}}});
      Tween<ColorTweener<Box>>::StartNew(this->entity, ctr, 5);
    }
    else if (t == TransitionType::RobotRegraspRight)
    {
      Sprite graspSprite("RoboBoxGrasp_" + std::to_string(to[1]) + "_sprite", "RoboBoxGrasp_" + std::to_string(to[1]));
      graspSprite.topic = "RobotInfo";
      graspSprite.scale = 0.1;
      graspSprite.spriteName =  "icons/crosshairs/crosshair139";
      graspSprite.setColor({128, 255, 128, 255});
      this->entity->publish("HoloUpdateSprite", graspSprite);

      Sprite prevGraspSprite("RoboBoxGrasp_" + std::to_string(from[1]) + "_sprite",
                             "RoboBoxGrasp_" + std::to_string(from[1]));
      prevGraspSprite.topic = "RobotInfo";
      prevGraspSprite.scale = 0.05;
      prevGraspSprite.spriteName =  "icons/crosshairs/crosshair117";
      prevGraspSprite.setColor({128, 128, 128, 128});
      this->entity->publish("HoloUpdateSprite", prevGraspSprite);

      ArrowPlain arrow("RobotGraspArrowR", "PowerGrasp_R", {120, 120, 80, 180});
      arrow.topic = "RobotInfo";
      arrow.targetTransform = "RoboBoxGrasp_" + std::to_string(to[1]);
      this->entity->publish("HoloUpdateArrowPlain", arrow);

      Box b("BoxCage", "Box_real_center", {9, 169, 232, 120});
      b.topic = "RobotInfo";
      b.setExtents(0.63*0.9, 0.36*0.9, 0.96*0.9);
      ColorTweener<Box> ct(b, {{0, {0, 0, 0, 0}}, {0.5, {9, 232, 113, 255}}, {1.0, {0, 255, 0, 0}}}, "HoloUpdateBox");
      Tween<ColorTweener<Box>>::StartNew(this->entity, ct, 5);

      Box br("BoxGeom", "Vicon box", {9, 169, 232, 120});
      br.topic = "RobotInfo";
      br.setExtents(0.63*0.9, 0.36*0.9, 0.96*0.9);
      ColorTweener<Box> ctr(br, {{0, {0, 0, 0, 0}}, {0.5, {140, 7, 56, 255}}, {1.0, {0, 0, 0, 0}}}, "HoloUpdateBox");
      Tween<ColorTweener<Box>>::StartNew(this->entity, ctr, 5);
    }

    //TODO: This requires access to a graph, but not here yet
    //    RcsBody* box = RcsGraph_getBodyByName(this->graph, "Box_v");
    //    if (box)
    //    {
    //      const HTr* box_pose = box->A_BI;
    //
    //      int RH1 = to[1];
    //      int LH1 = to[2];
    //
    //      HTr pose_r = this->contactPointsRobot[RH1];
    //      HTr pose_l = this->contactPointsRobot[LH1];
    //
    //      HTr* pose_w_r = HTr_create();
    //      HTr* pose_w_l = HTr_create();
    //
    //      HTr_transform(pose_w_r, box_pose, &pose_r);
    //      HTr_transform(pose_w_l, box_pose, &pose_l);
    //
    //      entity->publish("DesiredHandPoses",
    //                           pose_w_r->org[0],
    //                           pose_w_r->org[1],
    //                           pose_w_r->org[2],
    //                           pose_w_l->org[0],
    //                           pose_w_l->org[1],
    //                           pose_w_l->org[2]);
    //      RFREE(pose_w_r);
    //      RFREE(pose_w_l);
    //    }

    entity->publish("DesiredStateChanged", -to[0] * deltaPhi);
  }
#endif // 0
}

std::map<std::string, double> BoxObjectModel::computeSensorSignals(RcsGraph* desiredGraph, RcsGraph* currentGraph)
{
  //  RcsGraph_setState(this->graph, currentGraph->q, NULL);
  //  std::vector<std::pair<std::string, double> > sensorSignals;

  RcsSensor* sensor = NULL;

  // retrieve joint torques for arms and endeffector forces
  //  double armJointTorques[2][7];
  //  double handNormal[2][3];
  //  double forceVectorHand[2][3];
  //  double forceVector[2][3];
  double forceVectorFTS[2][6];
  double forceVectorHandFTS[2][3];
  double normalForceTwoFinger[2];
  double forceNonGravityFTS[2];


  //////// calculate FT signals

  Vec3d_setZero(forceVectorFTS[0]);
  Vec3d_setZero(forceVectorFTS[1]);
  Vec3d_setZero(forceVectorHandFTS[0]);
  Vec3d_setZero(forceVectorHandFTS[1]);

  HTr A_SB;

  //TODO: this is currently using the passed in graph!
  RcsBody* handBody[2];
  handBody[0] = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_R");
  handBody[1] = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_L");

  if (handBody[0] && handBody[1])
  {


    // right side
    sensor = RcsGraph_getSensorByName(currentGraph, "WristLoadCellLBR_R");
    VecNd_copy(forceVectorFTS[0], sensor->rawData->ele, 6);

    // transform from sensor to world frame
    const RcsBody* mountBdy = RCSBODY_BY_ID(currentGraph, sensor->bodyId);
    const HTr* A_MB = &mountBdy->A_BI;
    const HTr* A_SM = &sensor->A_SB;
    HTr_transform(&A_SB, A_MB, A_SM);

    // care: this is just performing the rotation part to world frame
    Vec3d_transRotateSelf(&forceVectorFTS[0][0], A_SB.rot);
    Vec3d_transRotateSelf(&forceVectorFTS[0][3], A_SB.rot);

    // rotate force part to match powergrasp frame
    Vec3d_rotate(forceVectorHandFTS[0], handBody[0]->A_BI.rot, &forceVectorFTS[0][0]);


    //left side
    sensor = RcsGraph_getSensorByName(currentGraph, "WristLoadCellLBR_L");
    VecNd_copy(forceVectorFTS[1], sensor->rawData->ele, 6);

    mountBdy = RCSBODY_BY_ID(currentGraph, sensor->bodyId);
    const HTr* A_MB2 = &mountBdy->A_BI;
    const HTr* A_SM2 = &sensor->A_SB;
    HTr_transform(&A_SB, A_MB2, A_SM2);

    // this is just performing the rotation part to world frame
    Vec3d_transRotateSelf(&forceVectorFTS[1][0], A_SB.rot);
    Vec3d_transRotateSelf(&forceVectorFTS[1][3], A_SB.rot);

    // rotate force part to match powergrasp frame
    Vec3d_rotate(forceVectorHandFTS[1], handBody[1]->A_BI.rot, &forceVectorFTS[1][0]);

    normalForceTwoFinger[0] = - forceVectorHandFTS[0][1];
    normalForceTwoFinger[1] = - forceVectorHandFTS[1][1];

    forceNonGravityFTS[0] =
      sqrt(forceVectorFTS[0][0] * forceVectorFTS[0][0] + forceVectorFTS[0][1] * forceVectorFTS[0][1]);
    forceNonGravityFTS[1] =
      sqrt(forceVectorFTS[1][0] * forceVectorFTS[1][0] + forceVectorFTS[1][1] * forceVectorFTS[1][1]);

    //  RLOG(0, "[%2d requests remaining] dt: %6.4f, Normal Force: % 5.3f, % 5.3f",
    //(int) requests.size(), dt, normalForceTwoFinger[0], normalForceTwoFinger[1]);

    // store all values in map
    sensorData["normalForceRight"] = normalForceTwoFinger[0];
    sensorData["normalForceLeft"] = normalForceTwoFinger[1];
    sensorData["forceNonGravityRight"] = forceNonGravityFTS[0];
    sensorData["forceNonGravityLeft"] = forceNonGravityFTS[1];
    sensorData["pushForce"] = fmax(- forceVectorFTS[0][0], - forceVectorFTS[1][0]);
    sensorData["normalForceDifference"] = normalForceTwoFinger[0] - normalForceTwoFinger[1];

    sensorData["forceGravityRight"] = - forceVectorFTS[0][2];
    sensorData["forceGravityLeft"] = - forceVectorFTS[1][2];

    sensorData["forceHandZRight"] = forceVectorHandFTS[0][2];
    sensorData["forceHandZLeft"] = forceVectorHandFTS[1][2];

    sensorData["torqueX_right"] = forceVectorFTS[0][3];
    sensorData["torqueY_right"] = forceVectorFTS[0][4];
    sensorData["torqueZ_right"] = forceVectorFTS[0][5];

    sensorData["torqueX_left"] = forceVectorFTS[1][3];
    sensorData["torqueY_left"] = forceVectorFTS[1][4];
    sensorData["torqueZ_left"] = forceVectorFTS[1][5];

  }

  //=======================
  // use vicon data to determine object (mis-)alignment and human hand poses
  const RcsBody* objBdy_cur = RcsGraph_getBodyByName(currentGraph, "Box_real");
  const RcsBody* objBdy_des = RcsGraph_getBodyByName(desiredGraph, "Box_v");

  const RcsBody* handBdy_des_R = RcsGraph_getBodyByName(desiredGraph, "PowerGrasp_R");
  const RcsBody* handBdy_des_L = RcsGraph_getBodyByName(desiredGraph, "PowerGrasp_L");

  const RcsBody* handBdy_cur_R = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_R");
  const RcsBody* handBdy_cur_L = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_L");

  if (objBdy_cur && objBdy_des && handBdy_des_R && handBdy_des_L && handBdy_cur_R && handBdy_cur_L)
  {

    const HTr* des_A_RH_I = &handBdy_des_R->A_BI;
    const HTr* des_A_LH_I = &handBdy_des_L->A_BI;

    const HTr* cur_A_RH_I = &handBdy_cur_R->A_BI;
    const HTr* cur_A_LH_I = &handBdy_cur_L->A_BI;

    const HTr* cur_A_OBJ_I = &objBdy_cur->A_BI;
    const HTr* des_A_OBJ_I = &objBdy_des->A_BI;


    // determine desired position of hands in object frame

    //  HTr_transform(rh_cur, cur_T_RH_I, )

    HTr* des_A_RH_OBJ = HTr_create();
    HTr_invTransform(des_A_RH_OBJ, des_A_OBJ_I, des_A_RH_I);

    HTr* des_A_LH_OBJ = HTr_create();
    HTr_invTransform(des_A_LH_OBJ, des_A_OBJ_I, des_A_LH_I);

    HTr* cur_A_RH_OBJ = HTr_create();
    HTr_invTransform(cur_A_RH_OBJ, cur_A_OBJ_I, des_A_RH_I);

    HTr* cur_A_LH_OBJ = HTr_create();
    HTr_invTransform(cur_A_LH_OBJ, cur_A_OBJ_I, des_A_LH_I);

    //  double pos_rh[3], pos_rh1[3], pos_rh2[3];
    //  Vec3d_setZero(pos_rh);
    //  Vec3d_copy(pos_rh, des_A_RH_I->org);
    //
    //  Vec3d_transform(pos_rh1, des_A_OBJ_I, pos_rh);
    //  Vec3d_invTransform(pos_rh2, des_A_OBJ_I, pos_rh);

    double pos_error_rh[3];
    Vec3d_sub(pos_error_rh, des_A_RH_OBJ->org, cur_A_RH_OBJ->org);

    double pos_error_lh[3];
    Vec3d_sub(pos_error_lh, des_A_LH_OBJ->org, cur_A_LH_OBJ->org);

    //  char text[256];
    ////  sprintf(text, " RHand: %6.3f, %6.3f, %6.3f", pos_rh1[0], pos_rh1[1], pos_rh1[2]);
    //  sprintf(text, " RHand: %6.3f, %6.3f, %6.3f", des_A_RH_OBJ->org[0], des_A_RH_OBJ->org[1], des_A_RH_OBJ->org[2]);
    ////  sprintf(text, " Box: %6.3f, %6.3f, %6.3f",
    ////objBdy_des->A_BI->org[0], objBdy_des->A_BI->org[1], objBdy_des->A_BI->org[2]);
    //  getEntity()->publish<std::string>("SetTextLine", std::string(text), 5);
    ////  sprintf(text, " RHand: %6.3f, %6.3f, %6.3f", pos_rh2[0], pos_rh2[1], pos_rh2[2]);
    ////  getEntity()->publish<std::string>("SetTextLine", std::string(text), 7);

    //TODO: read these values from the graph bodies
    sensorData["boxDistance"] = Vec3d_distance(des_A_OBJ_I->org, cur_A_OBJ_I->org);
    double z_I[3] = {0.0, 0.0, 1.0};
    double cur_z_OBJ[3];
    double des_z_OBJ[3];

    //we use the direction of the unit-z vector in box frame to determine the rotation angle
    Vec3d_rotate(cur_z_OBJ, (double (*)[3])cur_A_OBJ_I->rot, z_I);
    Vec3d_rotate(des_z_OBJ, (double (*)[3])des_A_OBJ_I->rot, z_I);

    double cur_box_angle = atan2(cur_z_OBJ[0], cur_z_OBJ[1]);
    double des_box_angle = atan2(des_z_OBJ[0], des_z_OBJ[1]);

    double box_angle_error = des_box_angle - cur_box_angle;

    //  RLOG(0, "Angle error: %5.2f", box_angle_error);
    while (box_angle_error > M_PI)
    {
      box_angle_error -= 2.0 * M_PI;
    }
    while (box_angle_error < - M_PI)
    {
      box_angle_error += 2.0 * M_PI;
    }
    sensorData["boxRotationError"] = fabs(box_angle_error);

    //  RLOG(0, "Angle error: %5.2f (des: %5.2f, cur: %5.2f",
    //RCS_RAD2DEG(box_angle_error), RCS_RAD2DEG(des_box_angle), RCS_RAD2DEG(cur_box_angle));
    //  RLOG(0, "Angle: %5.2f, z_obj: %5.2f, %5.2f, %5.2f", RCS_RAD2DEG(angle), z_OBJ[0], z_OBJ[1], z_OBJ[2]);
    //  RLOG(0, "Angle: %5.2f, z_I:   %5.2f, %5.2f, %5.2f", RCS_RAD2DEG(angle), z_I[0], z_I[1], z_I[2]);


    //  double y_OBJ[3] = {0.0, 1.0, 0.0};
    //  double y_I[3];
    //  Vec3d_rotate(z_I, cur_A_OBJ_I->rot, z_OBJ);

    //  printf("Desired: \n");
    //  HTr_print(des_A_OBJ_I);
    //  printf("Current: \n");
    //  HTr_print(cur_A_OBJ_I);

    //  VecNd_printComment("Pos:", cur_A_OBJ_I->org, 3);
    //
    //  for (int i = 0; i < 3; i ++)
    //  {
    //    for (int j = 0; j < 3; j++)
    //    {
    //      printf("%5.2f, ", cur_A_OBJ_I->rot[i][j]);
    //    }
    //    printf("\n");
    //  }
    //
    //
    //
    //
    //
    //
    //  RLOG(0, "Angle: %5.2f, y_obj: %5.2f, %5.2f, %5.2f", RCS_RAD2DEG(angle), y_OBJ[0], y_OBJ[1], y_OBJ[2]);
    //  RLOG(0, "Angle: %5.2f, y_I:   %5.2f, %5.2f, %5.2f", RCS_RAD2DEG(angle), y_I[0], y_I[1], y_I[2]);
    //
    //  HTr* A_curOBJ_desOBJ = HTr_create();
    //
    //  HTr_invTransform(A_curOBJ_desOBJ, des_A_OBJ_I, cur_A_OBJ_I );
    //  double trans[6];
    //  HTr_to6DVector(trans, A_curOBJ_desOBJ);
    ////  VecNd_printComment("des to cur:", trans, 6);
    //
    //  sensorData["boxRotationError"] = fabs(trans[5]);

    sensorData["handDistanceRight"] = Vec3d_distance(des_A_RH_OBJ->org, cur_A_RH_OBJ->org);
    sensorData["handDistanceLeft"] = Vec3d_distance(des_A_LH_OBJ->org, cur_A_LH_OBJ->org);

    delete (des_A_RH_OBJ);
    delete (cur_A_RH_OBJ);
    delete (des_A_LH_OBJ);
    delete (cur_A_LH_OBJ);

    double hand_pos_R[3];
    Vec3d_sub(hand_pos_R, des_A_RH_I->org, cur_A_RH_I->org);
    double hand_pos_L[3];
    Vec3d_sub(hand_pos_L, des_A_LH_I->org, cur_A_LH_I->org);

    sensorData["hand_pos_R_x"] = hand_pos_R[0];
    sensorData["hand_pos_R_y"] = hand_pos_R[1];
    sensorData["hand_pos_R_z"] = hand_pos_R[2];
    sensorData["hand_pos_L_x"] = hand_pos_L[0];
    sensorData["hand_pos_L_y"] = hand_pos_L[1];
    sensorData["hand_pos_L_z"] = hand_pos_L[2];
  }

  //  const RcsBody* headBdy_cur = RcsGraph_getBodyByName(currentGraph, "human_head");
  const std::string headBody = "Vicon head";
  const RcsBody* headBdy_cur = RcsGraph_getBodyByName(currentGraph, headBody.c_str());
  if (headBdy_cur)
  {
    const HTr* cur_A_head_I = &headBdy_cur->A_BI;

    const RcsBody* shoulderBdy_cur = RcsGraph_getBodyByName(currentGraph, "RailBot");
    const HTr* cur_A_bot_I = &shoulderBdy_cur->A_BI;

    double vec_head_bot[3];
    Vec3d_invTransform(vec_head_bot, cur_A_head_I, cur_A_bot_I->org);
    //  Vec3d_sub(vec_head_bot, cur_A_bot_I->org, cur_A_head_I->org);
    Vec3d_normalizeSelf(vec_head_bot);
    double vec_unit_x[3];
    Vec3d_set(vec_unit_x, 1.0, 0.0, 0.0);
    double gazeAngle = acos(Vec3d_innerProduct(vec_unit_x, vec_head_bot));

    //    char text[256];
    //    sprintf(text,
    //            " Head to robot: %6.3f, %6.3f, %6.3f  angle: %6.3f",
    //            vec_head_bot[0],
    //            vec_head_bot[1],
    //            vec_head_bot[2],
    //            RCS_RAD2DEG(gazeAngle));
    //    getEntity()->publish<std::string>("SetTextLine", std::string(text), 5);

    sensorData["gazeError"] = gazeAngle;
  }
  else
  {
    RLOG(5, "Body '%s' not found in graph. 'gazeError' fixed to 0.0!", headBody.c_str());
    sensorData["gazeError"] = 0.0;
  }
  //=======================

  return sensorData;
}


std::vector<int> BoxObjectModel::getStableHandSupports(int phi) const
{
  std::vector<int> result;
  result.push_back(phi);
  result.push_back(getSupportPoint(phi, false, true));
  result.push_back(getSupportPoint(phi, true, true));
  result.push_back(getSupportPoint(phi, false, false));
  result.push_back(getSupportPoint(phi, true, false));

  return result;
}

int BoxObjectModel::getSupportPoint(int phi_, bool left, bool robot, bool print) const
{
  int support = -1;
  double x = 0.0, y = 0.0, x_comp = 0.0, y_comp = 0.0;
  int phi;

  const std::vector<BoxStrategy5D::ContactPoint2D>* contacts;
  if (robot)
  {
    contacts = roboContacts;
    phi = phi_;
  }
  else
  {
    contacts = partnerContacts;
    phi = -phi_;
  }

  bool findMin = left;
  //  if (!robot)
  //  {
  //    findMin = !findMin;
  //  }

  for (size_t i = 0; i < contacts->size(); i++)
  {

    if (contacts->at(i).isSupportPoint(phi*deltaPhi))
    {
      if (print)
      {
        printf("\t %2d: support\n", (int) i);
      }
      contacts->at(i).getContactInWorld(x, y, phi*deltaPhi);

      if (support == -1)
      {
        support = i;
        x_comp = x;
        y_comp = y;
      }
      else
      {
        if ((findMin  && ((x < x_comp) || ((x == x_comp) && (y < y_comp))))
            || (!findMin && ((x > x_comp) || ((x == x_comp) && (y < y_comp)))))
        {
          support = i;
          x_comp = x;
          y_comp = y;
        }
      }
    }
    else
    {
      if (print)
      {
        printf("\t %2d: no support\n", (int) i);
      }
    }
  }

  if (print)
  {
    printf("-------\n  Best support: %d\n\n", support);
  }
  return support;
}

/************************************************************************
 * Monitors
 ************************************************************************/

std::vector<MonitorRequest> BoxObjectModel::configureMonitorsForState(std::vector<int> state) const
{
  RLOG(0, "Requesting monitors for state (%d, %d, %d, %d, %d)", state[0], state[1], state[2], state[3], state[4]);

  double gazeAttentionThreshold = RCS_DEG2RAD(60.0);



  std::vector<MonitorRequest> monitors;

  double rotationForceDifference = 16.0;
  double activationTime = 0.2;

  //TODO: this check needs to be fixed -> use canonical states or similar
  if (positive_modulo(state[0], this->numPhiDiscretizations / 4) == 0)
    //(state[0] % (this->numPhiDiscretizations / 4) == 0)
  {
    activationTime = 0.4;
    rotationForceDifference = 30.0;
  }

  MonitorRequest mon("Intention", (int) Intention::ROTATE_RIGHT, 0, "Rotate right", activationTime);
  mon.addSignal("normalForceDifference", rotationForceDifference, false, rotationForceDifference*0.85);
  mon.addSignal("gazeError", gazeAttentionThreshold, true);
  monitors.push_back(mon);

  MonitorRequest mon2("Intention", (int) Intention::ROTATE_LEFT, 0, "Rotate left", activationTime);
  mon2.addSignal("normalForceDifference", -rotationForceDifference, true, rotationForceDifference*0.85);
  mon2.addSignal("gazeError", gazeAttentionThreshold, true);
  monitors.push_back(mon2);

  return monitors;
}


std::vector<MonitorRequest> BoxObjectModel::configureConfirmationMonitorsForTransition(std::vector<int> startState,
    std::vector<int> endState) const
{
  std::vector<MonitorRequest> monitors;

  unsigned int type = getTransitionType(startState, endState);

  //      RLOG(0, "Transition type: %d", (int) type);

  // double normalForceUnloadedThreshold = 2.0;
  double normalForceLoadedThreshold   = 8.0;
  double gazeAttentionThreshold =  RCS_DEG2RAD(60.0);
  double regraspLoadDifference = 0.0;

  if (type == TransitionType::BothRotateObject)
  {
    if (startState[0] > endState[0])
    {
      MonitorRequest mon("Confirmation", 0, 0, "rotation right");
      mon.addSignal("normalForceRight", normalForceLoadedThreshold, false);
      mon.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon);
    }
    else
    {
      MonitorRequest mon("Confirmation", 0, 0, "rotation left");
      mon.addSignal("normalForceLeft", normalForceLoadedThreshold, false);
      mon.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon);
    }
  }
  else if (type == TransitionType::RobotRegraspRight)
  {
    // regrasping hand needs to be unloaded
    //        MonitorRequest mon("Confirmation", 0, 0, "regrasp right, hand free");
    //        mon.addSignal("normalForceRight", normalForceUnloadedThreshold, true);
    //        mon.addSignal("gazeError", gazeAttentionThreshold, true);
    //        monitors.push_back(mon);

    // Check discrete distance between hands, if below a threshold the hands must be on the same side of the box
    // Only add confirmation monitors if the hands are on DIFFERENT sides.
    int state_diff = startState[1] - startState[2];
    while (state_diff < 0)
    {
      state_diff += roboContacts->size(); // account for wraparound in state space
    }
    if (state_diff > 4)  /// TODO: Cast this code into the pit
    {
      // regrasping hand can be load, but support hand also needs minimum load
      MonitorRequest mon2("Confirmation", 0, 0, "regrasp right, light load, left loaded");
      mon2.addSignal("normalForceLeft", normalForceLoadedThreshold, false);
      mon2.addSignal("normalForceRight", normalForceLoadedThreshold, true);
      mon2.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon2);

      MonitorRequest mon3("Confirmation", 0, 0, "regrasp right, hand free");
      //        mon3.addSignal("normalForceRight", normalForceUnloadedThreshold, true);
      mon3.addSignal("normalForceLeft", normalForceLoadedThreshold, false);
      mon3.addSignal("normalForceDifference", -regraspLoadDifference, true);
      mon3.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon3);
    }
    else
    {
      MonitorRequest mon("Confirmation", 0, 0, "none - just continue");
      monitors.push_back(mon);
    }
  }
  else if (type == TransitionType::RobotRegraspLeft)
  {
    // regrasping hand needs to be unloaded
    //        MonitorRequest mon("Confirmation", 0, 0, "regrasp left, hand free");
    //        mon.addSignal("normalForceLeft", normalForceUnloadedThreshold, true);
    //        mon.addSignal("gazeError", gazeAttentionThreshold, true);
    //        monitors.push_back(mon);


    // Check discrete distance between hands, if below a threshold the hands must be on the same side of the box
    // Only add confirmation monitors if the hands are on DIFFERENT sides.
    int state_diff = startState[1] - startState[2];
    while (state_diff < 0)
    {
      state_diff += roboContacts->size(); // account for wraparound in state space
    }
    if (state_diff > 4) /// TODO: Cast this code into the pit
    {
      // regrasping hand can be load, but support hand also needs minimum load
      MonitorRequest mon2("Confirmation", 0, 0, "regrasp left, light load, right loaded");
      mon2.addSignal("normalForceRight", normalForceLoadedThreshold, false);
      mon2.addSignal("normalForceLeft", normalForceLoadedThreshold, true);
      mon2.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon2);

      MonitorRequest mon3("Confirmation", 0, 0, "regrasp right, hand free");
      //        mon3.addSignal("normalForceRight", normalForceUnloadedThreshold, true);
      mon3.addSignal("normalForceRight", normalForceLoadedThreshold, false);
      mon3.addSignal("normalForceDifference", regraspLoadDifference, false);
      mon3.addSignal("gazeError", gazeAttentionThreshold, true);
      monitors.push_back(mon3);
    }
    else
    {
      MonitorRequest mon("Confirmation", 0, 0, "none - just continue");
      monitors.push_back(mon);
    }
  }
  else if (type == TransitionType::None)
  {
    //        RLOG(0, "!!!! Confirmation for none-transition requested: passing blank");
    MonitorRequest mon("Confirmation", 0, 0, "none - just continue");
    monitors.push_back(mon);
  }
  else
  {
    RLOG(0, "!!! Confirmation requests can not be created for undefined transitions (%d - %s)",
         type, transitionTypeToString(type).c_str());
  }

  //TODO: Intentions need to be put here:
  //      MonitorRequest mon("Intention", 0, 500);
  //      mon.addSignal("normalForceDifference", 30, false);
  //      monitors.push_back(mon);



  return monitors;
}


std::vector<MonitorRequest> BoxObjectModel::configureMonitorsForTransition(std::vector<int> startState,
    std::vector<int> endState) const
{
  //TODO: this should handle
  // intentions to monitor for changed intentions
  // progress to monitor proper execution


  std::vector<MonitorRequest> monitors;

  unsigned int type = getTransitionType(startState, endState);

  RLOG(0, "Transition type: %d", (int) type);

  //      double normalForceUnloadedThreshold = 2.0;
  //      double normalForceLoadedThreshold   = 8.0;
  double rotationForceDifference = 16.0;

  if (type == TransitionType::BothRotateObject)
  {
    if (startState[0] > endState[0])
    {
      MonitorRequest mon("Intention", (int) BoxObjectModel::Intention::ROTATE_LEFT, 0, "Rotate left");
      mon.addSignal("normalForceDifference", -rotationForceDifference, true);
      monitors.push_back(mon);
    }
    else
    {
      MonitorRequest mon("Intention", (int) BoxObjectModel::Intention::ROTATE_RIGHT, 0, "Rotate right");
      mon.addSignal("normalForceDifference", rotationForceDifference, false);
      monitors.push_back(mon);
    }
  }
  else if (type == TransitionType::RobotRegraspRight)
  {
    //        // regrasping hand needs to be unloaded
    //        MonitorRequest mon("Confirmation", 0, 0, "regrasp right, hand free");
    //        mon.addSignal("normalForceRight", normalForceUnloadedThreshold, true);
    //        monitors.push_back(mon);
    //
    //        // regrasping hand can be load, but support hand also needs minimum load
    //        MonitorRequest mon2("Confirmation", 0, 0, "regrasp right, light load, other hand loaded");
    //        mon2.addSignal("normalForceLeft", normalForceLoadedThreshold, false);
    //        mon2.addSignal("normalForceRight", normalForceLoadedThreshold, true);
    //        monitors.push_back(mon2);
  }
  else if (type == TransitionType::RobotRegraspLeft)
  {
    //        // regrasping hand needs to be unloaded
    //        MonitorRequest mon("Confirmation", 0, 0, "regrasp left, hand free");
    //        mon.addSignal("normalForceLeft", normalForceUnloadedThreshold, true);
    //        monitors.push_back(mon);
    //
    //        // regrasping hand can be load, but support hand also needs minimum load
    //        MonitorRequest mon2("Confirmation", 0, 0, "regrasp left, light load, other hand loaded");
    //        mon2.addSignal("normalForceRight", normalForceLoadedThreshold, false);
    //        mon2.addSignal("normalForceLeft", normalForceLoadedThreshold, true);
    //        monitors.push_back(mon2);
  }

  //TODO: Intentions need to be put here:
  //      MonitorRequest mon("Intention", 0, 500);
  //      mon.addSignal("normalForceDifference", 30, false);
  //      monitors.push_back(mon);


  return monitors;
}




/************************************************************************
 * Action Progress Graphs
 ************************************************************************/

std::vector<Apg_ptr> BoxObjectModel::getActionProgressGraphs() const
{
  std::vector<Apg_ptr> actionProgressGraphs;

  actionProgressGraphs.push_back(createRegraspAPG(true));
  actionProgressGraphs.push_back(createRegraspAPG(false));
  actionProgressGraphs.push_back(createRotateObjectAPG());

  return actionProgressGraphs;
}




Apg_ptr BoxObjectModel::createRegraspAPG(bool rightHandRegrasp) const
{
  //TODO: this is shared between functions
  double gazeAttentionThreshold =  RCS_DEG2RAD(60.0);
  double gazeAttentionDuration = 0.5;
  double duration = 6.0;

  //  double externalCollisionForce = 16.0;
  double handForceLoaded = 8.0;
  double handForceUnloaded = 2.0;
  //  double positionError = 0.08;
  double maxRotationError = 0.26; // ~15 deg

  std::string side = rightHandRegrasp ? "right" : "left";

  Apg_ptr tmp(new ActionProgressGraph(std::to_string(1), ("regrasp " + side)));
  if (rightHandRegrasp)
  {
    tmp->id_ = "1";
  }
  else
  {
    tmp->id_ = "2";
  }

  tmp->duration_ = duration;

  ActionProgressState_ptr s0(new ActionProgressState());
  s0->id_ = "0";
  s0->description_ = "start, both hands in contact";
  s0->action_ = tmp;

  //    s0->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "HumanGraspError"));
  s0->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "ObjectAlignment"));
  s0->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));

  ActionProgressState_ptr s1(new ActionProgressState());
  s1->id_ = "1";
  s1->description_ = "one hand in transit";
  s1->action_ = tmp;

  //  s1->enterEvents.push_back(std::make_pair<std::string, std::string>("You are ", "on your way"));
  s1->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));

  ActionProgressState_ptr s2(new ActionProgressState());
  s2->id_ = "2";
  s2->description_ = "goal reached, both hands in contact again";
  s2->action_ = tmp;
  s2->goal_ = true;
  s2->success_ = true;

  ActionProgressState_ptr s3(new ActionProgressState());
  s3->id_ = "3";
  s3->description_ = "fault, movement finished, but no contact re-established";
  s3->action_ = tmp;
  s3->goal_ = true;
  s3->success_ = false;

  ActionProgressState_ptr s4(new ActionProgressState());
  s4->id_ = "4";
  s4->description_ = "fault, external contact while moving hand";
  s4->action_ = tmp;
  s4->goal_ = true;
  s4->success_ = false;

  ActionProgressState_ptr s5(new ActionProgressState());
  s5->id_ = "5";
  s5->description_ = "box pose is shifting too far. Pausing until correction.";
  s5->action_ = tmp;
  s5->goal_ = false;
  s5->success_ = false;

  s5->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));

  ActionProgressState_ptr s6(new ActionProgressState());
  s6->id_ = "6";
  s6->description_ = "fault, box following on withdrawl";
  s6->action_ = tmp;
  s6->goal_ = true;
  s6->success_ = false;

  s6->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "ObjectAlignment"));

  ActionProgressState_ptr s7(new ActionProgressState());
  s7->id_ = "7";
  s7->description_ = "fault, partner not paying attention";
  s7->action_ = tmp;
  s7->goal_ = false;
  s7->success_ = false;

  //    s7->enterEvents.push_back(std::make_pair<std::string, std::string>("StopActions", ""));
  s7->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));
  //  ManipulationState_ptr s7(new ActionProgressState());
  //  s7->id_ = "7";
  //  s7->description_ = "box pose is shifting too far. Adapting trajectory.";
  //  s7->action_ = tmp;
  //  s7->goal_ = false;
  //  s7->success_ = false;

  //  sensorData["normalForceRight"] = 0.0;
  //  sensorData["normalForceLeft"] = 0.0;
  //  sensorData["forceNonGravityRight"] = 0.0;
  //  sensorData["forceNonGravityLeft"] = 0.0;
  //  sensorData["pushForce"] = 0.0;
  //  sensorData["normalForceDifference"] = 0.0;
  //  "boxDistance"
  //  "handDistanceRight"
  //  "handDistanceLeft"



  //---------------------------------------------------------------------

  Transition_ptr t_hand_release(new ActionProgressTransition(0, s0, s1, side + " hand released"));
  t_hand_release->deltaTmin_ = 0.0;
  t_hand_release->deltaTmax_ = 1.0;
  s0->transitions_out_.push_back(t_hand_release);
  s1->transitions_in_.push_back(t_hand_release);

  MonitorRequest_ptr m_hand_release(new MonitorRequest("Progress", 0, 0, side + " hand released", 0.0));
  if (rightHandRegrasp)
  {
    m_hand_release->addSignal("normalForceRight", handForceUnloaded, true);
    m_hand_release->addSignal("forceNonGravityRight", handForceUnloaded, true);
    m_hand_release->addSignal("normalForceLeft", handForceLoaded, false);
    m_hand_release->addSignal("forceGravityLeft", handForceLoaded, false);
  }
  else
  {
    m_hand_release->addSignal("normalForceLeft", handForceUnloaded, true);
    m_hand_release->addSignal("forceNonGravityLeft", handForceUnloaded, true);
    m_hand_release->addSignal("normalForceRight", handForceLoaded, false);
    m_hand_release->addSignal("forceGravityRight", handForceLoaded, false);
  }
  //  m_hand_release_r->addSignal("forceNonGravityRight", handForceUnloaded, true);
  //  m_hand_release_r->addSignal("forceNonGravityLeft", handForceUnloaded, true);
  t_hand_release->monitors_.push_back(m_hand_release);

  s0->monitors_.push_back(m_hand_release);

  //---------------------------------------------------------------------

  Transition_ptr t_hand_recontact(new ActionProgressTransition(1, s1, s2, "reach goal"));
  t_hand_recontact->deltaTmin_ = 0.8;
  t_hand_recontact->deltaTmax_ = 1.0;
  s1->transitions_out_.push_back(t_hand_recontact);
  s2->transitions_in_.push_back(t_hand_recontact);

  //TODO: these conditions need fixing
  MonitorRequest_ptr m_hand_recontact(new MonitorRequest("Progress", 1, 0, side + " hand re-establish contact", 0.0));

  if (rightHandRegrasp)
  {
    m_hand_recontact->addSignal("normalForceRight", handForceLoaded, false);
    m_hand_recontact->addSignal("normalForceLeft", handForceLoaded, false);
    m_hand_recontact->addSignal("forceGravityLeft", handForceLoaded, false);
  }
  else
  {
    m_hand_recontact->addSignal("normalForceLeft", handForceLoaded, false);
    m_hand_recontact->addSignal("normalForceRight", handForceLoaded, false);
    m_hand_recontact->addSignal("forceGravityRight", handForceLoaded, false);
  }

  t_hand_recontact->monitors_.push_back(m_hand_recontact);

  s1->monitors_.push_back(m_hand_recontact);
  //---------------------------------------------------------------------

  //  Transition_ptr t_external_contact(new ActionProgressTransition(2, s0, s4, "collision with obstacle"));
  //  t_external_contact->deltaTmin_ = 0.2;
  //  t_external_contact->deltaTmax_ = 1.1;
  //  s0->transitions_out_.push_back(t_external_contact);
  //  s4->transitions_in_.push_back(t_external_contact);
  //
  //  MonitorRequest_ptr m_ext_collision(new MonitorRequest("Progress", 2, 0, "collision with obstacle", 0.0));
  //  if (rightHandRegrasp)
  //  {
  //    m_ext_collision->addSignal("forceNonGravityRight", externalCollisionForce, false);
  //    m_ext_collision->addSignal("normalForceLeft", handForceLoaded, false);
  //    m_ext_collision->addSignal("forceGravityLeft", handForceLoaded, false);
  //  }
  //  else
  //  {
  //    m_ext_collision->addSignal("forceNonGravityLeft", externalCollisionForce, false);
  //    m_ext_collision->addSignal("normalForceRight", handForceLoaded, false);
  //    m_ext_collision->addSignal("forceGravityRight", handForceLoaded, false);
  //  }
  //  t_external_contact->monitors_.push_back(m_ext_collision);
  //
  //  s0->monitors_.push_back(m_ext_collision);

  //---------------------------------------------------------------------

  //  Transition_ptr t_collision(new ActionProgressTransition(3, s1, s4, "collision with obstacle"));
  //  t_collision->deltaTmin_ = 0.0;
  //  t_collision->deltaTmax_ = 1.0;
  //  s1->transitions_out_.push_back(t_collision);
  //  s4->transitions_in_.push_back(t_collision);
  //
  //  MonitorRequest_ptr m_collision(new MonitorRequest(m_ext_collision));
  //  m_collision->description = "collision with obstacle";
  //  m_collision->id = 3;
  //
  //  t_collision->monitors_.push_back(m_collision); // use same monitor as above
  //
  //  s1->monitors_.push_back(m_collision);

  //---------------------------------------------------------------------

  //  Transition_ptr t_finshed_no_contact(new ActionProgressTransition(4, s1, s3, "finished regrasp, but no contact"));
  //  t_finshed_no_contact->deltaTmin_ = 1.0;
  //  t_finshed_no_contact->deltaTmax_ = 1.1;
  //  s1->transitions_out_.push_back(t_finshed_no_contact);
  //  s3->transitions_in_.push_back(t_finshed_no_contact);
  //
  //  //  MonitorRequest_ptr m_no_contact_r(new MonitorRequest("Progress", 4, 0, "collision with obstacle", 0.0));
  //  //  m_ext_collision_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
  //  //  m_ext_collision_r->addSignal("normalForceLeft", handForceLoaded, false);
  //  //  m_ext_collision_r->addSignal("forceGravityLeft", handForceLoaded, false);
  //  MonitorRequest_ptr m_regrasp_without_contact(new MonitorRequest(m_hand_release));
  //  m_regrasp_without_contact->description = "finished regrasp, but no contact";
  //
  //  t_finshed_no_contact->monitors_.push_back(m_regrasp_without_contact); // use same monitor as above
  //
  //  s1->monitors_.push_back(m_regrasp_without_contact);

  //---------------------------------------------------------------------

  Transition_ptr t_box_shifted_too_far(new ActionProgressTransition(5, s1, s5, "box shifted too far"));
  t_box_shifted_too_far->deltaTmin_ = 0.0;
  t_box_shifted_too_far->deltaTmax_ = 1.0;
  s1->transitions_out_.push_back(t_box_shifted_too_far);
  s5->transitions_in_.push_back(t_box_shifted_too_far);

  MonitorRequest_ptr m_box_shifted(new MonitorRequest("Progress", 5, 0, "box shifted too far", 0.3));

  if (rightHandRegrasp)
  {
    //  m_box_shifted->addSignal("forceNonGravityRight", externalCollisionForce, false);
    //    m_box_shifted->addSignal("normalForceLeft", handForceLoaded, false);
    //    m_box_shifted->addSignal("forceGravityLeft", handForceLoaded, false);
    m_box_shifted->addSignal("boxRotationError", maxRotationError, false);
  }
  else
  {
    //  m_box_shifted->addSignal("forceNonGravityRight", externalCollisionForce, false);
    //    m_box_shifted->addSignal("normalForceRight", handForceLoaded, false);
    //    m_box_shifted->addSignal("forceGravityRight", handForceLoaded, false);
    m_box_shifted->addSignal("boxRotationError", maxRotationError, false);
  }
  t_box_shifted_too_far->monitors_.push_back(m_box_shifted); // use same monitor as above

  s1->monitors_.push_back(m_box_shifted);

  //---------------------------------------------------------------------

  //  Transition_ptr t_box_remains_in_contact(new ActionProgressTransition(6, s0, s6, "box follows during regrasp"));
  //  t_box_remains_in_contact->deltaTmin_ = 0.0;
  //  t_box_remains_in_contact->deltaTmax_ = 1.0;
  //  s0->transitions_out_.push_back(t_box_remains_in_contact);
  //  s6->transitions_in_.push_back(t_box_remains_in_contact);
  //
  //  MonitorRequest_ptr
  //      m_box_remains_in_contact(new MonitorRequest("Progress", 6, 0, "box follows during regrasp", 0.0));
  //
  //  if (rightHandRegrasp)
  //  {
  //    //  m_box_remains_in_contact_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
  //    m_box_remains_in_contact->addSignal("normalForceRight", handForceLoaded, false);
  //    m_box_remains_in_contact->addSignal("forceGravityRight", handForceLoaded, false);
  //    m_box_remains_in_contact->addSignal("handDistanceRight", positionError, false);
  //  }
  //  else
  //  {
  //    //  m_box_remains_in_contact_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
  //    m_box_remains_in_contact->addSignal("normalForceLeft", handForceLoaded, false);
  //    m_box_remains_in_contact->addSignal("forceGravityLeft", handForceLoaded, false);
  //    m_box_remains_in_contact->addSignal("handDistanceLeft", positionError, false);
  //  }
  //
  //  t_box_remains_in_contact->monitors_.push_back(m_box_remains_in_contact); // use same monitor as above
  //
  //  s0->monitors_.push_back(m_box_remains_in_contact);

  //---------------------------------------------------------------------

  Transition_ptr t_box_aligned_again(new ActionProgressTransition(7, s5, s1, "box re-aligned"));
  t_box_aligned_again->deltaTmin_ = 0.0;
  t_box_aligned_again->deltaTmax_ = 10.0;
  s5->transitions_out_.push_back(t_box_aligned_again);
  s1->transitions_in_.push_back(t_box_aligned_again);

  MonitorRequest_ptr m_box_realigned(new MonitorRequest("Progress", 7, 0, "box re-aligned", 0.3));
  m_box_realigned->addSignal("boxRotationError", maxRotationError, true);

  t_box_aligned_again->monitors_.push_back(m_box_realigned);

  s5->monitors_.push_back(m_box_realigned);

  Transition_ptr t_attention_error(new ActionProgressTransition(8, s1, s7, "partner attention lost"));
  t_attention_error->deltaTmin_ = 0.0;
  t_attention_error->deltaTmax_ = 1.0;
  s1->transitions_out_.push_back(t_attention_error);
  s7->transitions_in_.push_back(t_attention_error);

  MonitorRequest_ptr m_checkGaze(new MonitorRequest("Progress", 8, 0, "partner not paying attention",
                                                    gazeAttentionDuration));
  m_checkGaze->addSignal("gazeError", gazeAttentionThreshold, false);

  t_attention_error->monitors_.push_back(m_checkGaze);
  s0->monitors_.push_back(m_checkGaze);

  Transition_ptr t_attention_recover(new ActionProgressTransition(9, s7, s1, "partner attention regained"));
  t_attention_recover->deltaTmin_ = 0.0;
  t_attention_recover->deltaTmax_ = 10.0;
  s7->transitions_out_.push_back(t_attention_recover);
  s1->transitions_in_.push_back(t_attention_recover);

  MonitorRequest_ptr m_checkGazeGood(new MonitorRequest("Progress", 9, 0, "partner paying attention",
                                                        gazeAttentionDuration));
  m_checkGazeGood->addSignal("gazeError", gazeAttentionThreshold, true);

  t_attention_recover->monitors_.push_back(m_checkGazeGood);
  s7->monitors_.push_back(m_checkGazeGood);



  tmp->states_.push_back(s0);
  tmp->states_.push_back(s1);
  tmp->states_.push_back(s2);
  tmp->states_.push_back(s3);
  tmp->states_.push_back(s4);
  tmp->states_.push_back(s5);
  tmp->states_.push_back(s6);
  tmp->states_.push_back(s7);

  tmp->transitions_.push_back(t_hand_release);
  tmp->transitions_.push_back(t_hand_recontact);
  //  tmp->transitions_.push_back(t_external_contact);
  //  tmp->transitions_.push_back(t_collision);
  //  tmp->transitions_.push_back(t_finshed_no_contact);
  tmp->transitions_.push_back(t_box_shifted_too_far);
  //  tmp->transitions_.push_back(t_box_remains_in_contact);
  tmp->transitions_.push_back(t_box_aligned_again);
  tmp->transitions_.push_back(t_attention_error);
  tmp->transitions_.push_back(t_attention_recover);

  return tmp;
}

Apg_ptr BoxObjectModel::createRotateObjectAPG() const
{

  double gazeAttentionThreshold =  RCS_DEG2RAD(60.0);
  double gazeAttentionDuration = 0.5;
  double duration = 6.0;

  Apg_ptr tmp2(new ActionProgressGraph(std::to_string(0), ("rotate object")));

  tmp2->duration_ = duration;

  ActionProgressState_ptr s8(new ActionProgressState());
  s8->id_ = "0";
  s8->description_ = "start";
  s8->action_ = tmp2;

  s8->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "CurrentAndGoal"));
  s8->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));

  ActionProgressState_ptr s9(new ActionProgressState());
  s9->id_ = "1";
  s9->description_ = "fault, partner not paying attention";
  s9->action_ = tmp2;
  s9->goal_ = false;
  s9->success_ = false;

  s9->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));

  Transition_ptr t_attention_error(new ActionProgressTransition(1, s8, s9, "partner attention lost"));
  t_attention_error->deltaTmin_ = 0.0;
  t_attention_error->deltaTmax_ = 1.0;
  s8->transitions_out_.push_back(t_attention_error);
  s9->transitions_in_.push_back(t_attention_error);

  MonitorRequest_ptr
  m_checkGaze(new MonitorRequest("Progress", 1, 0, "partner not paying attention", gazeAttentionDuration));
  m_checkGaze->addSignal("gazeError", gazeAttentionThreshold, false);

  t_attention_error->monitors_.push_back(m_checkGaze);
  s8->monitors_.push_back(m_checkGaze);

  Transition_ptr t_attention_recover(new ActionProgressTransition(1, s9, s8, "partner attention regained"));
  t_attention_recover->deltaTmin_ = 0.0;
  t_attention_recover->deltaTmax_ = 10.0;
  s9->transitions_out_.push_back(t_attention_recover);
  s8->transitions_in_.push_back(t_attention_recover);

  MonitorRequest_ptr
  m_checkGazeGood(new MonitorRequest("Progress", 1, 0, "partner paying attention", gazeAttentionDuration));
  m_checkGazeGood->addSignal("gazeError", gazeAttentionThreshold, true);

  t_attention_recover->monitors_.push_back(m_checkGazeGood);
  s9->monitors_.push_back(m_checkGazeGood);

  tmp2->states_.push_back(s8);
  tmp2->states_.push_back(s9);

  tmp2->transitions_.push_back(t_attention_error);
  tmp2->transitions_.push_back(t_attention_recover);

  return tmp2;
}

/*******************************************************************************
 * Transition Types
 ******************************************************************************/
unsigned int BoxObjectModel::getTransitionTypeSingle(int phi0, int rh0, int lh0,
                                                     int phi1, int rh1, int lh1)
{
  unsigned int tt;

  if (phi0!=phi1)
  {
    tt = RotateObject;

    if ((rh0!=rh1) || (lh0!=lh1))
    {
      tt = TransitionUndefined;
    }
  }
  else if (rh0 != rh1)
  {
    tt = RegraspRight;

    if ((phi0!=phi1) || (lh0!=lh1))
    {
      tt = TransitionUndefined;
    }
  }
  else if (lh0 != lh1)
  {
    tt = RegraspLeft;

    if ((phi0!=phi1) || (rh0!=rh1))
    {
      tt = TransitionUndefined;
    }
  }
  else
  {
    tt = None;
  }


  if (tt == TransitionUndefined)
  {
    RLOG(0, "Phi: %d -> %d, RH: %d -> %d, LH: %d -> %d",
         phi0, phi1, rh0, rh1, lh0, lh1);
  }

  return tt;
}


unsigned int BoxObjectModel::getTransitionType(std::vector<int> from, std::vector<int> to) const
{
  return BoxObjectModel::getTransitionTypeStatic(from, to);
}

unsigned int BoxObjectModel::getTransitionTypeStatic(std::vector<int> s0, std::vector<int> s1)
{
  if (s0.size() != 5 || s1.size() != 5)
  {
    RLOG(0, "Sizes incorrect in state vector: %d, %d! Should be 5.",
         (int) s0.size(), (int) s1.size());
    return TransitionUndefined;
  }
  return getTransitionType(s0[0], s0[1], s0[2], s0[3], s0[4],
                           s1[0], s1[1], s1[2], s1[3], s1[4]);
}

unsigned int BoxObjectModel::getTransitionType(int phi0, int ra0, int la0, int rp0, int lp0,
                                               int phi1, int ra1, int la1, int rp1, int lp1)
{
  unsigned int ttRobo = BoxObjectModel::getTransitionTypeSingle(phi0, ra0, la0,
                                                                phi1, ra1, la1);

  unsigned int ttPartner = BoxObjectModel::getTransitionTypeSingle(phi0, rp0, lp0,
                                                                   phi1, rp1, lp1);

  unsigned int tt = TransitionUndefined;

  if (ttRobo == BoxObjectModel::RotateObject && ttPartner == BoxObjectModel::RotateObject)
  {
    tt = BothRotateObject;
  }
  else if (ttRobo == BoxObjectModel::None)
  {
    if (ttPartner == BoxObjectModel::RegraspRight)
    {
      tt = PartnerRegraspRight;
    }
    else if (ttPartner == BoxObjectModel::RegraspLeft)
    {
      tt = PartnerRegraspLeft;
    }
    else if (ttPartner == BoxObjectModel::None)
    {
      tt = BoxObjectModel::TransitionType::None;
    }
  }
  else if (ttPartner == BoxObjectModel::None)
  {
    if (ttRobo == BoxObjectModel::RegraspRight)
    {
      tt = RobotRegraspRight;
    }
    else if (ttRobo == BoxObjectModel::RegraspLeft)
    {
      tt = RobotRegraspLeft;
    }
  }

  if (tt == TransitionUndefined)
  {
    RLOG(0, "Undefined transition: Robo: %d, Human: %d",
         (int) ttRobo, (int) ttPartner);
  }

  return tt;
}

}
