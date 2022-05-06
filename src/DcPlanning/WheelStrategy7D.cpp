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

#include "WheelStrategy7D.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>
#include <Rcs_body.h>

/*
  Search state:

  0: s     Arc coordinate of wheel: [0 ... linearDiscretization[
  1: phi   Wheel flip angle: ]-inf ... inf[
  2: theta Wheel rolling angle: ]-inf ... inf[

  For the wheel trajectory, the states 0 and (n-1) correspond to
  preparatory states, with the hands a bit retracted from the
  wheel, and the fingers opened.

  For the hand contacts: even: palm normal points in wheel z-direction,
  odd: opposite to that

  3: robR  Robot right hand: [0 ... 2*contactDiscretization[
  4: robL  Robot left hand: [0 ... 2*contactDiscretization[
  5: humR  Human right hand: [0 ... 2*contactDiscretization[
  6: humL  Human left hand: [0 ... 2*contactDiscretization[

*/

//#define WITH_HUMAN
#define V3(x) (x)[0], (x)[1], (x)[2]
#define COND_RETURN(condition) if ((condition)) { return false;}


namespace Rcs
{

WheelStrategy7D::WheelStrategy7D(const RcsGraph* graph) :
  wheelRadius(-1.0), shoulderheight(0.35), linearDiscretization(12),
  flipDiscretization(12), rollDiscretization(8), contactDiscretization(64)
{
  RCHECK(check(graph));
  // ensure that there are states to place the wheel vertical and horizontal
  RCHECK(flipDiscretization % 4 == 0);

  initializeStateSpace(graph);
}

bool WheelStrategy7D::check(const RcsGraph* graph) const
{
  bool success = true;

  std::vector<std::string> jointNames =
  {
    "WheelY", "WheelZ", "WheelFlip", "WheelRoll",
    "ContactRobotRightThZ", "ContactRobotLeftThZ",
    "ContactHumanRightThZ", "ContactHumanLeftThZ",
    "ContactRobotRightThX", "ContactRobotLeftThX",
    "ContactHumanRightThX", "ContactHumanLeftThX"
  };

  for (size_t i = 0; i < jointNames.size(); ++i)
  {
    RcsJoint* jnt = RcsGraph_getJointByName(graph, jointNames[i].c_str());
    if (jnt == NULL)
    {
      RLOG(1, "Couldn't find joint \"%s\" in graph", jointNames[i].c_str());
      success = false;
    }
  }

  const RcsBody* wheelGeom = RcsGraph_getBodyByName(graph, "WheelGeom");

  if (wheelGeom == NULL)
  {
    RLOG(1, "Couldn't find body WheelGeom in graph - failed to initialize");
    success = false;
  }

  return success;
}

bool WheelStrategy7D::initializeStateSpace(const RcsGraph* graph)
{
  const RcsJoint* wheelY = RcsGraph_getJointByName(graph, "WheelY");
  const RcsJoint* wheelZ = RcsGraph_getJointByName(graph, "WheelZ");
  double y0 = graph->q->ele[wheelY->jointIndex];
  double z0 = graph->q->ele[wheelZ->jointIndex];

  const RcsBody* wheelStand = RcsGraph_getBodyByName(graph, "WheelStand");
  const RcsBody* wheelPole = RcsGraph_getBodyByName(graph, "WheelPole");
  RCHECK(wheelStand);
  RCHECK(wheelPole);

  for (int i = 0; i < linearDiscretization; ++i)
  {
    double tCoord = (double)i / (linearDiscretization - 1);
    double t = M_PI_2 + (double)M_PI_2*i / (linearDiscretization - 1);
    double bump = 0.2*sin(2*(double)M_PI_2*i / (linearDiscretization - 1));
    double yi = y0 + y0*cos(t);
    double zi = z0*sin(t)+bump;
    double xi = (1.0-tCoord)*(wheelStand->A_BI.org[1] - wheelPole->A_BI.org[1]);

    HTr trans;
    HTr_setIdentity(&trans);
    Vec3d_set(trans.org, xi, yi, zi);
    sState.push_back(trans);
  }

  std::reverse(sState.begin(), sState.end());

  //  // We manually move the first point next to the holder outwards, so that a
  //  // flip motion does not lead to a collision with the stand.
  //  Vec3d_copy(sState[linearDiscretization - 2].org, sState[linearDiscretization - 3].org);

  // for (size_t i=0;i<sState.size(); ++i)
  // {
  //   RLOG(0, "%zd: %f %f %f", i, sState[i].org[0], sState[i].org[1], sState[i].org[2]);
  // }

  // RcsGraph* sepp = (RcsGraph*) graph;
  // sepp->q->ele[wheelY->jointIndex] = sState.back().org[1];

  // Fat tire wheel radius
  this->wheelRadius = 0.37;

  return true;
}

double WheelStrategy7D::heuristicCost(const std::vector<int>& value) const
{
  RCHECK(!goalState.empty());
  double hCost = std::abs(value[WheelCoord] - goalState[WheelCoord]) +
                 std::abs(value[WheelFlip] - goalState[WheelFlip]);

  // In the start pose, the contact locations should be closely together when the goal is farther away from 0
  // double upScale = 0.1*(double) goalState[WheelCoord]/(linearDiscretization-1);
  // hCost += upScale*fabs(getGraspAngle(value[RoboContactLeft])-getGraspAngle(value[RoboContactRight]));

  // In the start pose, the contact locations should be closely together
  // double upScale = 0.001*(double) value[WheelCoord]/(linearDiscretization-1);
  // hCost += upScale*fabs(getGraspAngle(value[RoboContactLeft])-getGraspAngle(value[RoboContactRight]));

  // Just enforce the right hand to be close to the wheel center angle
  // double upScale = 0.01*(double) to[WheelCoord]/(linearDiscretization-1);
  // transitionCost += upScale*fabs(M_PI-getGraspAngle(to[RoboContactRight]));

  // Enforce symmetric grasp holds
  //if (value[WheelCoord]==0)
  {
    hCost += 0.1*(1.0/(double)contactDiscretization)*std::abs(value[RoboContactRight]-(contactDiscretization/2)+value[RoboContactLeft]-(contactDiscretization/2));
  }

  return hCost;
}

#if 0
double WheelStrategy7D::calculateTransitionCost(std::vector<int> from,
                                                std::vector<int> to) const
{
  double transitionCost = 0.0;

  if (from[WheelCoord] != to[WheelCoord] || from[WheelFlip] != to[WheelFlip])
  {
    transitionCost += fabs(from[WheelCoord] - to[WheelCoord]) +
                      fabs(from[WheelFlip] - to[WheelFlip]);


  }
  else
  {
    //double sumDist = getContactDistance(from[RoboContactRight], to[RoboContactRight]) +
    //                 getContactDistance(from[RoboContactLeft], to[RoboContactLeft]) +
    //                 getContactDistance(from[HumanContactRight], to[HumanContactRight]) +
    //                 getContactDistance(from[HumanContactLeft], to[HumanContactLeft]);
    //transitionCost += sumDist/ contactDiscretization;

    transitionCost += 0.1 * (10 * (from[WheelCoord] + 1) / linearDiscretization) ;

    //    if (from[WheelCoord] == to[WheelCoord])
    //    {
    //      transitionCost *= 0.05 * from[WheelCoord];
    //    }
  }

  // default cost for each action -> combined actions preferred over chaining singles
  transitionCost += 0.001;

  // Prefer flipping in the middle
  // if (from[WheelFlip] != to[WheelFlip])
  // {
  //   int sMiddle = (from[WheelCoord] + to[WheelCoord])/2;
  //   double dMiddle = Math_clip(sMiddle - (linearDiscretization/2), 0.0, 1.0);
  //   transitionCost += 0.1*(1.0-dMiddle);
  // }

  // In the end state, both hands should be symmetric
  // if (to[WheelCoord]==0)
  // {
  //   transitionCost += 10.1*(to[RoboContactRight]-32+to[RoboContactLeft]-32);
  // }

  return transitionCost;
}
#else
double WheelStrategy7D::transitionCost(const std::vector<int>& from,
                                       const std::vector<int>& to) const
{
  double transitionCost = 0.0;

  if (from[WheelCoord] != to[WheelCoord] || from[WheelFlip] != to[WheelFlip])
  {
    transitionCost += fabs(from[WheelCoord] - to[WheelCoord]) +
                      fabs(from[WheelFlip] - to[WheelFlip]);


  }
  else
  {
    //double sumDist = getContactDistance(from[RoboContactRight], to[RoboContactRight]) +
    //                 getContactDistance(from[RoboContactLeft], to[RoboContactLeft]) +
    //                 getContactDistance(from[HumanContactRight], to[HumanContactRight]) +
    //                 getContactDistance(from[HumanContactLeft], to[HumanContactLeft]);
    //transitionCost += sumDist/ contactDiscretization;

    transitionCost += 0.1 * (10 * (from[WheelCoord] + 1) / linearDiscretization) ;

    //    if (from[WheelCoord] == to[WheelCoord])
    //    {
    //      transitionCost *= 0.05 * from[WheelCoord];
    //    }
  }

  // default cost for each action -> combined actions preferred over chaining singles
  transitionCost += 0.001;

  // Prefer flipping in the middle
  // if (from[WheelFlip] != to[WheelFlip])
  // {
  //   int sMiddle = (from[WheelCoord] + to[WheelCoord])/2;
  //   double dMiddle = Math_clip(sMiddle - (linearDiscretization/2), 0.0, 1.0);
  //   transitionCost += 0.1*(1.0-dMiddle);
  // }

  return transitionCost;
}
#endif

void WheelStrategy7D::print() const
{
  RLOG(0, "wheelRadius = %f", this->wheelRadius);
  printf("WheelCoord: [0 ... %d[\n", linearDiscretization - 1);
  printf("WheelFlip: [-inf ... inf[, delta angle is %.2f deg\n",
         RCS_RAD2DEG(getDeltaFlip()));
  printf("WheelRoll: [-inf ... inf[, delta angle is %.2f deg\n",
         RCS_RAD2DEG(getDeltaRoll()));
  printf("RoboContactRight: [0 ... %d[\n", contactDiscretization - 1);
  printf("RoboContactLeft: [0 ... %d[\n", contactDiscretization - 1);
  printf("HumanContactRight: [0 ... %d[\n", contactDiscretization - 1);
  printf("HumanContactLeft: [0 ... %d[\n", contactDiscretization - 1);
}

bool WheelStrategy7D::checkState(std::vector<int> state) const
{
  return checkState(state[0], state[1], state[2], state[3], state[4],
                    state[5], state[6], false);
}

bool WheelStrategy7D::checkState(std::vector<int> state, bool testMe) const
{
  return checkState(state[0], state[1], state[2], state[3], state[4],
                    state[5], state[6], testMe);
}

bool WheelStrategy7D::checkState(int s, int flip, int roll,
                                 int robR, int robL, int humR, int humL,
                                 bool testing) const
{
  bool success = true;

  NLOG(9, "Checking state %d %d %d %d %d %d %d",
       s, flip, roll, robR, robL, humR, humL);

  // Move wheel from and on support rails only when upright
  if (s >= (linearDiscretization-2) && !wheelVertical(flip))
  {
    success = false;
    NLOG(1, "Wheel not upright on state %d", s);
    COND_RETURN(!testing);
  }

  // Lowest part of wheel must not be lower than support (virtual plane at
  // support point height). This implicitely leads to moving it down
  // horizontally.
  if (getWheelGap(s, flip) < 0.0)
  {
    success = false;
    NLOG(9, "Excluding s=%d: Height is below zero: %f (angle is %f)",
         s, getWheelGap(s, flip), RCS_RAD2DEG(getFlipAngle(flip)));
    COND_RETURN(!testing);
  }

  // Most left part of wheel must not be more left than support (virtual plane
  //  at support point).
  if (getLateralWheelGap(s, flip) < -1.0e-5)
  {
    success = false;
    NLOG(1, "Excluding s=%d: Sidewards position negative: %f (angle is %f)",
         s, getLateralWheelGap(s, flip), RCS_RAD2DEG(getFlipAngle(flip)));
    COND_RETURN(!testing);
  }

  // Rotate the hand contact points from the R-frame (wheel) into a frame that
  // is aligned with the inertial frame (but has zero offset).
  double A_RI[3][3];
  getWheelRotMat(A_RI, flip, roll);

  // Disallow grasping wheel at the center
  // if (robR==contactDiscretization/2 || robR==1+contactDiscretization/2)
  // {
  // success = false;
  //   COND_RETURN(!testing);
  // }

  // if (robL==contactDiscretization/2 || robL==1+contactDiscretization/2)
  // {
  // success = false;
  //   COND_RETURN(!testing);
  // }


  // Disallow grasping wheel at the center if the wheel is horizontal
  if ((robR==contactDiscretization/2 || robR==1+contactDiscretization/2) &&
      wheelHorizontal(flip))
  {
    success = false;
    COND_RETURN(!testing);
  }

  if ((robL==contactDiscretization/2 || robL==1+contactDiscretization/2) &&
      wheelHorizontal(flip))
  {
    success = false;
    COND_RETURN(!testing);
  }

  // Allow grasping wheel at the center only if the wheel is vertical
  // const double* wheelAxis = A_RI[2];
  // if (robR==contactDiscretization/2 && fabs(wheelAxis[2])>0.1) return false;
  // if (robL==contactDiscretization/2 && fabs(wheelAxis[2])>0.1) return false;




  // Robot right hand from R-frame to I-frame
  double r_robR[3];
  getHandInWorld(r_robR, robR, A_RI, wheelRadius);

  // Robot right hand must always be right from wheel
  if (r_robR[1] > 1.0e-3 || r_robR[0] > -0.25)
  {
    success = false;
    NLOG(9, "r_robR = %f %f %f", r_robR[0], r_robR[1], r_robR[2]);
    COND_RETURN(!testing);
  }

  // Robot left hand from R-frame to I-frame
  double r_robL[3];
  getHandInWorld(r_robL, robL, A_RI, wheelRadius);

  // Robot left hand must always be left from wheel
  if (r_robL[1] < -1.0e-3 || r_robL[0] > -0.25)
  {
    success = false;
    NLOG(9, "r_robL = %f %f %f", r_robL[0], r_robL[1], r_robL[2]);
    COND_RETURN(!testing);
  }

  // Allow only narrow grasps when wheel is not horizontal
  // if (fabs(wheelAxis[2])>0.1 && r_robR[0]>-0.3) return false;
  // if (fabs(wheelAxis[2])>0.1 && r_robL[0]>-0.3) return false;
  // if (s==linearDiscretization-1 && r_robR[0]>-0.3) return false;
  // if (s==linearDiscretization-1 && r_robL[0]>-0.3) return false;



  // Robot right and left hand must not occupy the same space
  if (Vec3d_distance(r_robL, r_robR) < 0.2)
  {
    success = false;
    NLOG(9, "Robot hand distance: %f", Vec3d_sqrDistance(r_robL, r_robR));
    COND_RETURN(!testing);
  }

  // Robot right palm normal: positive z-axis. If robR is odd, wrist is flipped.
  double palmNormalR[3];
  getRoboRightPalmNormal(palmNormalR, robR, A_RI);

  // RLOG(1, "palmNormalR = %f %f %f", palmNormalR[0], palmNormalR[1], palmNormalR[2]);
  // RLOG(1, "palmNormalR atan2 = %f", 180.0/M_PI*atan2(palmNormalR[1], palmNormalR[2]));


  // The right hand normal must point left
  if (palmNormalR[1] < -0.6)
  {
    success = false;
    NLOG(9, "Right robot hand normal points right");
    COND_RETURN(!testing);
  }

  // Robot left palm normal: negative z-axis. If robL is odd, wrist is flipped.
  double palmNormalL[3];
  getRoboLeftPalmNormal(palmNormalL, robL, A_RI);

  // RLOG(1, "palmNormalL = %f %f %f", palmNormalL[0], palmNormalL[1], palmNormalL[2]);
  // RLOG(1, "palmNormalL atan2 = %f", 180.0/M_PI*atan2(palmNormalL[1], palmNormalL[2]));

  // The left hand normal must point right
  if (palmNormalL[1] > 0.6)
  {
    success = false;
    NLOG(9, "Left robot hand normal points left: %f %f %f", V3(palmNormalL));
    COND_RETURN(!testing);
  }

  const double wheelHeight = sState[s].org[2];
  if (checkShoulderHeightAlignment(palmNormalR, palmNormalL,
                                   r_robR[2], r_robL[2],
                                   wheelHeight, testing) == false)
  {
    success = false;
    NLOG(9, "checkShoulderHeightAlignment failed");
    COND_RETURN(!testing);
  }

  // Check human pose
#if defined (WITH_HUMAN)
  if (checkHuman(humR, humL, A_RI, wheelHeight, testing)==false)
  {
    success = false;
    RLOG(9, "checkHuman failed");
    COND_RETURN(!testing);
  }
#endif

  // {
  // double phiRight = atan2(palmNormalR[1], palmNormalR[2]);
  // double phiLeft = atan2(palmNormalL[1], palmNormalL[2]);
  // RLOG(1, "phiRight = %f   phiLeft = %f", 180.0/M_PI*phiRight, 180.0/M_PI*phiLeft);
  // }


  return success;
}

bool WheelStrategy7D::checkStateFeasible(int s, int flip, int roll) const
{
  for (int robR = 0; robR < contactDiscretization; robR++)
  {
    for (int robL = 0; robL < contactDiscretization; robL++)
    {
      if (checkState(s, flip, roll, robR, robL, 0, 0))
      {
        return true;
      }
    }
  }

  return false;
}

bool WheelStrategy7D::checkTransition(int s0, int flip0, int roll0,
                                      int robR0, int robL0,
                                      int humR0, int humL0,
                                      int s1, int flip1, int roll1,
                                      int robR1, int robL1,
                                      int humR1, int humL1) const
{
  if ((s0==linearDiscretization - 1) && (!wheelVertical(flip1)))
  {
    return false;
  }
  if ((s1==linearDiscretization - 1) && (!wheelVertical(flip0)))
  {
    return false;
  }

  //TODO: do we need/want this?
  if ((s1==linearDiscretization - 1) && (s0!=linearDiscretization - 2))
  {
    return false;
  }
  //TODO: do we need/want this?
  if ((s1==0) && (s0!=1))
  {
    return false;
  }


  int numContactChanges = 0;

  if (robR0!=robR1)
  {
    numContactChanges++;
  }
  if (robL0!=robL1)
  {
    numContactChanges++;
  }
  if (humR0!=humR1)
  {
    numContactChanges++;
  }
  if (humL0!=humL1)
  {
    numContactChanges++;
  }

  // If no hand swings, the wheel must move somehow
  if (numContactChanges == 0)
  {
    if ((s0==s1) && (flip0==flip1) && (roll0==roll1))
    {
      return false;
    }
  }
  // If exactly one hand swings, nothing else should move
  else if (numContactChanges == 1)
  {
    if ((s0!=s1) || (flip0!=flip1) || (roll0!=roll1))
    {
      return false;
    }
  }
  // We disallow more than one hand to swing at a time.
  else if (numContactChanges > 1)
  {
    return false;
  }

  return true;
}

bool WheelStrategy7D::goalReached(const std::vector<int>& state) const
{
  if (state[0] != goalState[0])
  {
    return false;
  }

  if (state[1] != goalState[1])
  {
    return false;
  }

  return true;
}

std::vector<int> WheelStrategy7D::getState(const RcsGraph* graph) const
{
  double wheelPos[3];
  wheelPos[0] = 0.0;
  wheelPos[1] = RcsGraph_getJointValue(graph, "WheelY");
  wheelPos[2] = RcsGraph_getJointValue(graph, "WheelZ");

  std::vector<int> state(StateMaxIndex);

  double distance = Vec3d_sqrDistance(wheelPos, sState[0].org);
  for (size_t i = 1; i < sState.size(); ++i)
  {
    double di = Vec3d_sqrDistance(wheelPos, sState[i].org);

    if (di < distance)
    {
      distance = di;
      state[WheelCoord] = i;
    }
  }

  state[WheelFlip] = lround(RcsGraph_getJointValue(graph, "WheelFlip") / getDeltaFlip());
  state[WheelRoll] = lround(RcsGraph_getJointValue(graph, "WheelRoll") / getDeltaRoll());

  double wristRot, wheelRot;
  int upsideDown;

  wristRot = Math_fmodAngle(RcsGraph_getJointValue(graph, "ContactRobotRightThX"));
  upsideDown = fabs(wristRot) > M_PI_2 ? 1 : 0;
  wheelRot = fmod(RcsGraph_getJointValue(graph, "ContactRobotRightThZ"), 2.0 * M_PI);
  state[RoboContactRight] = 2 * lround(wheelRot / getDeltaGrasp()) + upsideDown;

  wristRot = Math_fmodAngle(RcsGraph_getJointValue(graph, "ContactRobotLeftThX"));
  upsideDown = fabs(wristRot) > M_PI_2 ? 1 : 0;
  wheelRot = fmod(RcsGraph_getJointValue(graph, "ContactRobotLeftThZ"), 2.0 * M_PI);
  state[RoboContactLeft] = 2 * lround(wheelRot / getDeltaGrasp()) + upsideDown;

  wristRot = Math_fmodAngle(RcsGraph_getJointValue(graph, "ContactHumanRightThX"));
  upsideDown = fabs(wristRot) > M_PI_2 ? 1 : 0;
  wheelRot = fmod(RcsGraph_getJointValue(graph, "ContactHumanRightThZ"), 2.0 * M_PI);
  state[HumanContactRight] = 2 * lround(wheelRot / getDeltaGrasp()) + upsideDown;

  wristRot = Math_fmodAngle(RcsGraph_getJointValue(graph, "ContactHumanLeftThX"));
  upsideDown = fabs(wristRot) > M_PI_2 ? 1 : 0;
  wheelRot = fmod(RcsGraph_getJointValue(graph, "ContactHumanLeftThZ"), 2.0 * M_PI);
  state[HumanContactLeft] = 2 * lround(wheelRot / getDeltaGrasp()) + upsideDown;

  return state;
}

void WheelStrategy7D::getQfromState(RcsGraph* graph,
                                    const std::vector<int> s) const
{
  RcsGraph_setJoint(graph, "WheelY", sState[s[WheelCoord]].org[1]);
  RcsGraph_setJoint(graph, "WheelZ", sState[s[WheelCoord]].org[2]);
  RcsGraph_setJoint(graph, "WheelFlip", getFlipAngle(s[WheelFlip]));
  RcsGraph_setJoint(graph, "WheelRoll", getRollAngle(s[WheelRoll]));

  RcsGraph_setJoint(graph, "ContactRobotRightThZ", getGraspAngle(s[RoboContactRight]));
  RcsGraph_setJoint(graph, "ContactRobotLeftThZ", getGraspAngle(s[RoboContactLeft]));
  RcsGraph_setJoint(graph, "ContactHumanRightThZ", getGraspAngle(s[HumanContactRight]));
  RcsGraph_setJoint(graph, "ContactHumanLeftThZ", getGraspAngle(s[HumanContactLeft]));

  RcsGraph_setJoint(graph, "ContactRobotRightThX", s[RoboContactRight] % 2 == 0 ? 0.0 : M_PI);
  RcsGraph_setJoint(graph, "ContactRobotLeftThX", s[RoboContactLeft] % 2 == 0 ? 0.0 : M_PI);
  RcsGraph_setJoint(graph, "ContactHumanRightThX", s[HumanContactRight] % 2 == 0 ? 0.0 : M_PI);
  RcsGraph_setJoint(graph, "ContactHumanLeftThX", s[HumanContactLeft] % 2 == 0 ? 0.0 : M_PI);
}

void WheelStrategy7D::printSolutionPath(const std::vector<std::vector<int>>& states)
{
  for (size_t i = 0; i < states.size(); ++i)
  {
    for (size_t j = 0; j < states[i].size(); ++j)
    {
      std::cout << states[i][j] << " ";
    }
    std::cout << std::endl;
  }

}

bool WheelStrategy7D::test(const RcsGraph* graph, int numIterations) const
{
  bool success = true;

  // RLOG(0, "Contact distance (1, 1) = %d", getContactDistance(1, 1));
  // RLOG(0, "Contact distance (1, 31) = %d", getContactDistance(1, 31));
  // RLOG(0, "Contact distance (1, -65) = %d", getContactDistance(1, -65));
  // RLOG(0, "Contact distance (0, -15) = %d", getContactDistance(0, 15));

  int contactDistance = getContactDistance(contactDiscretization - 1, contactDiscretization);
  if (contactDistance != 1)
  {
    RLOG(1, "Contact distance (%d, %d) = %d", contactDiscretization - 1, contactDiscretization, contactDistance);
    success = false;
  }

  std::vector<std::string> jointNames =
  {
    "WheelY", "WheelZ", "WheelFlip", "WheelRoll",
    "ContactRobotRightThZ", "ContactRobotLeftThZ",
    "ContactHumanRightThZ", "ContactHumanLeftThZ",
    "ContactRobotRightThX", "ContactRobotLeftThX",
    "ContactHumanRightThX", "ContactHumanLeftThX"
  };

  for (size_t i = 0; i < jointNames.size(); ++i)
  {
    RcsJoint* jnt = RcsGraph_getJointByName(graph, jointNames[i].c_str());
    if (jnt == NULL)
    {
      RLOG(1, "Couldn't find joint \"%s\" in graph", jointNames[i].c_str());
      success = false;
    }
  }

  const RcsBody* wheelGeom = RcsGraph_getBodyByName(graph, "WheelGeom");

  if (wheelGeom == NULL)
  {
    RLOG(1, "Couldn't find body WheelGeom in graph - failed to initialize");
    success = false;
  }

  RcsGraph* copyOfGraph = RcsGraph_clone(graph);

  for (int i = 0; i < numIterations; ++i)
  {
    std::vector<int> state1(StateMaxIndex);

    state1[WheelCoord] = Math_getRandomInteger(0, linearDiscretization - 1);
    state1[WheelFlip] = Math_getRandomInteger(-3 * flipDiscretization, 3 * flipDiscretization);
    state1[WheelRoll] = Math_getRandomInteger(-3 * rollDiscretization, 3 * rollDiscretization);

    for (size_t j = RoboContactRight; j < StateMaxIndex; ++j)
    {
      state1[j] = Math_getRandomInteger(0, contactDiscretization - 1);
    }

    getQfromState(copyOfGraph, state1);
    RcsGraph_setState(copyOfGraph, NULL, NULL);
    std::vector<int> state2 = getState(copyOfGraph);

    if (state1 != state2)
    {
      REXEC(0)
      {
        RLOG(0, "Mismatch:");
        print();
        RcsGraph_printState(copyOfGraph, copyOfGraph->q);
        for (size_t k = 0; k < state1.size(); ++k)
        {
          std::cout << state1[k] << " " << state2[k] << std::endl;
        }
      }
      success = false;
    }


    // Test wheel rotation matrix
    const RcsBody* wheelBdy = RcsGraph_getBodyByName(copyOfGraph, "Wheel");
    if (wheelBdy == NULL)
    {
      RLOG(1, "Couldn't find body \"Wheel\" in graph");
      success = false;
    }
    else
    {
      double A_FI[3][3], A_RF[3][3], A_RI[3][3];
      Mat3d_setRotMatX(A_FI, getFlipAngle(state1[WheelFlip]));
      Mat3d_setRotMatZ(A_RF, getRollAngle(state1[WheelRoll]));
      Mat3d_mul(A_RI, A_RF, A_FI);

      Mat3d_subSelf(A_RI, (double (*)[3])wheelBdy->A_BI.rot);
      double rmErr = Mat3d_maxAbsEle(A_RI);
      if (rmErr > 1.0e-6)
      {
        RLOG(1, "Rotation matrix error: %g", rmErr);
        success = false;
      }
    }

  }   // numiterations

  RcsGraph_destroy(copyOfGraph);

  return success;
}

bool WheelStrategy7D::checkShoulderHeightAlignment(const double palmNormalR[3],
                                                   const double palmNormalL[3],
                                                   double rightHandZ,
                                                   double leftHandZ,
                                                   double wheelHeight,
                                                   bool testing) const
{
  const double palmLimitZ = 0.9;

  // If the right hand is above the shoulder, the palm normal must point down.
  if (wheelHeight + rightHandZ > shoulderheight)
  {
    if (palmNormalR[2] > palmLimitZ)
    {
      RLOG(9, "Over shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalR), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }
  else
  {
    if (palmNormalR[2] < -palmLimitZ)
    {
      RLOG(9, "Under shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalR), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }

  // If the left hand is above the shoulder, the palm normal must point down.
  if (wheelHeight + leftHandZ > shoulderheight)
  {
    if (palmNormalL[2] > palmLimitZ)
    {
      RLOG(9, "Over shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalR), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }
  else
  {
    if (palmNormalL[2] < -palmLimitZ)
    {
      RLOG(9, "Under shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalL), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }

  return true;
}

bool WheelStrategy7D::checkHuman(int humR, int humL, double A_RI[3][3], double wheelHeight, bool testing) const
{
  const double palmLimitZ = 0.7;

  // Human right hand from R-frame to I-frame
  double r_humR[3];
  Vec3d_set(r_humR, cos(getGraspAngle(humR)), sin(getGraspAngle(humR)), 0.0);
  Vec3d_constMulSelf(r_humR, wheelRadius);
  Vec3d_transRotateSelf(r_humR, A_RI);

  // Human left hand from R-frame to I-frame
  double r_humL[3];
  Vec3d_set(r_humL, cos(getGraspAngle(humL)), sin(getGraspAngle(humL)), 0.0);
  Vec3d_constMulSelf(r_humL, wheelRadius);
  Vec3d_transRotateSelf(r_humL, A_RI);

  // Human right hand must always be left from wheel (it's mirrored)
  if (r_humR[1] < -1.0e-3 || r_humR[0] < 0.25)
  {
    RLOG(9, "r_humR = %f %f %f", r_humR[0], r_humR[1], r_humR[2]);
    if (!testing)
    {
      return false;
    }
  }

  // Human left hand must always be right from wheel (it's mirrored)
  if (r_humL[1] > 1.0e-3 || r_humL[0] < 0.25)
  {
    RLOG(9, "r_humL = %f %f %f", r_humL[0], r_humL[1], r_humL[2]);
    if (!testing)
    {
      return false;
    }
  }

  // Robot right and left hand must not occupy the same space
  if (Vec3d_sqrDistance(r_humL, r_humR) < 1.0e-3)
  {
    RLOG(9, "Human hand distance: %f", Vec3d_sqrDistance(r_humL, r_humR));
    if (!testing)
    {
      return false;
    }
  }

  // Robot right palm normal: positive z-axis. If robR is odd, wrist is flipped.
  double palmNormalR[3];
  Vec3d_set(palmNormalR, 0.0, 0.0, humR % 2 == 0 ? -1.0 : 1.0);
  Vec3d_transRotateSelf(palmNormalR, A_RI);

  // Robot left palm normal: negative z-axis. If robL is odd, wrist is flipped.
  double palmNormalL[3];
  Vec3d_set(palmNormalL, 0.0, 0.0, humL % 2 == 0 ? 1.0 : -1.0);
  Vec3d_transRotateSelf(palmNormalL, A_RI);

  if (wheelHeight > shoulderheight)
  {
    if (palmNormalR[2] > palmLimitZ || palmNormalL[2] > palmLimitZ)
    {
      RLOG(9, "Over shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalR), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }
  else
  {
    if (palmNormalR[2] < -palmLimitZ || palmNormalL[2] < -palmLimitZ)
    {
      RLOG(9, "Under shoulder height (%f): palmR=%f %f %f palmL=%f %f %f",
           wheelHeight, V3(palmNormalR), V3(palmNormalL));
      if (!testing)
      {
        return false;
      }
    }
  }

  //  RLOG(1, "palmR=%f %f %f", V3(palmNormalR));


  if (palmNormalR[1] > 0.9)
  {
    RLOG(9, "Right human hand normal points left");
    if (!testing)
    {
      return false;
    }
  }

  // The left hand normal must point right
  if (palmNormalL[1] < -0.9)
  {
    RLOG(9, "Left human hand normal points right");
    if (!testing)
    {
      return false;
    }
  }

  return true;
}


std::vector<std::vector<int>> WheelStrategy7D::exploreFlipTranslation(int s, int flip, int roll,
                                                                      int robR, int robL,
                                                                      int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  // Try to move the wheel to all positions along the s coordinate
  int s_min = 0;
  int s_max = sState.size();
  int s_steps = 18;
  s_min = (s - s_steps < 0) ? 0 : s - s_steps;
  s_max = (s + s_steps > (int)sState.size()) ? sState.size() : s + s_steps;

  // Try to flip the wheel to all angles within ]-180 : 180[ degrees
  int nRotSteps = lround(floor(M_PI / getDeltaFlip()));
  int flipMin = flip - nRotSteps;
  int flipMax = flip + nRotSteps;

  for (int i = s_min; i<s_max; ++i)
  {
    for (int j = flipMin; j<flipMax; ++j)
    {
      if (checkState(i, j, roll, robR, robL, humR, humL) &&
          checkTransition(s, flip, roll, robR, robL, humR, humL,
                          i, j, roll, robR, robL, humR, humL))
      {
        std::vector<int> newStateValues = { i, j, roll, robR, robL, humR, humL };
        nextStates.push_back(std::move(newStateValues));
      }
    }
  }

  return nextStates;
}



std::vector<std::vector<int>> WheelStrategy7D::exploreTranslation(int s, int flip, int roll,
                                                                  int robR, int robL,
                                                                  int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  // Try to move the wheel to all positions along the s coordinate
  int s_min = 0;
  int s_max = sState.size();
  int s_steps = 6;
  s_min = (s - s_steps < 0) ? 0 : s - s_steps;
  s_max = (s + s_steps > (int)sState.size()) ? sState.size() : s + s_steps;

  if (s==0)
  {
    // s_min = 0;
    // s_max = 4;
  }
  else if (s==linearDiscretization-1)
  {
    s_min = linearDiscretization-2;
    s_max = linearDiscretization-1;
  }

  for (int i = s_min; i<s_max; ++i)
  {
    if (checkState(i, flip, roll, robR, robL, humR, humL) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        i, flip, roll, robR, robL, humR, humL))
    {
      std::vector<int> newStateValues = {i, flip, roll, robR, robL, humR, humL};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::exploreFlip(int s, int flip, int roll,
                                                           int robR, int robL,
                                                           int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  // Try to flip the wheel to all angles within ]-180 : 180[ degrees
  int nRotSteps = lround(floor(M_PI / getDeltaFlip()));
  int flipMin = flip - nRotSteps;
  int flipMax = flip + nRotSteps;

  //flipMin = Math_iClip(flipMin, -flipDiscretization-1, flipDiscretization+1);
  //flipMax = Math_iClip(flipMax, -flipDiscretization-1, flipDiscretization+1);

  for (int i = flipMin; i < flipMax; ++i)
  {
    if (checkState(s, i, roll, robR, robL, humR, humL) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        s, i, roll, robR, robL, humR, humL))
    {
      std::vector<int> newStateValues = {s, i, roll, robR, robL, humR, humL};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::exploreRoboRight(int s, int flip, int roll,
                                                                int robR, int robL,
                                                                int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  // Try to move hands to neighboring states
  //for (int i=0; i<contactDiscretization; ++i)
  for (int i = robR - contactDiscretization / 2; i < robR + contactDiscretization / 2; ++i)
  {
    if (checkState(s, flip, roll, i, robL, humR, humL) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        s, flip, roll, i, robL, humR, humL))
    {
      std::vector<int> newStateValues = {s, flip, roll, i, robL, humR, humL};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::exploreRoboLeft(int s, int flip, int roll,
                                                               int robR, int robL,
                                                               int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  for (int i = 0; i < contactDiscretization; ++i)
  {
    if (checkState(s, flip, roll, robR, i, humR, humL) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        s, flip, roll, robR, i, humR, humL))
    {
      std::vector<int> newStateValues = {s, flip, roll, robR, i, humR, humL};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::exploreHumanRight(int s, int flip, int roll,
                                                                 int robR, int robL,
                                                                 int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  // Try to move hands to neighboring states
  //for (int i=0; i<contactDiscretization; ++i)
  for (int i = humR - contactDiscretization / 2; i < humR + contactDiscretization / 2; ++i)
  {
    if (checkState(s, flip, roll, robR, robL, i, humL) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        s, flip, roll, robR, robL, i, humL))
    {
      std::vector<int> newStateValues = {s, flip, roll, robR, robL, i, humL};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::exploreHumanLeft(int s, int flip, int roll,
                                                                int robR, int robL,
                                                                int humR, int humL) const
{
  std::vector<std::vector<int>> nextStates;

  for (int i = 0; i < contactDiscretization; ++i)
  {

    if (checkState(s, flip, roll, robR, robL, humR, i) &&
        checkTransition(s, flip, roll, robR, robL, humR, humL,
                        s, flip, roll, robR, robL, humR, i))
    {
      std::vector<int> newStateValues = {s, flip, roll, robR, robL, humR, i};
      nextStates.push_back(std::move(newStateValues));
    }
  }

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::explore3(const std::vector<int>& currentState) const
{
  const int s = currentState[WheelCoord];
  const int flip = currentState[WheelFlip];
  const int roll = currentState[WheelRoll];
  const int robR = currentState[RoboContactRight];
  const int robL = currentState[RoboContactLeft];
  const int humR = currentState[HumanContactRight];
  const int humL = currentState[HumanContactLeft];

  std::vector<std::vector<int>> nextStates, states_i;

  states_i = exploreTranslation(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  states_i.clear();
  states_i = exploreFlip(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  states_i.clear();
  states_i = exploreRoboRight(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  states_i.clear();
  states_i = exploreRoboLeft(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  return nextStates;
}

std::vector<std::vector<int>> WheelStrategy7D::explore(const std::vector<int>& currentState) const
{
  const int s = currentState[WheelCoord];
  const int flip = currentState[WheelFlip];
  const int roll = currentState[WheelRoll];
  const int robR = currentState[RoboContactRight];
  const int robL = currentState[RoboContactLeft];
  const int humR = currentState[HumanContactRight];
  const int humL = currentState[HumanContactLeft];

  std::vector<std::vector<int>> nextStates, states_i, states_j;



#if 0
  states_i = exploreTranslation(s, flip, roll, robR, robL, humR, humL);

  for (size_t i = 0; i < states_i.size(); ++i)
  {
    states_j = exploreFlip(states_i[i][0], flip, roll, robR, robL, humR, humL);
    nextStates.insert(std::end(nextStates), std::begin(states_j), std::end(states_j));
  }

  states_i = exploreFlip(s, flip, roll, robR, robL, humR, humL);

  for (size_t i = 0; i < states_i.size(); ++i)
  {
    states_j = exploreTranslation(states_i[i][0], flip, roll, robR, robL, humR, humL);
    nextStates.insert(std::end(nextStates), std::begin(states_j), std::end(states_j));
  }

  // Remove duplicate entries
  auto last = std::unique(nextStates.begin(), nextStates.end());
  nextStates.erase(last, nextStates.end());
#else
  nextStates = exploreFlipTranslation(s, flip, roll, robR, robL, humR, humL);
#endif




  states_i = exploreRoboRight(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  states_i = exploreRoboLeft(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

#if defined (WITH_HUMAN)
  states_i = exploreHumanRight(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));

  states_i = exploreHumanLeft(s, flip, roll, robR, robL, humR, humL);
  nextStates.insert(std::end(nextStates), std::begin(states_i), std::end(states_i));
#endif

  return nextStates;
}

int WheelStrategy7D::getNumberOfContactChanges(std::vector<int> from,
                                               std::vector<int> to)
{
  int numContactChanges = 0;

  if (from[WheelStrategy7D::RoboContactRight] != to[WheelStrategy7D::RoboContactRight])
  {
    numContactChanges++;
  }
  if (from[WheelStrategy7D::RoboContactLeft] != to[WheelStrategy7D::RoboContactLeft])
  {
    numContactChanges++;
  }
  if (from[WheelStrategy7D::HumanContactRight] != to[WheelStrategy7D::HumanContactRight])
  {
    numContactChanges++;
  }
  if (from[WheelStrategy7D::HumanContactLeft] != to[WheelStrategy7D::HumanContactLeft])
  {
    numContactChanges++;
  }

  return numContactChanges;
}

unsigned int WheelStrategy7D::getTransitionType(std::vector<int> from, std::vector<int> to)
{
  if (getNumberOfContactChanges(from, to) > 1)
  {
    return WheelStrategy7D::TransitionUndefined;
  }

  unsigned int tType = WheelStrategy7D::None;

  // The object somehow moves
  if ((from[WheelStrategy7D::WheelCoord] != to[WheelStrategy7D::WheelCoord]) ||
      (from[WheelStrategy7D::WheelFlip] != to[WheelStrategy7D::WheelFlip]) ||
      (from[WheelStrategy7D::WheelRoll] != to[WheelStrategy7D::WheelRoll]))
  {
    tType = WheelStrategy7D::ObjectMoves;
  }
  else if (from[WheelStrategy7D::RoboContactRight] != to[WheelStrategy7D::RoboContactRight])
  {
    tType = WheelStrategy7D::RobotRegraspRight;
  }
  else if (from[WheelStrategy7D::RoboContactLeft] != to[WheelStrategy7D::RoboContactLeft])
  {
    tType = WheelStrategy7D::RobotRegraspLeft;
  }
  else if (from[WheelStrategy7D::HumanContactRight] != to[WheelStrategy7D::HumanContactRight])
  {
    tType = WheelStrategy7D::HumanRegraspRight;
  }
  else if (from[WheelStrategy7D::HumanContactLeft] != to[WheelStrategy7D::HumanContactLeft])
  {
    tType = WheelStrategy7D::HumanRegraspLeft;
  }

  return tType;
}

double WheelStrategy7D::getTtc(std::vector<int> from, std::vector<int> to, double baseTtc) const
{
  double nextTtc = baseTtc;
  unsigned int tType = WheelStrategy7D::getTransitionType(from, to);


  //TODO: add options to scale ttc by transition type and distance

  if (tType == WheelStrategy7D::ObjectMoves)
  {
    nextTtc = 2.0;
    //RLOG(0, "1 Next ttc is %f", nextTtc);

    double phiFlip = getFlipAngle(std::abs(to[WheelFlip]-from[WheelFlip]));
    nextTtc = std::max(nextTtc, (phiFlip/M_PI_2) * baseTtc);   // 90 degrees is full ttc
    //RLOG(0, "2 Next ttc is %f", nextTtc);

    double wheelTravel = Vec3d_distance(sState[from[WheelCoord]].org, sState[to[WheelCoord]].org);
    nextTtc = std::max(nextTtc, wheelTravel*baseTtc);  // 1 meter is full ttc
    //RLOG(0, "3 Next ttc is %f", nextTtc);
  }
  else if (tType == WheelStrategy7D::TransitionType::HumanRegraspLeft ||
           tType == WheelStrategy7D::TransitionType::HumanRegraspRight)
  {
    nextTtc = 1.0;
  }
  else if (tType == WheelStrategy7D::TransitionType::RobotRegraspRight)
  {
    int robR0 = from[WheelStrategy7D::RoboContactRight];
    int robR1 = to[WheelStrategy7D::RoboContactRight];
    int robL = from[WheelStrategy7D::RoboContactLeft];

    if (!contactInsideRange(robR0, robR1, robL) && ((robR0-robR1)%2==0))
    {
      nextTtc *= 0.5;
      RLOG(0, "Right hand just slides");
    }
  }
  else if (tType == WheelStrategy7D::TransitionType::RobotRegraspLeft)
  {
    int robL0 = from[WheelStrategy7D::RoboContactLeft];
    int robL1 = to[WheelStrategy7D::RoboContactLeft];
    int robR = from[WheelStrategy7D::RoboContactRight];

    if (!contactInsideRange(robL0, robL1, robR) && ((robL0-robL1)%2==0))
    {
      nextTtc *= 0.5;
      RLOG(0, "Left hand just slides");
    }
  }

  //RLOG(0, "Action: %s, baseTTc: %5.3f, scaled: %5.3f", transitionTypeToString(tType).c_str(), baseTtc, nextTtc);

  return nextTtc;
}

void WheelStrategy7D::setGoal(const std::vector<int>& goal)
{
  goalState = goal;
}

}   // namespace Rcs
