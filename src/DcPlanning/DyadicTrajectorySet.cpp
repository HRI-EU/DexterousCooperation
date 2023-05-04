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

#include "DyadicTrajectorySet.h"

#include <ActivationSet.h>
#include <PoseConstraint.h>
#include <ConstraintFactory.h>

#include <Rcs_macros.h>
#include <Rcs_math.h>



namespace Dc
{

class RelGrip
{
public:

  RelGrip(tropic::TrajectoryControllerBase* tc,
          const std::string& namePhi_Obj,
          const std::string& nameXYZ_R,
          const std::string& nameXYZ_L,
          const std::string& nameABC_R,
          const std::string& nameABC_L,
          const std::string& nameXYZ_Obj,
          double deltaPhi);

  RelGrip(tropic::TrajectoryControllerBase* tc,
          const std::string& nameXYZ_R,
          const std::string& nameXYZ_L,
          const std::string& nameABC_R,
          const std::string& nameABC_L,
          double deltaPhi);

  virtual ~RelGrip();

  std::string getClassName() const;
  std::vector<int> getState() const;
  int getState(int idx) const;
  void setContactPoints(std::vector<HTr> contacts);
  bool isPartner() const;
  void setSlideMode(bool enable);
  bool getSlideMode() const;
  bool computeObjectPosition(double pos[3]) const;

  std::shared_ptr<tropic::ConstraintSet>
  createActivationSet(bool active) const;

  std::shared_ptr<tropic::ConstraintSet>
  moveTo(int phi, int rh, int lh, double t_goal) const;

  std::shared_ptr<tropic::ConstraintSet>
  moveFromTo(int phi0, int rh0, int lh0,
             int phi1, int rh1, int lh1,
             double t0, double duration) const;

  std::shared_ptr<tropic::ConstraintSet>
  moveHeight(int height, double t1) const;

private:

  void initFingers(tropic::TrajectoryControllerBase* tc);

  double deltaPhi;
  double wristAngle;
  bool slideMode;

  tropic::TrajectoryND* trajPhi;
  tropic::TrajectoryND* trajPosR;
  tropic::TrajectoryND* trajPosL;
  tropic::TrajectoryND* trajOriR;
  tropic::TrajectoryND* trajOriL;
  tropic::TrajectoryND* trajPosObj;

  const Rcs::Task* taskPhi;
  const Rcs::Task* taskPosR;
  const Rcs::Task* taskPosL;
  const Rcs::Task* taskOriR;
  const Rcs::Task* taskOriL;
  const Rcs::Task* taskPosObj;

  tropic::TrajectoryND* trajFingersL;
  tropic::TrajectoryND* trajFingersR;
  const Rcs::Task* taskFingersL;
  const Rcs::Task* taskFingersR;

  tropic::TrajectoryND* trajHandLinkL;
  tropic::TrajectoryND* trajHandLinkR;

  std::vector<HTr> contactPoints;
};



class TransitionModel
{
public:

  enum TransitionType
  {
    NoTransition = 0,
    Undefined,
    RotateObject,
    RegraspRight,
    RegraspLeft,
  };

  static TransitionType getTransitionType(int phi0, int rh0, int lh0,
                                          int phi1, int rh1, int lh1)
  {
    TransitionType tt = NoTransition;

    if (phi0!=phi1)
    {
      tt = RotateObject;

      if ((rh0!=rh1) || (lh0!=lh1))
      {
        tt = Undefined;
      }
    }
    else if (rh0 != rh1)
    {
      tt = RegraspRight;

      if ((phi0!=phi1) || (lh0!=lh1))
      {
        tt = Undefined;
      }
    }
    else if (lh0 != lh1)
    {
      tt = RegraspLeft;

      if ((phi0!=phi1) || (rh0!=rh1))
      {
        tt = Undefined;
      }
    }

    if (tt == Undefined)
    {
      RLOG(0, "Undefined transition: Phi: %d -> %d, RH: %d -> %d, LH: %d -> %d",
           phi0, phi1, rh0, rh1, lh0, lh1);
    }

    return tt;
  }

};

/*****************************************************************************
 *
 ****************************************************************************/
RelGrip::RelGrip(tropic::TrajectoryControllerBase* tc,
                 const std::string& namePhi_Obj,
                 const std::string& nameXYZ_R,
                 const std::string& nameXYZ_L,
                 const std::string& nameABC_R,
                 const std::string& nameABC_L,
                 const std::string& nameXYZ_Obj,
                 double deltaPhi_) :
  deltaPhi(deltaPhi_), wristAngle(RCS_DEG2RAD(20.0)), slideMode(true)
{
  this->taskPhi  = tc->getTask(namePhi_Obj);
  RCHECK(this->taskPhi);
  this->taskPosR = tc->getTask(nameXYZ_R);
  RCHECK(this->taskPosR);
  this->taskPosL = tc->getTask(nameXYZ_L);
  RCHECK(this->taskPosL);
  this->taskOriR = tc->getTask(nameABC_R);
  RCHECK(this->taskOriR);
  this->taskOriL = tc->getTask(nameABC_L);
  RCHECK(this->taskOriL);
  this->taskPosObj = tc->getTask(nameXYZ_Obj);
  RCHECK_MSG(this->taskPosObj, "%s", nameXYZ_Obj.c_str());

  this->trajPhi  = tc->getTrajectory(namePhi_Obj);
  RCHECK(this->trajPhi);
  this->trajPosR = tc->getTrajectory(nameXYZ_R);
  RCHECK(this->trajPosR);
  this->trajPosL = tc->getTrajectory(nameXYZ_L);
  RCHECK(this->trajPosL);
  this->trajOriR = tc->getTrajectory(nameABC_R);
  RCHECK(this->trajOriR);
  this->trajOriL = tc->getTrajectory(nameABC_L);
  RCHECK(this->trajOriL);
  this->trajPosObj = tc->getTrajectory(nameXYZ_Obj);
  RCHECK(this->trajPosObj);

  this->trajHandLinkL = tc->getTrajectory("RoboHand_L");
  this->trajHandLinkR = tc->getTrajectory("RoboHand_R");

  initFingers(tc);
}

/*****************************************************************************
 * Constructor for partner. Should not initialize fingers.
 ****************************************************************************/
RelGrip::RelGrip(tropic::TrajectoryControllerBase* tc,
                 const std::string& nameXYZ_R,
                 const std::string& nameXYZ_L,
                 const std::string& nameABC_R,
                 const std::string& nameABC_L,
                 double deltaPhi_) :
  deltaPhi(deltaPhi_), wristAngle(RCS_DEG2RAD(20.0)), slideMode(true)
{
  this->taskPhi  = NULL;
  this->taskPosR = tc->getTask(nameXYZ_R);
  RCHECK(this->taskPosR);
  this->taskPosL = tc->getTask(nameXYZ_L);
  RCHECK(this->taskPosL);
  this->taskOriR = tc->getTask(nameABC_R);
  RCHECK(this->taskOriR);
  this->taskOriL = tc->getTask(nameABC_L);
  RCHECK(this->taskOriL);
  this->taskPosObj = NULL;
  this->taskFingersR = NULL;
  this->taskFingersL = NULL;

  this->trajPhi  = NULL;
  this->trajPosR = tc->getTrajectory(nameXYZ_R);
  RCHECK(this->trajPosR);
  this->trajPosL = tc->getTrajectory(nameXYZ_L);
  RCHECK(this->trajPosL);
  this->trajOriR = tc->getTrajectory(nameABC_R);
  RCHECK(this->trajOriR);
  this->trajOriL = tc->getTrajectory(nameABC_L);
  RCHECK(this->trajOriL);
  this->trajPosObj = NULL;
  this->trajFingersR = NULL;
  this->trajFingersL = NULL;
}

/*****************************************************************************
 *
 ****************************************************************************/
RelGrip::~RelGrip()
{
}

/*****************************************************************************
 *
 ****************************************************************************/
void RelGrip::initFingers(tropic::TrajectoryControllerBase* tc)
{
  this->taskFingersR = tc->getTask("Fingers_R");
  this->taskFingersL = tc->getTask("Fingers_L");
  this->trajFingersR = tc->getTrajectory("Fingers_R");
  this->trajFingersL = tc->getTrajectory("Fingers_L");
}

/*****************************************************************************
 *
 ****************************************************************************/
void RelGrip::setContactPoints(std::vector<HTr> contacts)
{
  this->contactPoints = contacts;
}

/*****************************************************************************
 *
 ****************************************************************************/
std::string RelGrip::getClassName() const
{
  if (isPartner())
  {
    return std::string("RelGrip Partner");
  }

  return std::string("RelGrip Robot");
}

/*****************************************************************************
 *
 ****************************************************************************/
bool RelGrip::isPartner() const
{
  return this->trajPhi ? false : true;
}

/*****************************************************************************
 *
 ****************************************************************************/
std::vector<int> RelGrip::getState() const
{
  std::vector<int> state(3);
  double x[3];

  // Box rotation
  if (taskPhi)
  {
    taskPhi->computeX(x);
    state[0] = lround(x[2]/deltaPhi);
  }
  else
  {
    // case for human side of box (without actual box state)
    state[0] = -1;
  }

  // Right hand
  state[1] = 0;
  RCHECK(!contactPoints.empty());
  taskPosR->computeX(x);
  double minDist = Vec3d_distance(x, contactPoints[0].org);
  for (size_t i=1; i<contactPoints.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPoints[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[1] = i;
    }
  }

  // Left hand
  state[2] = 0;
  taskPosL->computeX(x);
  minDist = Vec3d_distance(x, contactPoints[0].org);
  for (size_t i=1; i<contactPoints.size(); ++i)
  {
    double dist_i = Vec3d_distance(x, contactPoints[i].org);
    if (dist_i < minDist)
    {
      minDist = dist_i;
      state[2] = i;
    }
  }

  return state;
}

/*****************************************************************************
 *
 ****************************************************************************/
int RelGrip::getState(int idx) const
{
  std::vector<int> state = getState();
  return state[idx];
}

/*****************************************************************************
 *
 ****************************************************************************/
void RelGrip::setSlideMode(bool enable)
{
  this->slideMode = enable;
}

/*****************************************************************************
 *
 ****************************************************************************/
bool RelGrip::getSlideMode() const
{
  return this->slideMode;
}

/*****************************************************************************
 *
 ****************************************************************************/
bool RelGrip::computeObjectPosition(double pos[3]) const
{
  if (!this->taskPosObj)
  {
    return false;
  }

  RCHECK_MSG(taskPosObj->getDim()==3, "%d", (int)taskPosObj->getDim());
  taskPosObj->computeX(pos);

  return true;
}

/*****************************************************************************
 *
 ****************************************************************************/
std::shared_ptr<tropic::ConstraintSet> RelGrip::moveTo(int phi, int rh, int lh, double t_goal) const
{
  std::vector<HTr> contacts = contactPoints;

  if ((rh<0) || ((size_t)rh >= contacts.size()))
  {
    RLOG(1, "Invalid state for next right hand: %d (should be 0...%d)",
         rh, (int)contacts.size());
    return nullptr;
  }

  if ((lh<0) || ((size_t)lh >= contacts.size()))
  {
    RLOG(1, "Invalid state for next left hand: %d (should be 0...%d)",
         lh, (int)contacts.size());
    return nullptr;
  }

  auto moveSet = std::make_shared<tropic::ActivationSet>();

  moveSet->addActivation(0.0, true, 0.0, this->trajPosR->getName());
  moveSet->addActivation(0.0, true, 0.0, this->trajPosL->getName());
  moveSet->addActivation(0.0, true, 0.0, this->trajOriR->getName());
  moveSet->addActivation(0.0, true, 0.0, this->trajOriL->getName());




  if (!isPartner())
  {
    moveSet->addActivation(0.0, true, 0.0, this->trajPhi->getName());
    moveSet->addActivation(0.0, true, 0.0, this->trajPosObj->getName());
    if (this->trajFingersR)
    {
      moveSet->addActivation(0.0, true, 0.0, this->trajFingersR->getName());
    }
    if (this->trajFingersL)
    {
      moveSet->addActivation(0.0, true, 0.0, this->trajFingersL->getName());
    }

    moveSet->add(std::make_shared<tropic::PositionConstraint>(t_goal, 0.0, 0.0,
                                                              deltaPhi*phi,
                                                              trajPhi->getName()));

    // If there is a rigid body relative pose constraint between hand and
    // frame, let's run this down to zero.
    if (trajHandLinkL && trajHandLinkR)
    {
      for (size_t i = 0; i < trajHandLinkL->getInternalDim(); ++i)
      {
        moveSet->add(t_goal, 0, trajHandLinkL->getTrajectory1D(i)->getName());
      }

      for (size_t i = 0; i < trajHandLinkR->getInternalDim(); ++i)
      {
        moveSet->add(t_goal, 0, trajHandLinkR->getTrajectory1D(i)->getName());
      }

    }
  }


  if (!isPartner())
  {
    HTr tRH, tLH;
    HTr_copy(&tRH, &contacts[rh]);
    HTr_copy(&tLH, &contacts[lh]);
    Mat3d_rotateSelfAboutXYZAxis(tRH.rot, 1, this->wristAngle);
    Mat3d_rotateSelfAboutXYZAxis(tLH.rot, 1, -this->wristAngle);

    moveSet->add(std::make_shared<tropic::PoseConstraint>(t_goal, &tRH,
                                                          trajPosR->getName(),
                                                          trajOriR->getName()));
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t_goal, &tLH,
                                                          trajPosL->getName(),
                                                          trajOriL->getName()));




    for (int i = 0; i<6; ++i)
    {
      if (i != 2)
      {
        if (trajFingersR)
        {
          moveSet->add(t_goal, 0.0,
                       trajFingersR->getTrajectory1D(i)->getName());
        }

        if (trajFingersL)
        {
          moveSet->add(t_goal, 0.0,
                       trajFingersL->getTrajectory1D(i)->getName());
        }
      }
    }

    if (trajFingersR && trajFingersL)
    {
      moveSet->add(t_goal, -RCS_DEG2RAD(85.0),
                   trajFingersR->getTrajectory1D(2)->getName());
      moveSet->add(t_goal, -RCS_DEG2RAD(85.0),
                   trajFingersL->getTrajectory1D(2)->getName());
      moveSet->add(t_goal, RCS_DEG2RAD(1.0),
                   trajFingersR->getTrajectory1D(6)->getName());
      moveSet->add(t_goal, RCS_DEG2RAD(1.0),
                   trajFingersL->getTrajectory1D(6)->getName());
    }
  }
  else
  {
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t_goal, &contacts[rh],
                                                          trajPosR->getName(),
                                                          trajOriR->getName()));
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t_goal, &contacts[lh],
                                                          trajPosL->getName(),
                                                          trajOriL->getName()));
  }

  return moveSet;
}

/*****************************************************************************
 *
 ****************************************************************************/
std::shared_ptr<tropic::ConstraintSet>
RelGrip::moveFromTo(int phi0, int rh0, int lh0,
                    int phi1, int rh1, int lh1,
                    double t0, double duration) const
{
  const unsigned int regraspFlag = 1;

  std::vector<HTr> contacts = contactPoints;

  if ((rh0<0) || ((size_t)rh0>=contacts.size()))
  {
    RLOG(1, "Invalid state for previous right hand: %d (should be 0...%d)",
         rh0, (int) contacts.size());
    return nullptr;
  }

  if ((lh0<0) || ((size_t)lh0>=contacts.size()))
  {
    RLOG(1, "Invalid state for previous left hand: %d (should be 0...%d)",
         lh0, (int) contacts.size());
    return nullptr;
  }

  if ((rh1<0) || ((size_t)rh1>=contacts.size()))
  {
    RLOG(1, "Invalid state for next right hand: %d (should be 0...%d)",
         rh1, (int) contacts.size());
    return nullptr;
  }

  if ((lh1<0) || ((size_t)lh1>=contacts.size()))
  {
    RLOG(1, "Invalid state for next left hand: %d (should be 0...%d)",
         lh1, (int) contacts.size());
    return nullptr;
  }

  auto moveSet = std::make_shared<tropic::ActivationSet>();

  unsigned int transition = TransitionModel::getTransitionType(phi0, rh0, lh0,
                                                               phi1, rh1, lh1);

  const double t1 = t0 + duration;
  const double phiBox = deltaPhi*phi1;
  const double backDist = 0.15;
  const double swayDist = 0.1;
  const double bumpDist = 0.005;
  const double bumpTime = 0.05;


  RLOG(5, "Adding transition %d %d %d - %d %d %d",
       phi0, rh0, lh0, phi1, rh1, lh1);



  if (!isPartner())
  {
    moveSet->add(std::make_shared<tropic::PositionConstraint>(t1, 0.0, 0.0, phiBox,
                                                              trajPhi->getName()));

    HTr tRH, tLH;
    HTr_copy(&tRH, &contacts[rh1]);
    HTr_copy(&tLH, &contacts[lh1]);
    Mat3d_rotateSelfAboutXYZAxis(tRH.rot, 1, this->wristAngle);
    Mat3d_rotateSelfAboutXYZAxis(tLH.rot, 1, -this->wristAngle);

    moveSet->add(std::make_shared<tropic::PoseConstraint>(t1, &tRH,
                                                          trajPosR->getName(),
                                                          trajOriR->getName()));
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t1, &tLH,
                                                          trajPosL->getName(),
                                                          trajOriL->getName()));
  }
  else
  {
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t1, &contacts[rh1],
                                                          trajPosR->getName(),
                                                          trajOriR->getName()));
    moveSet->add(std::make_shared<tropic::PoseConstraint>(t1, &contacts[lh1],
                                                          trajPosL->getName(),
                                                          trajOriL->getName()));
  }


  switch (transition)
  {
    case TransitionModel::RotateObject:
    {
      break;
    }

    case TransitionModel::RegraspRight:
    {
      // If the contact normal doesn't differ, we slide along the surface
      if (getSlideMode() &&
          Mat3d_diffAngle(contacts[rh0].rot, contacts[rh1].rot)<1.0e-3)
      {
        break;
      }

      // Right hand backwards retract: to be applied to position index 0
      // (x points into box forward direction)
      moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[2], 0.0, 0.0, regraspFlag,
                   trajPosR->getTrajectory1D(2)->getName());
      moveSet->add(t0 + 2.0*bumpTime*duration, contacts[rh0].org[2], 0.0, 0.0, regraspFlag,
                   trajPosR->getTrajectory1D(2)->getName());
      moveSet->add(t0 + 0.5*duration, contacts[rh0].org[2] + backDist, 0.0, 0.0, regraspFlag,
                   trajPosR->getTrajectory1D(2)->getName());

      // Right hand sideways retract
      double invNormal[3], swayDir[3];
      Vec3d_copy(invNormal, contacts[rh0].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[0] + bumpDist*invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosR->getTrajectory1D(0)->getName());
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[1], 0.0, 0.0, regraspFlag,
                     trajPosR->getTrajectory1D(1)->getName());
        moveSet->add(t0 + 2.0*bumpTime*duration, contacts[rh0].org[1], 0.0, 0.0,
                     regraspFlag, trajPosR->getTrajectory1D(1)->getName());
        moveSet->add(t0 + 0.3*duration, contacts[rh0].org[0] + swayDir[0], 0.0, 0.0,
                     regraspFlag, trajPosR->getTrajectory1D(0)->getName());
      }
      else
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[1] + bumpDist*invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosR->getTrajectory1D(1)->getName());
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosR->getTrajectory1D(0)->getName());
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[rh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosR->getTrajectory1D(0)->getName());
        moveSet->add(t0 + 0.3*duration, contacts[rh0].org[1] + swayDir[1],
                     0.0, 0.0, regraspFlag, trajPosR->getTrajectory1D(1)->getName());
      }

      // moveSet->addA(new MyActivationPoint(t0+0.2*duration, false, 0.2*duration,
      //                            trajOriR));

      // Right hand sideways approach
      Vec3d_copy(invNormal, contacts[rh1].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        moveSet->add(t0 + 0.7*duration, contacts[rh1].org[0] + swayDir[0], 0.0, 0.0,
                     regraspFlag, trajPosR->getTrajectory1D(0)->getName());
      }
      else
      {
        moveSet->add(t0 + 0.7*duration, contacts[rh1].org[1] + swayDir[1], 0.0, 0.0,
                     regraspFlag, trajPosR->getTrajectory1D(1)->getName());
      }

      // moveSet->addA(new MyActivationPoint(t0+0.5*duration, true, 0.4*duration,
      //                            trajOriR));

      // if (!isPartner())
      // {
      //   moveSet->add(t0, -RCS_DEG2RAD(85.0), trajFingersR->getTrajectory1D(2));
      //   moveSet->add(t0+0.2*duration, -RCS_DEG2RAD(85.0), trajFingersR->getTrajectory1D(2));
      //   moveSet->add(t0+0.6*duration, -RCS_DEG2RAD(40.0), trajFingersR->getTrajectory1D(2));
      //   moveSet->add(t0+0.8*duration, -RCS_DEG2RAD(85.0), trajFingersR->getTrajectory1D(2));
      //   moveSet->add(t0+1.0*duration, -RCS_DEG2RAD(85.0), trajFingersR->getTrajectory1D(2));
      // }

      break;
    }

    case TransitionModel::RegraspLeft:
    {
      // If the contact normal doesn't differ, we slide along the surface
      if (getSlideMode() && Mat3d_diffAngle(contacts[lh0].rot, contacts[lh1].rot)<1.0e-3)
      {
        break;
      }

      // Left hand backwards retract: to be applied to position index 0
      // (x points into box forward direction)

      moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[2], 0.0, 0.0, regraspFlag,
                   trajPosL->getTrajectory1D(2)->getName());
      moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[2], 0.0, 0.0,
                   regraspFlag, trajPosL->getTrajectory1D(2)->getName());
      moveSet->add(t0 + 0.5*duration, contacts[lh0].org[2] + backDist, 0.0, 0.0,
                   regraspFlag, trajPosL->getTrajectory1D(2)->getName());

      // Left hand sideways retract
      double invNormal[3];
      double swayDir[3];
      Vec3d_copy(invNormal, contacts[lh0].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[0] + bumpDist*invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(0)->getName());
        // hold box steady during bump
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[1],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(1)->getName());
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[1],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(1)->getName());
        moveSet->add(t0 + 0.3*duration, contacts[lh0].org[0] + swayDir[0],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(0)->getName());
      }
      else
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[1] + bumpDist*invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(1)->getName());
        // hold box steady during bump
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(0)->getName());
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(0)->getName());
        moveSet->add(t0 + 0.3*duration, contacts[lh0].org[1] + swayDir[1],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(1)->getName());
      }

      // moveSet->addA(new MyActivationPoint(t0+0.2*duration, false, 0.2*duration,
      //                            trajOriL));

      // Left hand sideways approach
      Vec3d_copy(invNormal, contacts[lh1].rot[1]);
      Vec3d_constMulSelf(invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        moveSet->add(t0 + 0.7*duration, contacts[lh1].org[0] + invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(0)->getName());
      }
      else
      {
        moveSet->add(t0 + 0.7*duration, contacts[lh1].org[1] + invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosL->getTrajectory1D(1)->getName());
      }

      // moveSet->addA(new MyActivationPoint(t0+0.5*duration, true, 0.4*duration,
      //                            trajOriL));

      // if (!isPartner())
      // {
      //   moveSet->add(t0, -RCS_DEG2RAD(85.0), trajFingersL->getTrajectory1D(2));
      //   moveSet->add(t0+0.2*duration, -RCS_DEG2RAD(85.0), trajFingersL->getTrajectory1D(2));
      //   moveSet->add(t0+0.6*duration, -RCS_DEG2RAD(40.0), trajFingersL->getTrajectory1D(2));
      //   moveSet->add(t0+0.8*duration, -RCS_DEG2RAD(85.0), trajFingersL->getTrajectory1D(2));
      //   moveSet->add(t0+1.0*duration, -RCS_DEG2RAD(85.0), trajFingersL->getTrajectory1D(2));
      // }

      break;
    }

    default:
    {
      RLOG(1, "Unsupported transition %d:   %d %d %d - %d %d %d",
           transition, phi0, rh0, lh0, phi1, rh1, lh1);
      break;
    }

  }   // switch (transition)

  return moveSet;
}

/*****************************************************************************
 *
 ****************************************************************************/
std::shared_ptr<tropic::ConstraintSet> RelGrip::moveHeight(int height,
                                                           double t1) const
{
  if (this->trajPosObj==NULL)
  {
    return nullptr;
  }

  auto moveSet = std::make_shared<tropic::ConstraintSet>();
  double pos[3];
  taskPosObj->computeX(pos);
  pos[2] = 0.1*height;
  moveSet->add(std::make_shared<tropic::PositionConstraint>(t1, pos,
                                                            trajPosObj->getName()));

  return moveSet;
}

/*****************************************************************************
*
 ****************************************************************************/
std::shared_ptr<tropic::ConstraintSet>
RelGrip::createActivationSet(bool flag) const
{
  auto moveSet = std::make_shared<tropic::ActivationSet>();

  moveSet->addActivation(0.0, flag, 0.0, this->trajPosR->getName());
  moveSet->addActivation(0.0, flag, 0.0, this->trajPosL->getName());
  moveSet->addActivation(0.0, flag, 0.0, this->trajOriR->getName());
  moveSet->addActivation(0.0, flag, 0.0, this->trajOriL->getName());

  if (!isPartner())
  {
    moveSet->addActivation(0.0, flag, 0.0, this->trajPhi->getName());
    moveSet->addActivation(0.0, flag, 0.0, this->trajPosObj->getName());
    if (this->trajFingersR)
    {
      moveSet->addActivation(0.0, flag, 0.0,this->trajFingersR->getName());
    }
    if (this->trajFingersL)
    {
      moveSet->addActivation(0.0, flag, 0.0, this->trajFingersL->getName());
    }
  }

  return moveSet;
}








/*****************************************************************************
 *
 ****************************************************************************/
DyadicTrajectorySet::DyadicTrajectorySet(tropic::TrajectoryControllerBase* tc_,
                                         double deltaPhi) :
  tta(NULL), ttp(NULL), controlPartner(true)
{
  this->tta = new RelGrip(tc_, "Phi_Box", "XYZ_R", "XYZ_L", "ABC_R", "ABC_L",
                          "XYZ_Box", deltaPhi);
  this->ttp = new RelGrip(tc_, "Partner_XYZ_R", "Partner_XYZ_L",
                          "Partner_ABC_R", "Partner_ABC_L", deltaPhi);
}

DyadicTrajectorySet::~DyadicTrajectorySet()
{
  delete this->tta;
  delete this->ttp;
}

void DyadicTrajectorySet::setContactPoints(std::vector<HTr> roboContacts,
                                           std::vector<HTr> partnerContacts)
{
  tta->setContactPoints(roboContacts);
  ttp->setContactPoints(partnerContacts);
}

std::vector<int> DyadicTrajectorySet::getState(int withBoxHeight) const
{
  std::vector<int> roboState = tta->getState();
  std::vector<int> partnerState = ttp->getState();

  std::vector<int> state;
  state.push_back(roboState[0]);
  state.push_back(roboState[1]);
  state.push_back(roboState[2]);
  state.push_back(partnerState[1]);
  state.push_back(partnerState[2]);

  if (withBoxHeight)
  {
    // Box heigth
    double objectPos[3];
    Vec3d_set(objectPos, -1.0, -1.0, -1.0);
    tta->computeObjectPosition(objectPos);
    state.push_back(lround(10.0*objectPos[2]));
  }

  return state;
}


void DyadicTrajectorySet::setSlideMode(bool enable)
{
  tta->setSlideMode(enable);
  ttp->setSlideMode(enable);
}

bool DyadicTrajectorySet::getSlideMode() const
{
  return tta->getSlideMode() && ttp->getSlideMode();
}


// \todo: No effect
void DyadicTrajectorySet::setPartnerActive(bool enable)
{
  controlPartner = enable;
  auto moveSet = std::make_shared<tropic::ConstraintSet>();
  moveSet->add(ttp->createActivationSet(controlPartner));
}

std::shared_ptr<tropic::ConstraintSet>
DyadicTrajectorySet::createSolutionSet(std::vector<std::vector<int> > sln,
                                       double ttc) const
{
  if (sln.empty())
  {
    RLOG(1, "Solution is empty - not adding trajectory constraints");
    return nullptr;
  }

  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  double tReach = 0.0;
  for (size_t i=0; i<sln.size()-1; ++i)
  {
    RLOG(1, "Adding transition %d %d %d %d %d - %d %d %d %d %d",
         sln[i][0], sln[i][1], sln[i][2], sln[i][3], sln[i][4],
         sln[i+1][0], sln[i+1][1], sln[i+1][2], sln[i+1][3], sln[i+1][4]);


    std::vector<int> from, to;

    from.push_back(sln[i][0]);
    from.push_back(sln[i][1]);
    from.push_back(sln[i][2]);
    from.push_back(sln[i][3]);
    from.push_back(sln[i][4]);

    to.push_back(sln[i+1][0]);
    to.push_back(sln[i+1][1]);
    to.push_back(sln[i+1][2]);
    to.push_back(sln[i+1][3]);
    to.push_back(sln[i+1][4]);

    auto roboSet = tta->moveFromTo(sln[i][0], sln[i][1], sln[i][2],
                                   sln[i+1][0], sln[i+1][1], sln[i+1][2],
                                   tReach, ttc);
    if (roboSet==NULL)
    {
      return nullptr;
    }

    moveSet->add(roboSet);

    auto humanSet = ttp->moveFromTo(sln[i][0], sln[i][3], sln[i][4],
                                    sln[i+1][0], sln[i+1][3], sln[i+1][4],
                                    tReach, ttc);
    if (humanSet == nullptr)
    {
      return nullptr;
    }

    moveSet->add(humanSet);

    // Add height change
    if (sln[i].size() >=6)
    {
      int height0 = sln[i][5];
      auto hSet = tta->moveHeight(height0, tReach + ttc);

      if (hSet)
      {
        moveSet->add(hSet);
      }

    }

    tReach += ttc;
  }

  return moveSet;
}


std::shared_ptr<tropic::ConstraintSet>
DyadicTrajectorySet::createTransitionSet(int phi, int ra, int la,
                                         int rp, int lp, double ttc) const
{
  std::vector<int> from = getState();
  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  auto roboSet = tta->moveFromTo(from[0], from[1], from[2], phi, ra, la,
                                 0.0, ttc);
  if (roboSet == nullptr)
  {
    return nullptr;
  }
  else
  {
    moveSet->add(roboSet);
  }

  auto humanSet = ttp->moveFromTo(from[0], from[3], from[4], phi, rp, lp,
                                  0.0, ttc);
  if (humanSet == nullptr)
  {
    return nullptr;
  }
  else
  {
    moveSet->add(humanSet);
  }

  return moveSet;
}


std::shared_ptr<tropic::ConstraintSet>
DyadicTrajectorySet::createMoveToSet(int phi, int ra, int la,
                                     int rp, int lp, double ttc) const
{
  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  auto moveRobo = tta->moveTo(phi, ra, la, ttc);

  if (moveRobo != NULL)
  {
    moveSet->add(moveRobo);
  }
  else
  {
    return nullptr;
  }

  if (this->controlPartner)
  {
    auto movePartner = ttp->moveTo(phi, rp, lp, ttc);

    if (movePartner != nullptr)
    {
      moveSet->add(movePartner);
    }
    else
    {
      return nullptr;
    }

  }

  return moveSet;
}




std::shared_ptr<tropic::ConstraintSet>
DyadicTrajectorySet::createActivationSet(bool flag)
{
  auto moveSet = std::make_shared<tropic::ConstraintSet>();
  moveSet->add(tta->createActivationSet(flag));
  if (this->controlPartner)
  {
    moveSet->add(ttp->createActivationSet(flag));
  }

  return moveSet;
}

std::shared_ptr<tropic::ConstraintSet>
DyadicTrajectorySet::createMoveRobotToSet(int phi, int ra, int la,
                                          double ttc) const
{
  auto moveSet = tta->moveTo(phi, ra, la, ttc);

  if (moveSet == nullptr)
  {
    return nullptr;
  }

  return moveSet;
}


}   // namespace Dc
