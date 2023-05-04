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

#include "PolygonObjectConstraint.h"

#include <ActivationSet.h>
#include <PoseConstraint.h>
#include <Rcs_macros.h>
#include <Rcs_math.h>

#include <string>
using namespace tropic;

namespace
{

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

    if (phi0 != phi1)
    {
      tt = RotateObject;

      if ((rh0 != rh1) || (lh0 != lh1))
      {
        tt = Undefined;
      }
    }
    else if (rh0 != rh1)
    {
      tt = RegraspRight;

      if ((phi0 != phi1) || (lh0 != lh1))
      {
        tt = Undefined;
      }
    }
    else if (lh0 != lh1)
    {
      tt = RegraspLeft;

      if ((phi0 != phi1) || (rh0 != rh1))
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

}   // namespace



namespace Dc
{

/*******************************************************************************
 *
 ******************************************************************************/
PolygonObjectConstraint::PolygonObjectConstraint(const std::string& namePhi_Obj,
                                                 const std::string& nameXYZ_R,
                                                 const std::string& nameXYZ_L,
                                                 const std::string& nameABC_R,
                                                 const std::string& nameABC_L,
                                                 const std::string& nameXYZ_Obj,
                                                 double deltaPhi_,
                                                 double deltaH_) :
  deltaPhi(deltaPhi_), deltaH(deltaH_), wristAngle(RCS_DEG2RAD(30.0)),
  slideMode(true)
{
  this->trajPhi  = namePhi_Obj;
  this->trajPosR = nameXYZ_R;
  this->trajPosL = nameXYZ_L;
  this->trajOriR = nameABC_R;
  this->trajOriL = nameABC_L;
  this->trajPosObj = nameXYZ_Obj;
  this->trajFingersR = "Fingers_R";
  this->trajFingersL = "Fingers_L";
}

/*******************************************************************************
 * Constructor for partner. Should not initialize fingers.
 ******************************************************************************/
PolygonObjectConstraint::PolygonObjectConstraint(const std::string& nameXYZ_R,
                                                 const std::string& nameXYZ_L,
                                                 const std::string& nameABC_R,
                                                 const std::string& nameABC_L,
                                                 double deltaPhi_,
                                                 double deltaH_) :
  deltaPhi(deltaPhi_), deltaH(deltaH_), wristAngle(RCS_DEG2RAD(30.0)),
  slideMode(true)
{
  this->trajPosR = nameXYZ_R;
  this->trajPosL = nameXYZ_L;
  this->trajOriR = nameABC_R;
  this->trajOriL = nameABC_L;
}

/*******************************************************************************
 *
 ******************************************************************************/
void PolygonObjectConstraint::setContactPoints(std::vector<HTr> contactPoints)
{
  this->contacts = contactPoints;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::string PolygonObjectConstraint::getClassName() const
{
  if (isPartner())
  {
    return std::string("PolygonObjectConstraint Partner");
  }

  return std::string("PolygonObjectConstraint Robot");
}

/*******************************************************************************
 *
 ******************************************************************************/
bool PolygonObjectConstraint::isPartner() const
{
  return this->trajPhi.empty() ? true : false;
}

/*******************************************************************************
*
******************************************************************************/
void PolygonObjectConstraint::setSlideMode(bool enable)
{
  this->slideMode = enable;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool PolygonObjectConstraint::getSlideMode() const
{
  return this->slideMode;
}

/*******************************************************************************
 *
 ******************************************************************************/
TCS_sptr PolygonObjectConstraint::moveTo(int phi, int rh, int lh,
                                         double t_goal) const
{
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

  auto moveSet = std::make_shared<ActivationSet>();

  moveSet->addActivation(0.0, true, 0.0, this->trajPosR);
  moveSet->addActivation(0.0, true, 0.0, this->trajPosL);
  moveSet->addActivation(0.0, true, 0.0, this->trajOriR);
  moveSet->addActivation(0.0, true, 0.0, this->trajOriL);

  if (!isPartner())
  {
    moveSet->addActivation(0.0, true, 0.0, this->trajPhi);
    moveSet->addActivation(0.0, true, 0.0, this->trajPosObj);
    if (!trajFingersR.empty())
    {
      moveSet->addActivation(0.0, true, 0.0, this->trajFingersR);
    }
    if (!trajFingersL.empty())
    {
      moveSet->addActivation(0.0, true, 0.0, this->trajFingersL);
    }

    // Keep axis orientation of polygon object
    moveSet->add(t_goal, deltaPhi*phi, 0.0, 0.0, 7, trajPhi + " 2");


    // Re-align axis to x / y = 0
    // moveSet->add(std::make_shared<PositionConstraint>(t_goal, 0.0, 0.0,
    //                                                   deltaPhi*phi, trajPhi));


    moveSet->addActivation(0.0, true, 0.0, "FrontalAlignBox");
    moveSet->add(t_goal, 0.0, 0.0, 0.0, 7, "FrontalAlignBox 0");
  }


  if (!isPartner())
  {
    HTr tRH, tLH;
    HTr_copy(&tRH, &contacts[rh]);
    HTr_copy(&tLH, &contacts[lh]);
    Mat3d_rotateSelfAboutXYZAxis(tRH.rot, 1, this->wristAngle);
    Mat3d_rotateSelfAboutXYZAxis(tLH.rot, 1, -this->wristAngle);

    moveSet->add(std::make_shared<PoseConstraint>(t_goal, &tRH,
                                                  trajPosR, trajOriR));
    moveSet->add(std::make_shared<PoseConstraint>(t_goal, &tLH,
                                                  trajPosL, trajOriL));




    for (int i = 0; i<6; ++i)
    {
      if (i != 2)
      {
        if (!trajFingersR.empty())
        {
          moveSet->add(t_goal, 0.0, trajFingersR+" " + std::to_string(i));
        }

        if (!trajFingersL.empty())
        {
          moveSet->add(t_goal, 0.0, trajFingersL + " " + std::to_string(i));
        }
      }
    }

    if (!trajFingersR.empty() && !trajFingersL.empty())
    {
      moveSet->add(t_goal, -RCS_DEG2RAD(85.0), trajFingersR + " 2");
      moveSet->add(t_goal, -RCS_DEG2RAD(85.0), trajFingersL + " 2");
      moveSet->add(t_goal, RCS_DEG2RAD(1.0), trajFingersR + " 6");
      moveSet->add(t_goal, RCS_DEG2RAD(1.0), trajFingersL + " 6");
    }
  }
  else
  {
    moveSet->add(std::make_shared<PoseConstraint>(t_goal, &contacts[rh],
                                                  trajPosR, trajOriR));
    moveSet->add(std::make_shared<PoseConstraint>(t_goal, &contacts[lh],
                                                  trajPosL, trajOriL));
  }

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
TCS_sptr PolygonObjectConstraint::moveFromTo(int phi0, int rh0, int lh0,
                                             int phi1, int rh1, int lh1,
                                             double t0, double duration) const
{
  const unsigned int regraspFlag = 1;

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

  auto moveSet = std::make_shared<ActivationSet>();
  moveSet->setClassName("MoveFromTo");

  unsigned int transition = TransitionModel::getTransitionType(phi0, rh0, lh0,
                                                               phi1, rh1, lh1);

  const double t1 = t0 + duration;
  const double phiBox = deltaPhi*phi1;
  const double backDist = 0.05;   // was 0.2
  const double swayDist = 0.1;   // was 0.1
  const double bumpDist = 0.005;
  const double bumpTime = 0.05;
  const double fngrOpenAngle = RCS_DEG2RAD(-60.0);


  RLOG(5, "Adding transition %d %d %d - %d %d %d",
       phi0, rh0, lh0, phi1, rh1, lh1);



  if (!isPartner())
  {
    // Keep axis orientation of polygon object
    moveSet->add(t1, phiBox, 0.0, 0.0, 7, trajPhi + " 2");

    // Re-align axis to x / y = 0
    // moveSet->add(std::make_shared<PositionConstraint>(t1, 0.0, 0.0,
    //                                                 phiBox, trajPhi));

    HTr tRH, tLH;
    HTr_copy(&tRH, &contacts[rh1]);
    HTr_copy(&tLH, &contacts[lh1]);
    Mat3d_rotateSelfAboutXYZAxis(tRH.rot, 1, this->wristAngle);
    Mat3d_rotateSelfAboutXYZAxis(tLH.rot, 1, -this->wristAngle);

    moveSet->add(std::make_shared<PoseConstraint>(t1, &tRH,
                                                  trajPosR, trajOriR));
    moveSet->add(std::make_shared<PoseConstraint>(t1, &tLH,
                                                  trajPosL, trajOriL));
  }
  else
  {
    moveSet->add(std::make_shared<PoseConstraint>(t1, &contacts[rh1],
                                                  trajPosR, trajOriR));
    moveSet->add(std::make_shared<PoseConstraint>(t1, &contacts[lh1],
                                                  trajPosL, trajOriL));
  }


  switch (transition)
  {
    case TransitionModel::RotateObject:
    {
      if (!isPartner())
      {
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 4");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 4");
      }
      break;
    }

    case TransitionModel::RegraspRight:
    {
      // If the contact normal doesn't differ, we slide along the surface
      if (getSlideMode() &&
          Mat3d_diffAngle((double (*)[3])contacts[rh0].rot,
                          (double (*)[3])contacts[rh1].rot)<1.0e-3)
      {
        break;
      }

      // Right hand backwards retract: to be applied to position index 0
      // (x points into box forward direction)
      moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[2], 0.0, 0.0,
                   regraspFlag, trajPosR + " 2");
      moveSet->add(t0 + 2.0*bumpTime*duration, contacts[rh0].org[2], 0.0, 0.0,
                   regraspFlag, trajPosR + " 2");
      if (isPartner())
      {
        moveSet->add(t0 + 0.5*duration, contacts[rh0].org[2] + backDist,
                     0.0, 0.0, regraspFlag, trajPosR + " 2");
      }
      else
      {
        moveSet->add(t0 + 0.3*duration, contacts[rh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosR + " 2");
        moveSet->add(t0 + 0.5*duration, contacts[rh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosR + " 2");
        moveSet->add(t0 + 0.7*duration, contacts[rh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosR + " 2");

        // Open and close fingers
        moveSet->add(t0 + 0.6*duration, fngrOpenAngle, trajFingersR + " 0");
        moveSet->add(t0 + 0.6*duration, fngrOpenAngle, trajFingersR + " 4");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 4");
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 4");
      }

      // Right hand sideways retract
      double invNormal[3], swayDir[3];
      Vec3d_copy(invNormal, contacts[rh0].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration,
                     contacts[rh0].org[0] + bumpDist*invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosR + " 0");
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[1], 0.0, 0.0,
                     regraspFlag, trajPosR + " 1");
        moveSet->add(t0 + 2.0*bumpTime*duration, contacts[rh0].org[1], 0.0, 0.0,
                     regraspFlag, trajPosR + " 1");
        moveSet->add(t0 + 0.3*duration, contacts[rh0].org[0] + swayDir[0],
                     0.0, 0.0, regraspFlag, trajPosR + " 0");
      }
      else
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration,
                     contacts[rh0].org[1] + bumpDist*invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosR+ " 1");
        moveSet->add(t0 + bumpTime*duration, contacts[rh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosR + " 0");
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[rh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosR+ " 0");
        moveSet->add(t0 + 0.3*duration, contacts[rh0].org[1] + swayDir[1],
                     0.0, 0.0, regraspFlag, trajPosR+ " 1");
      }

      // Right hand sideways approach
      Vec3d_copy(invNormal, contacts[rh1].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        moveSet->add(t0 + 0.7*duration, contacts[rh1].org[0] + swayDir[0], 0.0, 0.0,
                     regraspFlag, trajPosR+ " 0");
      }
      else
      {
        moveSet->add(t0 + 0.7*duration, contacts[rh1].org[1] + swayDir[1], 0.0, 0.0,
                     regraspFlag, trajPosR+ " 1");
      }

      break;
    }

    case TransitionModel::RegraspLeft:
    {
      // If the contact normal doesn't differ, we slide along the surface
      if (getSlideMode() &&
          Mat3d_diffAngle((double (*)[3])contacts[lh0].rot,
                          (double (*)[3])contacts[lh1].rot)<1.0e-3)
      {
        break;
      }

      // Left hand backwards retract: to be applied to position index 0
      // (x points into box forward direction)
      moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[2], 0.0, 0.0,
                   regraspFlag, trajPosL+ " 2");
      moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[2], 0.0, 0.0,
                   regraspFlag, trajPosL+ " 2");
      if (isPartner())
      {
        moveSet->add(t0 + 0.5*duration, contacts[lh0].org[2] + backDist,
                     0.0, 0.0, regraspFlag, trajPosL+ " 2");
      }
      else
      {
        moveSet->add(t0 + 0.3*duration, contacts[lh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosL+ " 2");
        moveSet->add(t0 + 0.5*duration, contacts[lh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosL+ " 2");
        moveSet->add(t0 + 0.7*duration, contacts[lh0].org[2] + backDist,
                     0.0, 0.0, 1, trajPosL+ " 2");

        // Open and close fingers
        moveSet->add(t0 + 0.6*duration, fngrOpenAngle, trajFingersL + " 0");
        moveSet->add(t0 + 0.6*duration, fngrOpenAngle, trajFingersL + " 4");
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersL + " 4");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 0");
        moveSet->add(t0 + duration, 0.0, trajFingersR + " 4");
      }

      // Left hand sideways retract
      double invNormal[3];
      double swayDir[3];
      Vec3d_copy(invNormal, contacts[lh0].rot[1]);
      Vec3d_constMul(swayDir, invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration,
                     contacts[lh0].org[0] + bumpDist*invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosL+ " 0");
        // hold box steady during bump
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[1],
                     0.0, 0.0, regraspFlag, trajPosL+ " 1");
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[1],
                     0.0, 0.0, regraspFlag, trajPosL+ " 1");
        moveSet->add(t0 + 0.3*duration, contacts[lh0].org[0] + swayDir[0],
                     0.0, 0.0, regraspFlag, trajPosL+ " 0");
      }
      else
      {
        // bump box
        moveSet->add(t0 + bumpTime*duration,
                     contacts[lh0].org[1] + bumpDist*invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosL+ " 1");
        // hold box steady during bump
        moveSet->add(t0 + bumpTime*duration, contacts[lh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosL+ " 0");
        moveSet->add(t0 + 2 * bumpTime*duration, contacts[lh0].org[0],
                     0.0, 0.0, regraspFlag, trajPosL+ " 0");
        moveSet->add(t0 + 0.3*duration, contacts[lh0].org[1] + swayDir[1],
                     0.0, 0.0, regraspFlag, trajPosL+ " 1");
      }

      // Left hand sideways approach
      Vec3d_copy(invNormal, contacts[lh1].rot[1]);
      Vec3d_constMulSelf(invNormal, -swayDist);

      if (fabs(invNormal[0]) > fabs(invNormal[1]))
      {
        moveSet->add(t0 + 0.7*duration, contacts[lh1].org[0] + invNormal[0],
                     0.0, 0.0, regraspFlag, trajPosL+ " 0");
      }
      else
      {
        moveSet->add(t0 + 0.7*duration, contacts[lh1].org[1] + invNormal[1],
                     0.0, 0.0, regraspFlag, trajPosL+ " 1");
      }

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

/*******************************************************************************
 *
 ******************************************************************************/
TCS_sptr PolygonObjectConstraint::moveHeight(int height, double t1) const
{
  if (trajPosObj.empty())
  {
    return nullptr;
  }

  auto moveSet = std::make_shared<ConstraintSet>();
  moveSet->setClassName("MoveHeight");
  moveSet->add(t1, deltaH*height, trajPosObj + " 2");

  return moveSet;
}

/*******************************************************************************
 *
 ******************************************************************************/
TCS_sptr PolygonObjectConstraint::createActivationSet(bool flag,
                                                      double time) const
{
  auto moveSet = std::make_shared<ActivationSet>();

  moveSet->addActivation(0.0, flag, 0.0, this->trajPosR);
  moveSet->addActivation(0.0, flag, 0.0, this->trajPosL);
  moveSet->addActivation(0.0, flag, 0.0, this->trajOriR);
  moveSet->addActivation(0.0, flag, 0.0, this->trajOriL);

  if (!isPartner())
  {
    moveSet->addActivation(0.0, flag, 0.0, this->trajPhi);
    moveSet->addActivation(0.0, flag, 0.0, this->trajPosObj);
    if (!trajFingersR.empty())
    {
      moveSet->addActivation(0.0, flag, 0.0, this->trajFingersR);
    }
    if (!trajFingersL.empty())
    {
      moveSet->addActivation(0.0, flag, 0.0, this->trajFingersL);
    }
  }

  return moveSet;
}


}   // namespace
