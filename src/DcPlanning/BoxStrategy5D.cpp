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

/*
  Indices:

  phi   Object rotation angle
  ra    Right hand agent
  la    Left hand agent
  rp    Right hand partner
  lp    Left hand partner

*/

#include "BoxStrategy5D.h"
#include "DyadicTrajectorySet.h"

#include <Rcs_macros.h>
#include <Rcs_basicMath.h>
#include <Rcs_VecNd.h>
#include <Rcs_geometry.h>

#include <limits>
#include <cmath>
#include <sstream>

#define COND_RETURN(condition) if ((condition)) { return false;}

namespace Dc
{

BoxStrategy5D::ContactPoint2D::ContactPoint2D() : normalAngle(0.0), frictionAngle(65.0*M_PI/180.0), maxLever(1.0)
{
  this->pt[0] = 0.0;
  this->pt[1] = 0.0;
}

BoxStrategy5D::ContactPoint2D::ContactPoint2D(double x, double y,
                                              double normalAngle_,
                                              double frictionAngle_,
                                              double maxLever_) :
  normalAngle(normalAngle_), frictionAngle(frictionAngle_), maxLever(maxLever_)
{
  this->pt[0] = x;
  this->pt[1] = y;
}

BoxStrategy5D::ContactPoint2D::~ContactPoint2D()
{
}

void BoxStrategy5D::ContactPoint2D::shift(double sx, double sy)
{
  this->pt[0] += sx;
  this->pt[1] += sy;
}

void BoxStrategy5D::ContactPoint2D::scale(double sx, double sy)
{
  this->pt[0] *= sx;
  this->pt[1] *= sy;
}

void BoxStrategy5D::ContactPoint2D::toWorld(double worldPt[2],
                                            const double boxPt[2], double phi)
{
  const double cPhi = cos(phi);
  const double sPhi = sin(phi);
  worldPt[0] = cPhi*boxPt[0] - sPhi*boxPt[1];
  worldPt[1] = sPhi*boxPt[0] + cPhi*boxPt[1];
}

void BoxStrategy5D::ContactPoint2D::toWorld(double worldPt[2],
                                            double phi) const
{
  double cPhi = cos(phi);
  double sPhi = sin(phi);
  worldPt[0] = cPhi*pt[0] - sPhi*pt[1];
  worldPt[1] = sPhi*pt[0] + cPhi*pt[1];
}

void BoxStrategy5D::ContactPoint2D::getContactInWorld(double& x, double& y,
                                                      double phi) const
{
  double worldPt[2];
  toWorld(worldPt, this->pt, phi);
  x = worldPt[0];
  y = worldPt[1];
}

double BoxStrategy5D::ContactPoint2D::getInclinationAngle(double phi) const
{
  return Math_fmodAngle(phi + normalAngle - M_PI);
}

bool BoxStrategy5D::ContactPoint2D::isSupportPoint(double phi) const
{
  double diffAngle = Math_fmodAngle(getInclinationAngle(phi));

  // Add 1 deg to avoid numeric issues
  if (fabs(diffAngle) > this->frictionAngle + RCS_DEG2RAD(1.0))
  {
    return false;
  }

  //  // Permissible lever arm to COM
  //
  //  //TODO: this requires COM being centered in front of robot
  //  //TODO: is this robot base frame or actual world frame (what does driving do to this?)
  //
  //  double I_x[2];
  //  toWorld(I_x, pt, phi);
  //
  //
  //  //TODO: is the mass of the object actually going into this calculation?
  //  if (fabs(I_x[0]) > this->maxLever)
  //  {
  ////    RPAUSE_MSG("I_x[0] = %f   maxLever = %f", I_x[0], maxLever);
  //    return false;
  //  }

  return true;
}

double BoxStrategy5D::ContactPoint2D::getMomentAroundPoint(double phi,
                                                           double mass) const
{
  // Permissible lever arm to COM

  // \todo: this requires COM being centered in front of robot
  // \todo: is this robot base frame or actual world frame (what does driving
  //        do to this?)

  double I_x[2];
  toWorld(I_x, pt, phi);


  //TODO: is the mass of the object actually going into this calculation?
  //TODO: maybe this should be signed
  return fabs(I_x[0]) * mass;
}


bool BoxStrategy5D::ContactPoint2D::isOpposing(const ContactPoint2D& other,
                                               double maxAngle) const
{
  //  RLOG(1, "Normal angle: %.2f   other: %.2f   limit: %.2f",
  //       normalAngle*180.0/M_PI,
  //       other.normalAngle*180.0/M_PI,
  //       maxAngle*180.0/M_PI);

  if (fabs(Math_fmodAngle(normalAngle-other.normalAngle))>=maxAngle)
  {
    return true;
  }

  return false;
}

double BoxStrategy5D::ContactPoint2D::getDistance(const ContactPoint2D& other) const
{
  double dist = sqrt(pow(getX()-other.getX(), 2) + pow(getY()-other.getY(), 2));
  //  if (dist == 0.0)
  //  {
  //  RLOG(1, "(%5.2f, %5.2f) , (%5.2f, %5.2f)",
  //       getX(), getY(), other.getX(), other.getY());
  //  }
  return dist;
}

double BoxStrategy5D::ContactPoint2D::getX() const
{
  return this->pt[0];
}

double BoxStrategy5D::ContactPoint2D::getY() const
{
  return this->pt[1];
}

void BoxStrategy5D::ContactPoint2D::setFrictionAngle(double angle)
{
  this->frictionAngle = angle;
}

double BoxStrategy5D::ContactPoint2D::getFrictionAngle() const
{
  return this->frictionAngle;
}

double BoxStrategy5D::ContactPoint2D::getNormalAngle() const
{
  return this->normalAngle;
}

void BoxStrategy5D::ContactPoint2D::setMaxLever(double distance)
{
  this->maxLever = distance;
}

double BoxStrategy5D::ContactPoint2D::getMaxLever() const
{
  return this->maxLever;
}

void BoxStrategy5D::ContactPoint2D::setX(double value)
{
  this->pt[0] = value;
}

void BoxStrategy5D::ContactPoint2D::setY(double value)
{
  this->pt[1] = value;
}

void BoxStrategy5D::ContactPoint2D::setNormalAngle(double value)
{
  this->normalAngle = value;
}

// \todo: Re-think what should and should not be static, how to organize object
//        model data
unsigned int BoxStrategy5D::numPhiDiscretizations = 4;

/*******************************************************************************
 * Constructor
 ******************************************************************************/
BoxStrategy5D::BoxStrategy5D(ObjectType object_, int numPhiDiscretizations_) :
  ExplorationStrategy(), deltaPhi(2.0*M_PI/numPhiDiscretizations_),
  maxAngularStateChange(0.7*M_PI),
  minimumHandDistance(0.0), maxAngleBetweenHands(M_PI_2), object(object_),
  goalCondition(FullState), startCondition(DoesNotMatter)
{
  setObjectType(object);
  setNumPhiDiscretizations(numPhiDiscretizations_);
}

/*******************************************************************************
 * Destructor has nothing to do currently. We need it nevertheless due to
 * polymorphism.
 ******************************************************************************/
BoxStrategy5D::~BoxStrategy5D()
{
}

/*******************************************************************************
 * Check if the state is allowed, in other words if the box falls. The box
 * center of mass is assumed to be at (0 , 0).
 ******************************************************************************/
bool BoxStrategy5D::checkState(std::vector<int> state) const
{
  if (state.size() != 5)
  {
    RLOG(1, "State dimension is %zd but should be 5", state.size());
    return false;
  }

  return checkState(state[0], state[1], state[2], state[3], state[4], false);
}

/*******************************************************************************
 * Check if the state is allowed, in other words if the box falls. The box
 * center of mass is assumed to be at (0 , 0).
 ******************************************************************************/
bool BoxStrategy5D::checkState(std::vector<int> state, bool testMe) const
{
  if (state.size() != 5)
  {
    RLOG(1, "State dimension is %zd but should be 5", state.size());
    return false;
  }

  return checkState(state[0], state[1], state[2], state[3], state[4], testMe);
}

/*******************************************************************************
 * Check if the state is allowed, in other words if the box falls. The box
 * center of mass is assumed to be at (0 , 0).
 ******************************************************************************/
bool BoxStrategy5D::checkState(int phi_, int ra, int la, int rp, int lp, bool testing) const
{
  // Between the left and the right agent hand, there has to be some distance.
  double handDist = roboContacts[ra].getDistance(roboContacts[la]);
  if (handDist < this->minimumHandDistance)
  {
    RLOG(2, "[%d %d %d] Robot Hands too close: %f", phi_, ra, la, handDist);
    COND_RETURN(!testing);
  }

  // Compute the (absolute) coordinates of the grasp point in the world frame.
  double phi = phi_ * this->deltaPhi;
  double rightX = 0.0, rightY = 0.0, leftX = 0.0, leftY = 0.0;
  roboContacts[ra].getContactInWorld(rightX, rightY, phi);
  roboContacts[la].getContactInWorld(leftX, leftY, phi);

  // The agent's right hand has to stay on the right and the left on the left.
  if (rightX <= leftX)
  {
    RLOG(2, "[%d %d %d]: Agent hands cross: left=%f right=%f", phi_, ra, la, leftX, rightX);
    COND_RETURN(!testing);
  }

  // Right and left agent hand have to be on different sides of the center of
  // mass.
  if (rightX * leftX >= 0.0)
  {
    RLOG(2, "[%d %d %d]: Agent hands on one side of the COM", phi_, ra, la);
    COND_RETURN(!testing);
  }

  // add 1 deg to avoid numeric precision issues
  if (roboContacts[ra].isOpposing(roboContacts[la], maxAngleBetweenHands + RCS_DEG2RAD(1.0)))
  {
    RLOG(2, "[%d %d]: Agent hands are opposing", ra, la);
    COND_RETURN(!testing);
  }

  // Check that on each side at least one hand supports the object
  if ((!roboContacts[ra].isSupportPoint(phi)) && (!roboContacts[la].isSupportPoint(phi)))
  {
    RLOG(2, "Phi: %d [%d %d]: Agent has no support point", phi_, ra, la);
    COND_RETURN(!testing);
  }



  // From here, we check the state for the partner. Since it is on the opposite
  // side of the box, the box rotation angle needs to be mirrored.
  phi = -phi;


  double rightXP = 0.0, rightYP = 0.0, leftXP = 0.0, leftYP = 0.0;
  partnerContacts[rp].getContactInWorld(rightXP, rightYP, phi);
  partnerContacts[lp].getContactInWorld(leftXP, leftYP, phi);

  // Right and left partner hand have to be on different sides of the center of
  // mass.
  if (rightXP * leftXP >= 0.0)
  {
    RLOG(2, "[%d %d %d]: Partner hands on one side of the COM", phi_, rp, lp);
    COND_RETURN(!testing);
  }

  // The partner's right hand has to stay on the right and the left on the left.
  // We allow an overlap of 10cm
  if (rightXP <= leftXP-0.1)
  {
    RLOG(2, "[%d %d %d]: Partner hands cross", phi_, rp, lp);
    COND_RETURN(!testing);
  }

  // At least one of the partner's hands needs to be a supporting hand.
  if ((!partnerContacts[rp].isSupportPoint(phi)) && (!partnerContacts[lp].isSupportPoint(phi)))
  {
    RLOG(2, "Phi: %d [%d %d]: Partner has no support point", phi_, rp, lp);
    return false;
  }

  // Between the left and the right agent hand, there has to be some distance.
  //  handDist = partnerContacts[rp].getDistance(partnerContacts[lp]);
  //  if (handDist < this->minimumHandDistance)
  //  {
  //    RLOG(1, "[%d %d] Partner Hands too close: %f", rp, lp, handDist);
  //    return false;
  //  }


  // // The box COM must be balanced within the supporting polygon of the hands.
  // int nSupportingHands = 0;
  // double poly[4][2], comPrj[2], lBox = 1.0;
  // comPrj[0] = 0.0;
  // comPrj[1] = 0.5*lBox;

  // if (contact[lp].isSupportPoint(phi))
  // {
  //   poly[nSupportingHands][0] = -leftXP;
  //   poly[nSupportingHands][1] = 1.0;
  //   RLOG(5, "LP Contact point [%d,%.1f] is support point: %.3f %.3f",
  //        lp, RCS_RAD2DEG(phi), poly[nSupportingHands][0], poly[nSupportingHands][1]);
  //   nSupportingHands++;
  // }

  // if (contact[rp].isSupportPoint(phi))
  // {
  //   poly[nSupportingHands][0] = -rightXP;
  //   poly[nSupportingHands][1] = 1.0;
  //   RLOG(5, "RP Contact point [%d,%.1f] is support point: %.3f %.3f",
  //        rp, RCS_RAD2DEG(phi), poly[nSupportingHands][0], poly[nSupportingHands][1]);
  //   nSupportingHands++;
  // }

  // if (contact[la].isSupportPoint(-phi))
  // {
  //   poly[nSupportingHands][0] = leftX;
  //   poly[nSupportingHands][1] = 0.0;
  //   RLOG(5, "LA Contact point [%d,%.1f] is support point: %.3f %.3f",
  //        la, RCS_RAD2DEG(-phi), poly[nSupportingHands][0], poly[nSupportingHands][1]);
  //   nSupportingHands++;
  // }

  // if (contact[ra].isSupportPoint(-phi))
  // {
  //   poly[nSupportingHands][0] = rightX;
  //   poly[nSupportingHands][1] = 0.0;
  //   RLOG(5, "RA Contact point [%d,%.1f] is support point: %.3f %.3f",
  //        ra, RCS_RAD2DEG(-phi), poly[nSupportingHands][0], poly[nSupportingHands][1]);
  //   nSupportingHands++;
  // }

  // // if (nSupportingHands < 3)
  // // {
  // //   RLOG(0, "Only %d supporting hands, 3 or more are required", nSupportingHands);
  // //   return false;
  // // }


  // bool polyOK = Math_checkPolygon2D(poly, nSupportingHands);

  // double d = Math_distPointConvexPolygon2D(comPrj, poly, nSupportingHands, NULL, NULL);

  // if (!polyOK)
  // {
  // for (int i=0;i<nSupportingHands; ++i)
  // {
  //   RLOG(0, "Poly[%d] = %f %f", i, poly[i][0], poly[i][1]);
  // }
  // RLOG(0, "d = %.3f", d);

  // RPAUSE();
  // }


  // if (d > 0.1)
  // {
  //   RLOG(1, "%d support points: COM outside support polygon by %.3f m", nSupportingHands, d);
  //   return false;
  // }



  // add 1 deg to avoid numeric precision issues
  // if (contact[rp].isOpposing(contact[lp], maxAngleBetweenHands + RCS_DEG2RAD(1.0)))
  // {
  //   RLOG(1, "[%d %d]: Partner hands are opposing", ra, la);
  //   return false;
  // }

  return true;
}

/*******************************************************************************
 * Check whether the transition is possible. It is possible if
 * - Both hands don't change or
 *
 * It is impossible if
 * - Both hands change
 * - One hand changes while the box is rotated
 ******************************************************************************/
bool BoxStrategy5D::checkTransition(int phiOld, int raOld, int laOld, int rpOld, int lpOld,
                                    int phiNew, int raNew, int laNew, int rpNew, int lpNew) const
{
  int numContactChanges = 0;
  if (laOld!=laNew)
  {
    numContactChanges++;
  }
  if (raOld!=raNew)
  {
    numContactChanges++;
  }
  if (lpOld!=lpNew)
  {
    numContactChanges++;
  }
  if (rpOld!=rpNew)
  {
    numContactChanges++;
  }

  // The hands are at the same position before and after the transition.
  if (numContactChanges==0)
  {
    if (phiOld != phiNew)
    {
      // Any rotation is fine. If the new state is valid or not will be checked
      // in the checkState() function. This is a bit of an approximation, since
      // travelling through different states might lead to invalid intermediate
      // states. This is currently disregarded.
      // \todo: this is not true... transition could pass through invalid
      //        states...
      return true;
    }
    else
    {
      // We disallow the transition if nothing changes.
      return false;
    }
  }

  // If two or more hands are at different positions, the transition is not
  // allowed.
  if (numContactChanges > 1)
  {
    RLOG(2, "%d contact changes, max. 1 is allowed", numContactChanges);
    return false;
  }

  // From here on, we face a change of exactly one grasp hold. For this, we
  // disallow rotating the box.
  if (phiOld != phiNew)
  {
    return false;
  }

  if (roboContacts[laOld].isOpposing(roboContacts[laNew], maxAngularStateChange))
  {
    RLOG(2, "[%d %d]: Left hands are opposing", laOld, laNew);
    return false;
  }

  if (roboContacts[raOld].isOpposing(roboContacts[raNew], maxAngularStateChange))
  {
    RLOG(2, "[%d %d]: Right hands are opposing", raOld, raNew);
    return false;
  }

  if (partnerContacts[lpOld].isOpposing(partnerContacts[lpNew], maxAngularStateChange))
  {
    RLOG(2, "[%d %d]: Left partner hands are opposing", lpOld, lpNew);
    return false;
  }

  if (partnerContacts[rpOld].isOpposing(partnerContacts[rpNew], maxAngularStateChange))
  {
    RLOG(2, "[%d %d]: Right partner hands are opposing", rpOld, rpNew);
    return false;
  }

  double phi = this->deltaPhi*phiNew;
  double rpx = 0.0, rpy=0.0, lpx=0.0, lpy=0.0, rax=0.0, ray=0.0, lax=0.0, lay=0.0;
  double  poly[3][2];
  double comPrj[2], lBox = 1.0;
  comPrj[0] = 0.0;
  comPrj[1] = 0.5*lBox;

  partnerContacts[rpOld].getContactInWorld(rpx, rpy, -phi);
  partnerContacts[lpOld].getContactInWorld(lpx, lpy, -phi);
  rpx = -rpx;
  lpx = -lpx;

  roboContacts[raOld].getContactInWorld(rax, ray, phi);
  roboContacts[laOld].getContactInWorld(lax, lay, phi);

  if (laOld!=laNew)
  {
    if (!roboContacts[raOld].isSupportPoint(phi))
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove left agent hand: right is NOT supporting",
           phiOld, raOld, laOld, rpOld, lpOld);
      return false;
    }
    double moment = roboContacts[raOld].getMomentAroundPoint(phi);
    if (moment > roboContacts[raOld].getMaxLever())
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove left agent hand: moment around right support too large (%4.2f > %4.2f)",
           phiOld, raOld, laOld, rpOld, lpOld, moment, roboContacts[raOld].getMaxLever());
      return false;
    }

    poly[0][0] = rax;
    poly[0][1] = 0.0;
    poly[1][0] = rpx;
    poly[1][1] = lBox;
    poly[2][0] = lpx;
    poly[2][1] = lBox;
  }
  else if (raOld!=raNew)
  {
    if (!roboContacts[laOld].isSupportPoint(phi))
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove right agent hand: left is NOT supporting",
           phiOld, raOld, laOld, rpOld, lpOld);
      return false;
    }
    double moment = roboContacts[laOld].getMomentAroundPoint(phi);
    if (moment > roboContacts[laOld].getMaxLever())
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove right agent hand: moment around left support too large (%4.2f > %4.2f)",
           phiOld, raOld, laOld, rpOld, lpOld, moment, roboContacts[raOld].getMaxLever());
      return false;
    }

    poly[0][0] = lax;
    poly[0][1] = 0.0;
    poly[1][0] = rpx;
    poly[1][1] = lBox;
    poly[2][0] = lpx;
    poly[2][1] = lBox;
  }
  else if (lpOld!=lpNew)
  {
    if (!partnerContacts[rpOld].isSupportPoint(-phi))
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove left partner hand: right is NOT supporting",
           phiOld, raOld, laOld, rpOld, lpOld);
      return false;
    }
    poly[0][0] = lax;
    poly[0][1] = 0.0;
    poly[1][0] = rax;
    poly[1][1] = 0.0;
    poly[2][0] = rpx;
    poly[2][1] = lBox;
  }
  else  if (rpOld!=rpNew)
  {
    if (!partnerContacts[lpOld].isSupportPoint(-phi))
    {
      RLOG(2, "[%d %d %d %d %d]: Cannot remove right partner hand: left is NOT supporting",
           phiOld, raOld, laOld, rpOld, lpOld);
      return false;
    }
    //TODO: check if this is needed
    //    if (lpNew==rpNew)
    //    {
    //      return false;
    //    }
    poly[0][0] = lax;
    poly[0][1] = 0.0;
    poly[1][0] = rax;
    poly[1][1] = 0.0;
    poly[2][0] = lpx;
    poly[2][1] = lBox;
  }

  bool polyOK = Math_checkPolygon2D(poly, 3);

  // If it's not Ok, the vertices are not ordered counter-clock-wise. In this
  // case, we flip the 2nd and 3rd vertices. If the polygon is not ill-formed,
  // this should now be fine.
  if (!polyOK)
  {
    double tmp1 = poly[1][0];
    double tmp2 = poly[1][1];
    poly[1][0] = poly[2][0];
    poly[1][1] = poly[2][1];
    poly[2][0] = tmp1;
    poly[2][1] = tmp2;
  }

  // We do a second check here. If it fails, the polygon is ill-formed (e.g.
  // collapsed to a line), and we just consider this as a failure.
  polyOK = Math_checkPolygon2D(poly, 3);
  if (!polyOK)
  {
    REXEC(2)
    {
      RLOG(0, "Polygon malformed:");
      for (int i=0; i<3; ++i)
      {
        RLOG(0, "Poly[%d] = %f %f", i, poly[i][0], poly[i][1]);
      }

      return false;
    }
  }

  double d = Math_distPointPolygon2D(comPrj, poly, 3, NULL, NULL);

  // COM can lie a bit outside of support polygon as hand width has not been considered
  double handWidthHalf = 0.06;

  if (d <= handWidthHalf + 0.000001) // used epsilon instead of zero to prevent numerical precision problems
  {
    return true;
  }
  else
  {
    RLOG(2, "[%d %d %d %d %d] COM outside support polygon by %.10f m", phiNew, raNew, laNew, rpNew, lpNew, d);

    //    RLOG(2, "Ra %6.2f, La %6.2f,   Rp %6.2f, Lp %6.2f", rax, lax, rpx, lpx );
    //    RLOG(2, "     Robot:    %6.2f, %6.2f     %6.2f, %6.2f",
    // roboContacts[raOld].getX(), roboContacts[raOld].getY(), roboContacts[laOld].getX(), roboContacts[laOld].getY());
    //    RLOG(2, "     Partner:  %6.2f, %6.2f     %6.2f, %6.2f",
    // partnerContacts[raOld].getX(), partnerContacts[raOld].getY(),
    //partnerContacts[laOld].getX(), partnerContacts[laOld].getY());
  }

  return false;
}

/*******************************************************************************
 * Returns the heuristic based on the goal state. A rotation of one state value
 * corresponds to a cost of 1.
 ******************************************************************************/
double BoxStrategy5D::heuristicCost(const std::vector<int>& value) const
{
  double minH = std::numeric_limits<double>::max();

  for (size_t i = 0; i < goalStates.size(); i++)
  {
    double h = deltaPhi*std::abs(value[0] - goalStates[i][0]);
    if (h < minH)
    {
      minH = h;
    }
  }

  return minH;
}

/*******************************************************************************
 * Returns the heuristic based on the goal state. A rotation of one state value
 * corresponds to a cost of 1.
 ******************************************************************************/
double BoxStrategy5D::transitionCost(const std::vector<int>& from,
                                     const std::vector<int>& to) const
{
  const double gCostRotateFlat = 0.0001;
  const double gCostRotatePerRad = 1.0;
  const double gCostRegrasp = 1.0;
  //TODO: this would be nice for finding the path not through constraint but cost...
  const double gCostLever = 0.0;

  double transitionCost = 0.0;

  if (from[0] != to[0])
  {
    transitionCost += gCostRotateFlat + gCostRotatePerRad*deltaPhi*std::abs(from[0]-to[0]);
  }
  else
  {
    transitionCost += gCostRegrasp;

    int raOld = from[1];
    int raNew = to[1];
    int laOld = from[2];
    int laNew = to[2];
    int phi = from[0];

    double moment = 0.0;

    if (laOld != laNew)
    {
      moment = roboContacts[raOld].getMomentAroundPoint(phi);
    }
    else if (raOld != raNew)
    {
      moment = roboContacts[laOld].getMomentAroundPoint(phi);
    }
    transitionCost += (moment * gCostLever);
  }

  return transitionCost;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setNumPhiDiscretizations(unsigned int numPhiDiscretizations_in)
{
  if (numPhiDiscretizations_in <= 0)
  {
    RLOG(1, "Can't set discretization to negative value (%d)",
         numPhiDiscretizations_in);
    return;
  }
  this->numPhiDiscretizations = numPhiDiscretizations_in;
  this->deltaPhi = 2.0*M_PI / this->numPhiDiscretizations;
}

/*******************************************************************************
 *
 ******************************************************************************/
int BoxStrategy5D::getNumPhiDiscretizations() const
{
  return this->numPhiDiscretizations;
}

/*******************************************************************************
 *
 ******************************************************************************/
double BoxStrategy5D::getDeltaPhi() const
{
  return this->deltaPhi;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setFrictionAngle(double angle, int contactIdx)
{
  if (contactIdx == -1)
  {
    for (size_t i = 0; i < roboContacts.size(); ++i)
    {
      roboContacts[i].setFrictionAngle(angle);
    }
  }
  else
  {
    roboContacts[contactIdx].setFrictionAngle(angle);
  }
}

/*******************************************************************************
*
******************************************************************************/
double BoxStrategy5D::getFrictionAngle(size_t contactIdx) const
{
  return roboContacts[contactIdx].getFrictionAngle();
}

/*******************************************************************************
*
******************************************************************************/
std::vector<double> BoxStrategy5D::getFrictionAngles() const
{
  std::vector<double> angles;

  for (size_t i = 0; i < roboContacts.size(); ++i)
  {
    angles.push_back(roboContacts[i].getFrictionAngle());
  }

  return angles;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setMaxLever(double lever, int contactIdx)
{
  if (contactIdx == -1)
  {
    for (size_t i = 0; i < roboContacts.size(); ++i)
    {
      roboContacts[i].setMaxLever(lever);
    }
  }
  else
  {
    roboContacts[contactIdx].setMaxLever(lever);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setMaxAngularStateChange(double angle)
{
  this->maxAngularStateChange = angle;
}

/*******************************************************************************
 *
 ******************************************************************************/
double BoxStrategy5D::getMaxAngularStateChange() const
{
  return this->maxAngularStateChange;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool BoxStrategy5D::goalReached(const std::vector<int>& state) const
{
  RCHECK(!goalStates.empty());

  switch (this->goalCondition)
  {
    case FullState:
      for (size_t i = 0; i < goalStates.size(); i++)
      {
        if (state==goalStates[i])
        {
          return true;
        }
      }
      break;

    case RotationOnly:
      for (size_t i = 0; i < goalStates.size(); i++)
      {
        if (state[0]==goalStates[i][0])
        {
          return true;
        }
      }
      break;

    case RotationWithStableHold:
      for (size_t i = 0; i < goalStates.size(); i++)
      {
        if (state[0]==goalStates[i][0])
        {
          double phi = state[0] * this->getDeltaPhi();
          RLOG(5, "Angle: %d (%3.1f), RH: %d (%s), LH %d (%s)",
               state[0], RCS_RAD2DEG(phi), state[1],
               roboContacts[state[1]].isSupportPoint(state[0]) ? "support": "no",
               state[2],
               roboContacts[state[2]].isSupportPoint(state[0]) ? "support": "no");
          if ((roboContacts[state[1]].isSupportPoint(phi))
              && (roboContacts[state[2]].isSupportPoint(phi)))
          {
            return true;
          }
        }
      }
      break;

    case RotationWithMaxStableHold:
      for (size_t i = 0; i < goalStates.size(); i++)
      {
        if (state[0] ==goalStates[i][0])
        {
          double phi = state[0] * this->getDeltaPhi();
          RLOG(5, "Angle: %d (%3.1f), RH: %d (%s), LH %d (%s)",
               state[0], RCS_RAD2DEG(phi), state[1],
               roboContacts[state[1]].isSupportPoint(state[0]) ? "support": "no",
               state[2],
               roboContacts[state[2]].isSupportPoint(state[0]) ? "support": "no");

          //          RLOG(0, "Check: %4d %2d %2d %2d %2d", state->getValue(0), state->getValue(1), state->getValue(2), state->getValue(3), state->getValue(4));
          if ((state[1] == getSupportPoint(state[0], false, true))
              && (state[2] == getSupportPoint(state[0], true, true))
              && (state[3] == getSupportPoint(state[0], false, false))
              && (state[4] == getSupportPoint(state[0], true, false)))
          {
            return true;
          }
        }
      }
      break;

    default:
      RLOG(1, "Unknonw goal condition: %d", this->goalCondition);
      return true;
  }

  return false;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setMinimumHandDistance(double distance)
{
  this->minimumHandDistance = distance;
}

/*******************************************************************************
 *
 ******************************************************************************/
double BoxStrategy5D::getMinimumHandDistance() const
{
  return this->minimumHandDistance;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setMaxAngleBetweenHands(double angle)
{
  this->maxAngleBetweenHands = angle;
}

/*******************************************************************************
*
******************************************************************************/
double BoxStrategy5D::getMaxAngleBetweenHands() const
{
  return this->maxAngleBetweenHands;
}

/*******************************************************************************
 *
 ******************************************************************************/
int BoxStrategy5D::getPhiFromAngle(double angle) const
{
  return lround(angle/this->deltaPhi);
}

/*******************************************************************************
 *
 ******************************************************************************/
double BoxStrategy5D::getAngleFromPhi(int phi) const
{
  return phi * this->deltaPhi;
}

/*******************************************************************************
 *
 ******************************************************************************/
BoxStrategy5D::ObjectType BoxStrategy5D::getObjectType() const
{
  return this->object;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool BoxStrategy5D::setObjectType(BoxStrategy5D::ObjectType oType)
{

  switch (oType)
  {
    case Box:
    {
      RLOG(5, "BoxStrategy5D initialized with box shape");
      this->roboContacts = getBoxContacts(0.63, 0.36);
      this->setFrictionAngle(RCS_DEG2RAD(15.0));
      break;
    }

    case LShape:
    {
      RLOG(5, "BoxStrategy5D initialized with L-shape");
      this->roboContacts = getLShapeContacts(0.5, 0.5);
      break;
    }

    case Cylinder:
    {
      RLOG(5, "BoxStrategy5D initialized with cylinder shape");
      this->roboContacts = getCylinderContacts(0.3);
      break;
    }

    default:
      RLOG(5, "Unknown object type: %d - leaving old one", oType);
      return false;
  }

  this->object = oType;
  this->partnerContacts = this->roboContacts;
  mirrorContacts(this->partnerContacts);


  REXEC(5)
  {
    for (size_t i=0; i<roboContacts.size(); ++i)
    {
      RMSG("Robot contact   %2zd: %+f %+f   %+.2f", i,
           roboContacts[i].getX(),
           roboContacts[i].getY(),
           RCS_RAD2DEG(roboContacts[i].getNormalAngle()));
      RMSG("Partner contact %2zd: %+f %+f   %+.2f", i,
           partnerContacts[i].getX(),
           partnerContacts[i].getY(),
           RCS_RAD2DEG(partnerContacts[i].getNormalAngle()));
    }
  }

  return true;
}

/*******************************************************************************
 *
 ******************************************************************************/
BoxStrategy5D::GoalCondition BoxStrategy5D::getGoalCondition() const
{
  return this->goalCondition;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setGoalCondition(BoxStrategy5D::GoalCondition newCondition)
{
  this->goalCondition = newCondition;
}

/*******************************************************************************
 *
 ******************************************************************************/
void BoxStrategy5D::setStartCondition(BoxStrategy5D::StartCondition newCondition)
{
  this->startCondition = newCondition;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<HTr> BoxStrategy5D::getRoboContacts3D(std::vector<ContactPoint2D> roboContacts)
{
  std::vector<HTr> contacts;
  HTr handTransform;
  double x[6];

  for (size_t i = 0; i<roboContacts.size(); ++i)
  {
    VecNd_set6(x, roboContacts[i].getX(), roboContacts[i].getY(), 0.0, 0.0, 0.0,
               Math_fmodAngle(roboContacts[i].getNormalAngle() - M_PI));
    HTr_from6DVector(&handTransform, x);
    contacts.push_back(handTransform);
  }

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<HTr> BoxStrategy5D::getRoboContacts3D() const
{
  std::vector<HTr> contacts;
  HTr handTransform;
  double x[6];

  for (size_t i=0; i<roboContacts.size(); ++i)
  {
    VecNd_set6(x, roboContacts[i].getX(), roboContacts[i].getY(), 0.0, 0.0, 0.0,
               Math_fmodAngle(roboContacts[i].getNormalAngle()-M_PI));
    HTr_from6DVector(&handTransform, x);
    contacts.push_back(handTransform);
  }

  return contacts;
}

/*******************************************************************************
*
******************************************************************************/
std::vector<HTr> BoxStrategy5D::getPartnerContacts3D(std::vector<ContactPoint2D> boxContacts)
{
  std::vector<HTr> contact;
  HTr handTransform;
  double x[6];

  auto partnerContacts = boxContacts;
  mirrorContacts(partnerContacts);

  for (size_t i = 0; i<partnerContacts.size(); ++i)
  {
    VecNd_set6(x, partnerContacts[i].getX(), partnerContacts[i].getY(), 0.0, 0.0, 0.0,
               Math_fmodAngle(partnerContacts[i].getNormalAngle() - M_PI));
    HTr_from6DVector(&handTransform, x);
    contact.push_back(handTransform);
  }

  return contact;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<HTr> BoxStrategy5D::getPartnerContacts3D() const
{
  std::vector<HTr> contact;
  HTr handTransform;
  double x[6];

  for (size_t i=0; i<partnerContacts.size(); ++i)
  {
    VecNd_set6(x, partnerContacts[i].getX(), partnerContacts[i].getY(), 0.0, 0.0, 0.0,
               Math_fmodAngle(partnerContacts[i].getNormalAngle()-M_PI));
    HTr_from6DVector(&handTransform, x);
    contact.push_back(handTransform);
  }

  return contact;
}

/*******************************************************************************
 * Contact points for cylinder:
 * (view from the robot side)
 *
 *      y
 *      ^
 *      |
 *      ---> x
 *
 *
 *                5       4 (78.75 deg)
 *           ____________________
 *       6  /                    \ 3   (56.25 deg)
 *         /                      \
 *      7 /                        \  2   (33.75 deg)
 *       /                          \
 *    8 |            e1              | 1   (11.25 deg)
 *      |                            |
 *      |e2                        e0|    (-11.25 deg)
 *    9 |            e3              | 0
 *       \                          /
 *    10  \                        / 15
 *         \                      /
 *       11 \____________________/ 14
 *                12     13
 *
 ******************************************************************************/
std::vector<BoxStrategy5D::ContactPoint2D>
BoxStrategy5D::getCylinderContacts(double radius)
{
  std::vector<BoxStrategy5D::ContactPoint2D> contact(16);

  const double frictionAngle = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;
  const double sliceAngle = 2.0*M_PI/contact.size();
  double phi0 = 0.0;

  if (contact.size() % 2 == 0)
  {
    phi0 = -0.5*sliceAngle;
  }

  for (size_t i=0; i<contact.size(); ++i)
  {
    double phiNormal = phi0 + i*sliceAngle - M_PI_2;
    double x = radius*cos(phiNormal+M_PI_2);
    double y = radius*sin(phiNormal+M_PI_2);
    contact[i] = ContactPoint2D(x, y, phiNormal, frictionAngle, maxLever);
  }

  return contact;
}

/*******************************************************************************
 * Contact points for L-shape:
 * (view from the robot side)
 *
 *      y
 *      ^
 *      |
 *      ---> x
 *
 *         7
 *       ______
 *      |<.25->|
 *    8 |      | 6
 *      |      |
 *      |      | 5
 *    9 |      |
 *      |      | 4
 *      |      |
 *   10 |      |    3     2      1
 *      |      |_____________________
 *      |                            |
 *   11 |<--------- 1 -------------->| 0
 *      |____________________________|
 *         12     13     14      15
 *
 * The COM is at about 0.34 units
 *
 ******************************************************************************/
std::vector<BoxStrategy5D::ContactPoint2D>
BoxStrategy5D::getLShapeContacts(double lengthX, double lengthY)
{
  std::vector<BoxStrategy5D::ContactPoint2D> contact(16);

  const double frictionAngle = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;

  contact[0]  = ContactPoint2D(1.0, 0.125, -M_PI_2, frictionAngle, maxLever);

  contact[1]  = ContactPoint2D(0.8,  0.25, 0.0, frictionAngle, maxLever);
  contact[2]  = ContactPoint2D(0.6,  0.25, 0.0, frictionAngle, maxLever);
  contact[3]  = ContactPoint2D(0.4,  0.25, 0.0, frictionAngle, maxLever);

  contact[4]  = ContactPoint2D(0.25, 0.4, -M_PI_2, frictionAngle, maxLever);
  contact[5]  = ContactPoint2D(0.25, 0.6, -M_PI_2, frictionAngle, maxLever);
  contact[6]  = ContactPoint2D(0.25, 0.8, -M_PI_2, frictionAngle, maxLever);

  contact[7]  = ContactPoint2D(0.125,  1.0, 0.0, frictionAngle, maxLever);


  contact[8]  = ContactPoint2D(0.0, 0.8, M_PI_2, frictionAngle, maxLever);
  contact[9]  = ContactPoint2D(0.0, 0.6, M_PI_2, frictionAngle, maxLever);
  contact[10] = ContactPoint2D(0.0, 0.4, M_PI_2, frictionAngle, maxLever);
  contact[11] = ContactPoint2D(0.0, 0.2, M_PI_2, frictionAngle, maxLever);

  contact[12] = ContactPoint2D(0.2, 0.0, M_PI, frictionAngle, maxLever);
  contact[13] = ContactPoint2D(0.4, 0.0, M_PI, frictionAngle, maxLever);
  contact[14] = ContactPoint2D(0.6, 0.0, M_PI, frictionAngle, maxLever);
  contact[15] = ContactPoint2D(0.8, 0.0, M_PI, frictionAngle, maxLever);

  for (size_t i=0; i<contact.size(); ++i)
  {
    contact[i].shift(-0.34, -0.34);
    contact[i].scale(lengthX, lengthY);
  }

  return contact;
}

/*******************************************************************************
 * Contact points for box:
 * (view from the robot side)
 *
 *      y
 *      ^
 *      |
 *      ---> x
 *
 *
 *          6    5    4    3    2
 *       ____________________________
 *    7 |            e1              | 1
 *      |                            |
 *    8 |e2                        e0| 0
 *      |            e3              |
 *    9 |____________________________| 15
 *          10   11   12   13   14
 *
 ******************************************************************************/
std::vector<BoxStrategy5D::ContactPoint2D>
BoxStrategy5D::getBoxContacts(double lengthX, double lengthY)
{
  std::vector<BoxStrategy5D::ContactPoint2D> contact(16);

  const double frictionAngle = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;

  contact[15] = ContactPoint2D(0.5, -0.33, -M_PI_2, frictionAngle, maxLever);
  contact[0]  = ContactPoint2D(0.5,  0.0, -M_PI_2, frictionAngle, maxLever);
  contact[1]  = ContactPoint2D(0.5,  0.33, -M_PI_2, frictionAngle, maxLever);

  contact[2]  = ContactPoint2D(0.33, 0.5, 0.0, frictionAngle, maxLever);
  contact[3]  = ContactPoint2D(0.165, 0.5, 0.0, frictionAngle, maxLever);
  contact[4]  = ContactPoint2D(0.0, 0.5, 0.0, frictionAngle, maxLever);
  contact[5]  = ContactPoint2D(-0.165, 0.5, 0.0, frictionAngle, maxLever);
  contact[6]  = ContactPoint2D(-0.33, 0.5, 0.0, frictionAngle, maxLever);

  contact[7]  = ContactPoint2D(-0.5,  0.33, M_PI_2, frictionAngle, maxLever);
  contact[8]  = ContactPoint2D(-0.5,  0.0, M_PI_2, frictionAngle, maxLever);
  contact[9]  = ContactPoint2D(-0.5, -0.33, M_PI_2, frictionAngle, maxLever);

  contact[10] = ContactPoint2D(-0.33, -0.5, M_PI, frictionAngle, maxLever);
  contact[11] = ContactPoint2D(-0.165, -0.5, M_PI, frictionAngle, maxLever);
  contact[12] = ContactPoint2D(0.0, -0.5, M_PI, frictionAngle, maxLever);
  contact[13] = ContactPoint2D(0.165, -0.5, M_PI, frictionAngle, maxLever);
  contact[14] = ContactPoint2D(0.33, -0.5, M_PI, frictionAngle, maxLever);

  for (size_t i=0; i<contact.size(); ++i)
  {
    contact[i].scale(lengthX, lengthY);
  }

  return contact;
}

void BoxStrategy5D::mirrorContacts(std::vector<BoxStrategy5D::ContactPoint2D>& contacts)
{
  for (size_t i=0; i<contacts.size(); ++i)
  {
    contacts[i].setX(-contacts[i].getX());
    contacts[i].setNormalAngle(Math_fmodAngle(-contacts[i].getNormalAngle()));
  }
}

int BoxStrategy5D::getLeftMostSupportPoint(int phi) const
{
  int leftMostSupport = -1;
  double x = 0.0, y = 0.0, x_min = 0.0, y_min = 0.0;

  for (size_t i = 0; i < roboContacts.size(); ++i)
  {
    if (roboContacts[i].isSupportPoint(phi * deltaPhi))
    {
      roboContacts[i].getContactInWorld(x, y, phi * deltaPhi);

      if (leftMostSupport == -1)
      {
        leftMostSupport = (int) i;
        x_min = x;
        y_min = y;
      }
      else
      {
        if ((x < x_min) || ((x == x_min) && (y < y_min)))
        {
          leftMostSupport = (int) i;
          x_min = x;
          y_min = y;
        }
      }
    }
  }
  return leftMostSupport;
}

int BoxStrategy5D::getRightMostSupportPoint(int phi) const
{
  int rightMostSupport = -1;
  double x = 0.0, y = 0.0, x_max = 0.0, y_max = 0.0;

  for (size_t i = 0; i < roboContacts.size(); ++i)
  {
    if (roboContacts[i].isSupportPoint(phi*deltaPhi))
    {
      roboContacts[i].getContactInWorld(x, y, phi*deltaPhi);

      if (rightMostSupport == -1)
      {
        rightMostSupport = (int) i;
        x_max = x;
        y_max = y;
      }
      else
      {
        if ((x>x_max) || ((x==x_max) && (y<y_max)))
        {
          rightMostSupport = (int) i;
          x_max = x;
          y_max = y;
        }
      }
    }
  }

  return rightMostSupport;
}

std::vector<int> BoxStrategy5D::getStableHandSupports(int phi) const
{
  std::vector<int> result;
  result.push_back(phi);
  result.push_back(getSupportPoint(phi, false, true));
  result.push_back(getSupportPoint(phi, true, true));
  result.push_back(getSupportPoint(-phi, false, false));
  result.push_back(getSupportPoint(-phi, true, false));

  return result;
}

int BoxStrategy5D::getSupportPoint(int phi_, bool left, bool robot, bool print) const
{
  int support = -1;
  double x = 0.0, y = 0.0, x_comp = 0.0, y_comp = 0.0;
  int phi;

  const std::vector<ContactPoint2D>* contacts;
  if (robot)
  {
    contacts = &roboContacts;
    phi = phi_;
  }
  else
  {
    contacts = &partnerContacts;
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

/*******************************************************************************
 * Explore around the current state
 ******************************************************************************/
std::vector<std::vector<int>> BoxStrategy5D::explore(const std::vector<int>& currentState) const
{
  std::vector<std::vector<int>> nextStates;

  // Current state
  const int phiCurrent = currentState[0];
  const int raCurrent  = currentState[1];
  const int laCurrent  = currentState[2];
  const int rpCurrent  = currentState[3];
  const int lpCurrent  = currentState[4];


  // 1. Try to turn the box deltaPhi counter-clockwise by adding deltaPhi
  //    Check if the state is allowed and if the transition is possible
  if ((startCondition==DoesNotMatter) || (startCondition==RobotHandsAndBox) || (startCondition==HumanHandsAndBox))
  {
    int i_min = (int)floor(M_PI / deltaPhi);

    for (int i = -i_min; i < i_min; i++)
    {
      // Skip i=0 because it ends up at the same state
      if (i == 0)
      {
        continue;
      }

      if (checkState(phiCurrent + i, raCurrent, laCurrent, rpCurrent, lpCurrent) &&
          checkTransition(phiCurrent, raCurrent, laCurrent, rpCurrent, lpCurrent,
                          phiCurrent + i, raCurrent, laCurrent, rpCurrent, lpCurrent))
      {
        std::vector<int> newStateValues;
        newStateValues.push_back(phiCurrent + i);
        newStateValues.push_back(raCurrent);
        newStateValues.push_back(laCurrent);
        newStateValues.push_back(rpCurrent);
        newStateValues.push_back(lpCurrent);
        nextStates.push_back(newStateValues);
      }
    }
  }   // if (startCondition...)

  // 2. Try to move the right agent hand: Loop loop through all positions
  if ((startCondition==DoesNotMatter) || (startCondition==RobotHands) || (startCondition==RobotHandsAndBox))
  {
    for (int newRight = 0; newRight < (int)roboContacts.size(); ++newRight)
    {
      // Skip the current value
      if (newRight == raCurrent)
      {
        continue;
      }

      // Check if the state is allowed and if the transition is possible
      if (checkState(phiCurrent, newRight, laCurrent, rpCurrent, lpCurrent)
          && checkTransition(phiCurrent, raCurrent, laCurrent, rpCurrent, lpCurrent,
                             phiCurrent, newRight, laCurrent, rpCurrent, lpCurrent))
      {
        std::vector<int> newStateValues;
        newStateValues.push_back(phiCurrent);
        newStateValues.push_back(newRight);
        newStateValues.push_back(laCurrent);
        newStateValues.push_back(rpCurrent);
        newStateValues.push_back(lpCurrent);
        nextStates.push_back(newStateValues);
      }
    }


    // 3. Try to move the left agent hand: Loop loop through all positions
    for (int newLeft = 0; newLeft < (int)roboContacts.size(); ++newLeft)
    {
      // Skip the current value
      if (newLeft == laCurrent)
      {
        continue;
      }

      // Check if the state is allowed and if the transition is possible
      if (checkState(phiCurrent, raCurrent, newLeft, rpCurrent, lpCurrent) &&
          checkTransition(phiCurrent, raCurrent, laCurrent, rpCurrent, lpCurrent,
                          phiCurrent, raCurrent, newLeft, rpCurrent, lpCurrent))
      {
        std::vector<int> newStateValues;
        newStateValues.push_back(phiCurrent);
        newStateValues.push_back(raCurrent);
        newStateValues.push_back(newLeft);
        newStateValues.push_back(rpCurrent);
        newStateValues.push_back(lpCurrent);
        nextStates.push_back(newStateValues);
      }
    }
  }   // if (startCondition...)


  // 4. Try to move the right partner hand: Loop loop through all positions
  if ((startCondition==DoesNotMatter) || (startCondition==HumanHands) || (startCondition==HumanHandsAndBox))
  {
    for (int newRight = 0; newRight < (int)roboContacts.size(); ++newRight)
    {
      // Skip the current value
      if (newRight == rpCurrent)
      {
        continue;
      }

      // Check if the state is allowed and if the transition is possible
      if (checkState(phiCurrent, raCurrent, laCurrent, newRight, lpCurrent)
          && checkTransition(phiCurrent, raCurrent, laCurrent, rpCurrent, lpCurrent,
                             phiCurrent, raCurrent, laCurrent, newRight, lpCurrent))
      {
        std::vector<int> newStateValues;
        newStateValues.push_back(phiCurrent);
        newStateValues.push_back(raCurrent);
        newStateValues.push_back(laCurrent);
        newStateValues.push_back(newRight);
        newStateValues.push_back(lpCurrent);
        nextStates.push_back(newStateValues);
      }
    }


    // 5. Try to move the left partner hand: Loop loop through all positions
    for (int newLeft = 0; newLeft < (int)roboContacts.size(); ++newLeft)
    {
      // Skip the current value
      if (newLeft == lpCurrent)
      {
        continue;
      }

      // Check if the state is allowed and if the transition is possible
      if (checkState(phiCurrent, raCurrent, laCurrent, rpCurrent, newLeft) &&
          checkTransition(phiCurrent, raCurrent, laCurrent, rpCurrent, lpCurrent,
                          phiCurrent, raCurrent, laCurrent, rpCurrent, newLeft))
      {
        std::vector<int> newStateValues;
        newStateValues.push_back(phiCurrent);
        newStateValues.push_back(raCurrent);
        newStateValues.push_back(laCurrent);
        newStateValues.push_back(rpCurrent);
        newStateValues.push_back(newLeft);
        nextStates.push_back(newStateValues);
      }
    }
  }   // if (startCondition...)


  startCondition = DoesNotMatter;

  return nextStates;
}

void BoxStrategy5D::setGoal(const std::vector<int>& goal)
{
  goalStates.clear();
  goalStates.push_back(goal);
}

}  // namespace Dc
