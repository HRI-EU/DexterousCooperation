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

#include "PolygonContactPoint2D.h"

#include <Rcs_macros.h>
#include <Rcs_basicMath.h>
#include <Rcs_VecNd.h>
#include <Rcs_geometry.h>

#include <limits>
#include <cmath>
#include <sstream>

namespace Rcs
{

PolygonContactPoint2D::PolygonContactPoint2D() : normalAngle(0.0), frictionAngle(65.0*M_PI/180.0), maxLever(1.0)
{
  this->pt[0] = 0.0;
  this->pt[1] = 0.0;
}

PolygonContactPoint2D::PolygonContactPoint2D(double x, double y,
                                             double normalAngle_,
                                             double frictionAngle_,
                                             double maxLever_) :
  normalAngle(normalAngle_), frictionAngle(frictionAngle_), maxLever(maxLever_)
{
  this->pt[0] = x;
  this->pt[1] = y;
}

PolygonContactPoint2D::~PolygonContactPoint2D()
{
}

void PolygonContactPoint2D::shift(double sx, double sy)
{
  this->pt[0] += sx;
  this->pt[1] += sy;
}

void PolygonContactPoint2D::scale(double sx, double sy)
{
  this->pt[0] *= sx;
  this->pt[1] *= sy;
}

void PolygonContactPoint2D::toWorld(double worldPt[2],
                                    const double boxPt[2], double phi)
{
  const double cPhi = cos(phi);
  const double sPhi = sin(phi);
  worldPt[0] = cPhi*boxPt[0] - sPhi*boxPt[1];
  worldPt[1] = sPhi*boxPt[0] + cPhi*boxPt[1];
}

void PolygonContactPoint2D::toWorld(double worldPt[2],
                                    double phi) const
{
  double cPhi = cos(phi);
  double sPhi = sin(phi);
  worldPt[0] = cPhi*pt[0] - sPhi*pt[1];
  worldPt[1] = sPhi*pt[0] + cPhi*pt[1];
}

void PolygonContactPoint2D::getContactInWorld(double& x, double& y,
                                              double phi) const
{
  double worldPt[2];
  toWorld(worldPt, this->pt, phi);
  x = worldPt[0];
  y = worldPt[1];
}

double PolygonContactPoint2D::getInclinationAngle(double phi) const
{
  return Math_fmodAngle(phi + normalAngle - M_PI);
}

bool PolygonContactPoint2D::isSupportPoint(double phi) const
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

bool PolygonContactPoint2D::isOpposing(const PolygonContactPoint2D& other,
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

double PolygonContactPoint2D::getDistance(const PolygonContactPoint2D& other) const
{
  double dist = sqrt(pow(getX()-other.getX(), 2) + pow(getY()-other.getY(), 2));
  //  if (dist == 0.0)
  //  {
  //  RLOG(1, "(%5.2f, %5.2f) , (%5.2f, %5.2f)",
  //       getX(), getY(), other.getX(), other.getY());
  //  }
  return dist;
}

double PolygonContactPoint2D::getX() const
{
  return this->pt[0];
}

double PolygonContactPoint2D::getY() const
{
  return this->pt[1];
}

void PolygonContactPoint2D::setFrictionAngle(double angle)
{
  this->frictionAngle = angle;
}

double PolygonContactPoint2D::getFrictionAngle() const
{
  return this->frictionAngle;
}

double PolygonContactPoint2D::getNormalAngle() const
{
  return this->normalAngle;
}

void PolygonContactPoint2D::setMaxLever(double distance)
{
  this->maxLever = distance;
}

double PolygonContactPoint2D::getMaxLever() const
{
  return this->maxLever;
}

void PolygonContactPoint2D::setX(double value)
{
  this->pt[0] = value;
}

void PolygonContactPoint2D::setY(double value)
{
  this->pt[1] = value;
}

void PolygonContactPoint2D::setNormalAngle(double value)
{
  this->normalAngle = value;
}


}  // namespace Rcs
