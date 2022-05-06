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

#ifndef RCS_POLYGONCONTACTPOINT2D_H
#define RCS_POLYGONCONTACTPOINT2D_H

namespace Rcs
{

/*! \brief Class to represent 2d polygon points. The points also contain
 *         information on friction cone, normal direction and a lever arm
 *         property. The latter one is currently not used.
 */
class PolygonContactPoint2D
{
public:

  /*! \brief Constructs an empty point at [0 0] with normal angle 0, friction
   *         angle 65 degrees and maxLever of 1.
   */
  PolygonContactPoint2D();

  /*! \brief Constructs a point at [x y] with normal angle, friction
   *         angle and maxLever passed in the constructor arguments.
   */
  PolygonContactPoint2D(double x,  double y, double normalAngle,
                        double frictionAngle, double maxLever);

  /*! \brief Destructor is empty, we implement it for proper polymorphism so
   *         that other classes can derieve from it.
   */
  virtual ~PolygonContactPoint2D();

  /*! \brief Adds [sx sy] to the point coordinates.
   */
  void shift(double sx, double sy);

  /*! \brief Element-wise multiplication of point coordinates with [sx sy].
   */
  void scale(double sx, double sy);

  /*! \brief Assuming the point is rotated by angle phi, this function
   *         computes the point coordinates in the base frame.
   */
  static void toWorld(double worldPt[2], const double boxPt[2], double phi);

  /*! \brief Assuming the point is rotated by angle phi, this function
   *         computes the point coordinates in the base frame.
   */
  void toWorld(double worldPt[2], double phi) const;

  /*! \brief Assuming the point is rotated by angle phi, this function
   *         computes the point coordinates in the base frame.
   */
  void getContactInWorld(double& x, double& y, double phi) const;

  /*! \brief Given a rotation angle, this function determines if the point
   *         satisfies friction constraints etc. In case it is a valid
   *         contact point, the function returns true, otherwise false.
   */
  bool isSupportPoint(double phi) const;

  /*! \brief Returns true if the other point's normal is 180deg +/- maxAngle
   *         rotated with respect to the point.
   */
  bool isOpposing(const PolygonContactPoint2D& other, double maxAngle) const;

  /*! \brief Returns the Euclidian distance between the points.
   */
  double getDistance(const PolygonContactPoint2D& other) const;

  /*! \brief Returns the inclination angle of the normal rotated by angle phi.
   */
  double getInclinationAngle(double phi) const;

  /*! \brief Returns the point's x-component.
   */
  double getX() const;

  /*! \brief Returns the point's y-component.
   */
  double getY() const;

  /*! \brief Assigns frictionAngle to the point.
   */
  void setFrictionAngle(double frictionAngle);

  /*! \brief Returns the point's friction angle.
   */
  double getFrictionAngle() const;

  /*! \brief Returns the point's normal angle.
   */
  double getNormalAngle() const;

  /*! \brief Sets the point's maxLever property.
   */
  void setMaxLever(double distance);

  /*! \brief Returns the point's maxLever property.
   */
  double getMaxLever() const;

  /*! \brief Sets the point's x-component.
   */
  void setX(double value);

  /*! \brief Sets the point's y-component.
   */
  void setY(double value);

  /*! \brief Sets the point's normal angle.
   */
  void setNormalAngle(double value);


private:

  double pt[2];
  double normalAngle;
  double frictionAngle;
  double maxLever;
};

}

#endif // RCS_POLYGONCONTACTPOINT2D_H
