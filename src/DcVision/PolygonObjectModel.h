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

#ifndef DC_POLYGONOBJECTMODEL_H
#define DC_POLYGONOBJECTMODEL_H

#include "PolygonContactPoint2D.h"

#include <Rcs_MatNd.h>
#include <Rcs_HTr.h>

#include <string>
#include <vector>


namespace Dc
{

/*! \brief Class to compute and represent grasp features from polygon data.
 *         The grasp features are contact points and normals in 2d. The
 *         coordinate system is depicted below for an L-shape and box. The
 *         y-axis is assumed to point up. The numbers show the enumerated
 *         contact points.
 *
 *      y                               y
 *      ^                               ^
 *      |                               |
 *      ---> x                          ---> x
 *
 *         7
 *       ______                             6    5    4    3    2
 *      |<.25->|                         ____________________________
 *    8 |      | 6                    7 |            e1              | 1
 *      |      |                        |                            |
 *      |      | 5                    8 |e2                        e0| 0
 *    9 |      |                        |            e3              |
 *      |      | 4                    9 |____________________________| 15
 *      |      |                            10   11   12   13   14
 *   10 |      |    3     2      1
 *      |      |_____________________
 *      |                            |
 *   11 |<--------- 1 -------------->|
 *      |____________________________|
 *         12     13     14      15
 *
 *         The class provides some convenience methods to mirror the points to
 *         the opposite side. It is assumed that the 2d polygon outline
 *         protrudes into the third direction.
 *
*
*         Normal angles and directions: around z-axis, zero angle is at y-axis
*
*                     0
*                     |
*             90  ----+----  -90
*                     |
*                    180
 */
class PolygonObjectModel
{

public:

  /*! \brief Constructs an empty polygon model without any geometry.
   */
  PolygonObjectModel();

  /*! \brief Destructor is empty, we implement it for proper polymorphism so
   *         that other classes can derieve from it.
   */
  virtual ~PolygonObjectModel();

  /*! \brief Convenience function to set the contact features from an internal
   *         analytic box model of the given dimensions. See contact features
   *         and enumeration below. The frame is aligned with the box center.
   *         The name is set to "Box".
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
   *  \param[in] lengthX   Width of the box
   *  \param[in] lengthY   Height of the box
   */
  void setBoxPolygon(double lengthX, double lengthY);

  /*! \brief Convenience function to set the contact features from an internal
   *         analytic cylinder model of the given dimensions. See contact
   *         features and enumeration below. The frame is aligned with the
   *         cylinder's center. The name is set to "Cylinder".
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
   *  \param[in] radius   Cylinder radius
   */
  void setCylinderPolygon(double radius);

  /*! \brief Convenience function to set the contact features from an internal
   *         analytic L-shape model of the given dimensions. See contact
   *         features and enumeration below. The frame is located at 0.34
   *         units from the lower left edge. This corresponds to the L-shape's
   *         mass center. The with of the sides is 0.25 units. The name is
   *         set to "LShape".
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
   *  \param[in] lengthX   Width of L-shape
   *  \param[in] lengthY   Height of L-shape
   */
  void setLShapePolygon(double lengthX, double lengthY);

  /*! \brief Sets the polygon from a set of contact points and normals.
   */
  void setPolygon(const MatNd* contactPoints,
                  const MatNd* contactNormals);

  /*! \brief Sets the polygon from a file. The file is assumed to hold a
   *         2d MatNd, the units in mm. This function was for testing and is
   *         deprecated.
   */
  void setPolygon2(std::string fileName);

  /*! \brief Returns the vector of 2d contact points.
   */
  std::vector<PolygonContactPoint2D> getContacts2d() const;

  /*! \brief Returns the contact transforms.
   */
  std::vector<HTr> getContacts3D() const;

  /*! \brief Returns the contact transforms mirrored to the opposite side
   *         of the side.
   */
  std::vector<HTr> getMirroredContacts3D() const;

  /*! \brief Returns true if contact points exist, false otherwise.
   */
  bool valid() const;

  /*! \brief Returns the id of the leftmost stable contact point for the
   *         polygon. Friction cone and other properties are considered.
   */
  int getLeftMostRoboSupportPoint(double phi) const;

  /*! \brief Returns the id of the rightmost stable contact point for the
   *         polygon. Friction cone and other properties are considered.
   */
  int getRightMostRoboSupportPoint(double phi) const;

  /*! \brief Mirrored version of \ref getLeftMostRoboSupportPoint(double)
   */
  int getLeftMostMirroredSupportPoint(double phi) const;

  /*! \brief Mirrored version of \ref getRightMostRoboSupportPoint(double)
   */
  int getRightMostMirroredSupportPoint(double phi) const;

  /*! \brief Returns the name of the model as set by its creator.
   */
  std::string getName() const;

  /*! \brief Removes all contacts.
   */
  void clear();

private:

  static std::vector<PolygonContactPoint2D>
  getCylinderContacts(double radius);

  static std::vector<PolygonContactPoint2D>
  getLShapeContacts(double lengthX, double lengthY);

  static std::vector<PolygonContactPoint2D>
  getBoxContacts(double lengthX, double lengthY);

  static std::vector<PolygonContactPoint2D>
  getContactsFromPointSet(const MatNd* contactPoints,
                          const MatNd* contactNormals);

  static std::vector<HTr>
  getContacts3D(std::vector<PolygonContactPoint2D> contacts);

  static std::vector<HTr>
  getMirroredContacts3D(std::vector<PolygonContactPoint2D> contacts);

  /*! \brief Mirrors the 2d contacts on the y-z plane: x -> -x and
   *         normalAngle -> -normalAngle.
   */
  static void mirrorContacts(std::vector<PolygonContactPoint2D>& contacts);

  static int getLeftMostSupportPoint(std::vector<PolygonContactPoint2D> contacts, double phi);

  static int getRightMostSupportPoint(std::vector<PolygonContactPoint2D> contacts, double phi);

  std::vector<PolygonContactPoint2D> polyContacts;
  std::vector<HTr> contacts3D;
  std::vector<HTr> mirroredContacts3D;
  std::string name;
};

}



#endif   // DC_POLYGONOBJECTMODEL_H
