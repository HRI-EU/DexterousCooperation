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

#include "PolygonObjectModel.h"
#include "PolyGraspDetector.h"

#include <Rcs_macros.h>
#include <Rcs_math.h>


namespace Dc
{

PolygonObjectModel::PolygonObjectModel()
{
}

PolygonObjectModel::~PolygonObjectModel()
{
}

std::string PolygonObjectModel::getName() const
{
  return this->name;
}

void PolygonObjectModel::setBoxPolygon(double lengthX, double lengthY)
{
  this->polyContacts = getBoxContacts(lengthX, lengthY);
  this->contacts3D = getContacts3D(polyContacts);
  this->mirroredContacts3D = getMirroredContacts3D(polyContacts);
  this->name = "Box";
}

void PolygonObjectModel::setCylinderPolygon(double radius)
{
  this->polyContacts = getCylinderContacts(radius);
  this->contacts3D = getContacts3D(polyContacts);
  this->mirroredContacts3D = getMirroredContacts3D(polyContacts);
  this->name = "Cylinder";
}

void PolygonObjectModel::setLShapePolygon(double lengthX, double lengthY)
{
  this->polyContacts = getLShapeContacts(lengthX, lengthY);
  this->contacts3D = getContacts3D(polyContacts);
  this->mirroredContacts3D = getMirroredContacts3D(polyContacts);
  this->name = "LShape";
}

void PolygonObjectModel::setPolygon(const MatNd* contactPoints,
                                    const MatNd* contactNormals)
{
  polyContacts = getContactsFromPointSet(contactPoints, contactNormals);

  contacts3D = getContacts3D(polyContacts);
  mirroredContacts3D = getMirroredContacts3D(polyContacts);
  this->name = std::to_string(contactPoints->m) + "_vertices";
}

void PolygonObjectModel::setPolygon2(std::string fileName)
{
  MatNd* polyArr = MatNd_createFromFile(fileName.c_str());

  if (polyArr==NULL)
  {
    RLOG(0, "Failed to create polygon from file \"%s\"", fileName.c_str());
    return;
  }

  RLOG(5, "Created polygon file with %d vertices", polyArr->m);
  MatNd_constMulSelf(polyArr, 0.001);   // File dimensions are mm

  PolyGraspDetector graspDetector;
  graspDetector.setPolygon(polyArr);
  MatNd_destroy(polyArr);

  MatNd* contacts = MatNd_create(1, 2);
  MatNd* normals = MatNd_create(1, 2);
  graspDetector.getContacts(contacts, normals);

  setPolygon(contacts, normals);
  this->name = fileName;

  MatNd_destroy(contacts);
  MatNd_destroy(normals);
}

void PolygonObjectModel::clear()
{
  this->polyContacts.clear();
  this->contacts3D.clear();
  this->mirroredContacts3D.clear();
}

bool PolygonObjectModel::valid() const
{
  return polyContacts.empty() ? false : true;
}

std::vector<PolygonContactPoint2D> PolygonObjectModel::getContacts2d() const
{
  return polyContacts;
}

std::vector<HTr> PolygonObjectModel::getContacts3D() const
{
  return this->contacts3D;
}

std::vector<HTr> PolygonObjectModel::getMirroredContacts3D() const
{
  return this->mirroredContacts3D;
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
std::vector<PolygonContactPoint2D>
PolygonObjectModel::getCylinderContacts(double radius)
{
  std::vector<PolygonContactPoint2D> contact(16);

  const double frictAng = RCS_DEG2RAD(65.0);
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
    contact[i] = PolygonContactPoint2D(x, y, phiNormal, frictAng, maxLever);
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
std::vector<PolygonContactPoint2D>
PolygonObjectModel::getLShapeContacts(double lengthX, double lengthY)
{
  std::vector<PolygonContactPoint2D> contact(16);

  const double frictAng = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;

  contact[0]  = PolygonContactPoint2D(1.0, 0.125, -M_PI_2, frictAng, maxLever);

  contact[1]  = PolygonContactPoint2D(0.8,  0.25, 0.0, frictAng, maxLever);
  contact[2]  = PolygonContactPoint2D(0.6,  0.25, 0.0, frictAng, maxLever);
  contact[3]  = PolygonContactPoint2D(0.4,  0.25, 0.0, frictAng, maxLever);

  contact[4]  = PolygonContactPoint2D(0.25, 0.4, -M_PI_2, frictAng, maxLever);
  contact[5]  = PolygonContactPoint2D(0.25, 0.6, -M_PI_2, frictAng, maxLever);
  contact[6]  = PolygonContactPoint2D(0.25, 0.8, -M_PI_2, frictAng, maxLever);

  contact[7]  = PolygonContactPoint2D(0.125,  1.0, 0.0, frictAng, maxLever);


  contact[8]  = PolygonContactPoint2D(0.0, 0.8, M_PI_2, frictAng, maxLever);
  contact[9]  = PolygonContactPoint2D(0.0, 0.6, M_PI_2, frictAng, maxLever);
  contact[10] = PolygonContactPoint2D(0.0, 0.4, M_PI_2, frictAng, maxLever);
  contact[11] = PolygonContactPoint2D(0.0, 0.2, M_PI_2, frictAng, maxLever);

  contact[12] = PolygonContactPoint2D(0.2, 0.0, M_PI, frictAng, maxLever);
  contact[13] = PolygonContactPoint2D(0.4, 0.0, M_PI, frictAng, maxLever);
  contact[14] = PolygonContactPoint2D(0.6, 0.0, M_PI, frictAng, maxLever);
  contact[15] = PolygonContactPoint2D(0.8, 0.0, M_PI, frictAng, maxLever);

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
 *    Normal direction:
 *
 *                     0
 *                     |
 *             90  ----+----  -90
 *                     |
 *                    180
 *
 ******************************************************************************/
std::vector<PolygonContactPoint2D>
PolygonObjectModel::getBoxContacts(double lengthX, double lengthY)
{
  std::vector<PolygonContactPoint2D> contact(16);

  const double frictAng = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;

  contact[15] = PolygonContactPoint2D(0.5, -0.33, -M_PI_2, frictAng, maxLever);
  contact[0]  = PolygonContactPoint2D(0.5,  0.0, -M_PI_2, frictAng, maxLever);
  contact[1]  = PolygonContactPoint2D(0.5,  0.33, -M_PI_2, frictAng, maxLever);

  contact[2]  = PolygonContactPoint2D(0.33, 0.5, 0.0, frictAng, maxLever);
  contact[3]  = PolygonContactPoint2D(0.165, 0.5, 0.0, frictAng, maxLever);
  contact[4]  = PolygonContactPoint2D(0.0, 0.5, 0.0, frictAng, maxLever);
  contact[5]  = PolygonContactPoint2D(-0.165, 0.5, 0.0, frictAng, maxLever);
  contact[6]  = PolygonContactPoint2D(-0.33, 0.5, 0.0, frictAng, maxLever);

  contact[7]  = PolygonContactPoint2D(-0.5,  0.33, M_PI_2, frictAng, maxLever);
  contact[8]  = PolygonContactPoint2D(-0.5,  0.0, M_PI_2, frictAng, maxLever);
  contact[9]  = PolygonContactPoint2D(-0.5, -0.33, M_PI_2, frictAng, maxLever);

  contact[10] = PolygonContactPoint2D(-0.33, -0.5, M_PI, frictAng, maxLever);
  contact[11] = PolygonContactPoint2D(-0.165, -0.5, M_PI, frictAng, maxLever);
  contact[12] = PolygonContactPoint2D(0.0, -0.5, M_PI, frictAng, maxLever);
  contact[13] = PolygonContactPoint2D(0.165, -0.5, M_PI, frictAng, maxLever);
  contact[14] = PolygonContactPoint2D(0.33, -0.5, M_PI, frictAng, maxLever);

  for (size_t i=0; i<contact.size(); ++i)
  {
    contact[i].scale(lengthX, lengthY);
  }

  return contact;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<PolygonContactPoint2D>
PolygonObjectModel::getContactsFromPointSet(const MatNd* contactPoints,
                                            const MatNd* contactNormals)
{
  RCHECK(contactPoints->m==contactNormals->m);
  RCHECK(contactPoints->n==2);
  RCHECK(contactNormals->n==2);

  const unsigned int nVertices = contactNormals->m;
  const double frictAng = RCS_DEG2RAD(65.0);
  const double maxLever = 1.0;

  std::vector<PolygonContactPoint2D> contact(nVertices);

  for (unsigned int i=0; i<nVertices; ++i)
  {
    const double* cp_i = MatNd_getRowPtr(contactPoints, i);
    const double* n_i = MatNd_getRowPtr(contactNormals, i);
    const double phi = atan2(-n_i[0], n_i[1]);

    contact[i] = PolygonContactPoint2D(cp_i[0], cp_i[1], phi, frictAng,
                                       maxLever);
  }

  return contact;
}

/*******************************************************************************
 * These are the original normal angles:
 *                     0
 *                     |
 *             90  ----+----  -90
 *                     |
 *                    180
 *
 *
 * These are the mirrored normal angles:
 *                    -0
 *                     |
 *            -90  ----+----  90
 *                     |
 *                    180
 *
 ******************************************************************************/
void PolygonObjectModel::mirrorContacts(std::vector<PolygonContactPoint2D>& contacts)
{
  for (size_t i=0; i<contacts.size(); ++i)
  {
    contacts[i].setX(-contacts[i].getX());
    contacts[i].setNormalAngle(Math_fmodAngle(-contacts[i].getNormalAngle()));
  }
}

/*******************************************************************************
 * Normals point from outside towards contact (not away)
 ******************************************************************************/
std::vector<HTr> PolygonObjectModel::getContacts3D(std::vector<PolygonContactPoint2D> contacts2D)
{
  std::vector<HTr> contacts;
  HTr handTransform;
  double x[6];

  for (size_t i = 0; i<contacts2D.size(); ++i)
  {
    VecNd_set6(x, contacts2D[i].getX(), contacts2D[i].getY(), 0.0, 0.0, 0.0,
               Math_fmodAngle(contacts2D[i].getNormalAngle() - M_PI));
    HTr_from6DVector(&handTransform, x);
    contacts.push_back(handTransform);
  }

  return contacts;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<HTr> PolygonObjectModel::getMirroredContacts3D(std::vector<PolygonContactPoint2D> contacts2D)
{
  std::vector<HTr> contact;
  HTr handTransform;
  double x[6];

  std::vector<PolygonContactPoint2D> mirroredContacts2D = contacts2D;
  mirrorContacts(mirroredContacts2D);

  for (size_t i = 0; i<mirroredContacts2D.size(); ++i)
  {
    VecNd_set6(x, mirroredContacts2D[i].getX(), mirroredContacts2D[i].getY(), 0.0,
               0.0, 0.0,
               Math_fmodAngle(mirroredContacts2D[i].getNormalAngle() - M_PI));
    HTr_from6DVector(&handTransform, x);
    contact.push_back(handTransform);
  }

  return contact;
}

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getLeftMostSupportPoint(std::vector<PolygonContactPoint2D> contacts, double phi)
{
  int leftMostSupport = -1;
  double x = 0.0, y = 0.0, x_min = 0.0, y_min = 0.0;

  for (size_t i = 0; i < contacts.size(); ++i)
  {
    if (contacts[i].isSupportPoint(phi))
    {
      contacts[i].getContactInWorld(x, y, phi);

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

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getRightMostSupportPoint(std::vector<PolygonContactPoint2D> contacts, double phi)
{
  int rightMostSupport = -1;
  double x = 0.0, y = 0.0, x_max = 0.0, y_max = 0.0;

  for (size_t i = 0; i < contacts.size(); ++i)
  {
    if (contacts[i].isSupportPoint(phi))
    {
      contacts[i].getContactInWorld(x, y, phi);

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

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getLeftMostRoboSupportPoint(double phi) const
{
  return getLeftMostSupportPoint(polyContacts, phi);
}

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getRightMostRoboSupportPoint(double phi) const
{
  return getRightMostSupportPoint(polyContacts, phi);
}

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getLeftMostMirroredSupportPoint(double phi) const
{
  auto mirroredContacts2D = polyContacts;
  mirrorContacts(mirroredContacts2D);
  return getLeftMostSupportPoint(mirroredContacts2D, phi);
}

/*******************************************************************************
 *
 ******************************************************************************/
int PolygonObjectModel::getRightMostMirroredSupportPoint(double phi) const
{
  auto mirroredContacts2D = polyContacts;
  mirrorContacts(mirroredContacts2D);
  return getRightMostSupportPoint(mirroredContacts2D, phi);
}

}   // namespace
