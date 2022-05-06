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

#ifndef RCS_BOXSTRATEGY5D_H
#define RCS_BOXSTRATEGY5D_H

#include <ExplorationStrategy.h>
#include <Rcs_HTr.h>

#include <memory>


namespace Rcs
{

class BoxStrategy5D : public Gras::ExplorationStrategy
{
public:

  enum ObjectType
  {
    None = 0,
    Box,
    Cylinder,
    LShape
  };

  enum GoalCondition
  {
    FullState = 0,
    RotationOnly,
    RotationWithStableHold,
    RotationWithMaxStableHold
  };




  /*!
   * \brief Constructor based on end state and box dimensions
   * \param object                The ObjectType from enum list (box, cylinder, Lshape)
   * \param numPhiDiscretizations Number of discretizations for the rotation
   *                              angle over 360 degree
   */
  BoxStrategy5D(ObjectType object, int numPhiDiscretizations);

  /*!
   * \brief Destructor
   */
  virtual ~BoxStrategy5D();

  // Base class methods
  virtual std::vector<std::vector<int>> explore(const std::vector<int>& state) const;
  bool goalReached(const std::vector<int>& state) const;
  double heuristicCost(const std::vector<int>& value) const;
  double transitionCost(const std::vector<int>& from,
                        const std::vector<int>& to) const;
  bool checkState(std::vector<int> state) const;




  bool checkState(int phi, int ra, int la, int rp, int lp, bool testMe=false) const;
  bool checkState(std::vector<int> state, bool testMe) const;
  bool checkTransition(int ph0, int ra0, int la0, int rp0, int lp0,
                       int phi1, int ra1, int la1, int rp1, int lp1) const;
  void setNumPhiDiscretizations(unsigned int numPhiDiscretizations_in);
  int getNumPhiDiscretizations() const;
  double getDeltaPhi() const;
  void setFrictionAngle(double angle, int contactIdx=-1);
  double getFrictionAngle(size_t contactIdx) const;
  std::vector<double> getFrictionAngles() const;
  void setMaxLever(double lever, int contactIdx=-1);
  void setMaxAngularStateChange(double dPhi);
  double getMaxAngularStateChange() const;
  void setMinimumHandDistance(double distance);
  double getMinimumHandDistance() const;
  void setMaxAngleBetweenHands(double angle);
  double getMaxAngleBetweenHands() const;
  int getPhiFromAngle(double angle) const;
  double getAngleFromPhi(int phi) const;
  ObjectType getObjectType() const;
  bool setObjectType(ObjectType object);
  GoalCondition getGoalCondition() const;
  void setGoalCondition(GoalCondition newCondition);
  void setGoal(const std::vector<int>& goal);
  void setEnableReba(bool enable);
  bool getEnableReba() const;

  std::vector<int> getStableHandSupports(int phi) const;
  int getLeftMostSupportPoint(int phi) const;
  int getRightMostSupportPoint(int phi) const;

  int getSupportPoint(int phi, bool left, bool robot = true, bool print = false) const;

  class ContactPoint2D
  {
  public:

    ContactPoint2D();
    ContactPoint2D(double x,  double y, double normalAngle,
                   double frictionAngle, double maxLever);
    virtual ~ContactPoint2D();
    void shift(double sx, double sy);
    void scale(double sx, double sy);
    static void toWorld(double worldPt[2], const double boxPt[2], double phi);
    void toWorld(double worldPt[2], double phi) const;
    void getContactInWorld(double& x, double& y, double phi) const;
    bool isSupportPoint(double phi) const;
    double getMomentAroundPoint(double phi, double mass = 1.0) const;
    bool isOpposing(const ContactPoint2D& other, double maxAngle) const;
    double getDistance(const ContactPoint2D& other) const;
    double getInclinationAngle(double phi) const;
    double getX() const;
    double getY() const;
    void setFrictionAngle(double frictionAngle);
    double getFrictionAngle() const;
    double getNormalAngle() const;
    void setMaxLever(double distance);
    double getMaxLever() const;
    void setX(double value);
    void setY(double value);
    void setNormalAngle(double value);


  private:

    double pt[2];
    double normalAngle;
    double frictionAngle;
    double maxLever;
  };

  static std::vector<HTr> getRoboContacts3D(std::vector<ContactPoint2D> roboContacts);
  static std::vector<HTr> getPartnerContacts3D(std::vector<ContactPoint2D> partnerContacts);
  std::vector<HTr> getRoboContacts3D() const;
  std::vector<HTr> getPartnerContacts3D() const;

  std::vector<ContactPoint2D> roboContacts;
  std::vector<ContactPoint2D> partnerContacts;

private:

  double deltaPhi;
  static unsigned int numPhiDiscretizations;
  double maxAngularStateChange;
  double minimumHandDistance;
  double maxAngleBetweenHands;
  double mass;
  ObjectType object;
  GoalCondition goalCondition;
  std::vector< std::vector<int>> goalStates;

public:
  static std::vector<ContactPoint2D> getBoxContacts(double lengthX,
                                                    double lengthY);
  static std::vector<ContactPoint2D> getCylinderContacts(double radius);
  static std::vector<ContactPoint2D> getLShapeContacts(double lengthSide,
                                                       double lengthUp);
  static void mirrorContacts(std::vector<ContactPoint2D>& contacts);
};

typedef std::shared_ptr<BoxStrategy5D> BoxStrategy5D_ptr;

}

#endif // RCS_BOXSTRATEGY5D_H
