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

#ifndef DC_WHEELSTRATEGY7D_H
#define DC_WHEELSTRATEGY7D_H

#include <ExplorationStrategy.h>
#include <Rcs_graph.h>
#include <Rcs_Vec3d.h>
#include <Rcs_Mat3d.h>

#include <algorithm>
#include <string>
#include <cmath>



namespace Dc
{

/*! \brief Class to describe discrete search problem of finding a sequence of
 *         steps to move and rotate a wheel, including change of contact points
 *         of one agent. This class inherits from ExplorationStrategy and
 *         exposes the interface required for search algorithms implemented in
 *         the RcsSearch library. We implement some of the functions inline,
 *         since computational efficiency of them is determining the speed for
 *         the search algorithms.
 */
class WheelStrategy7D : public Gras::ExplorationStrategy
{
public:

  /*! \brief Indices to access problem-specific integer states from vectors
   */
  enum StateElement
  {
    WheelCoord = 0,
    WheelFlip,
    WheelRoll,
    RoboContactRight,
    RoboContactLeft,
    HumanContactRight,
    HumanContactLeft,
    StateMaxIndex
  };

  /*! \brief Convenience string vector to print out the above indices
  */
  std::vector<std::string> StateElementName =
  {
    "WheelCoord",
    "WheelFlip",
    "WheelRoll",
    "RoboContactRight",
    "RoboContactLeft",
    "HumanContactRight",
    "HumanContactLeft"
  };

  /************************************************************************
  * Transition Types
  ************************************************************************/

  /*! \brief Enum describing all possible transitions between contact states
   */
  enum TransitionType
  {
    // general
    None = 0,
    TransitionUndefined,
    // object and robo OR partner
    RotateObject,
    RegraspRight,
    RegraspLeft,
    MoveAll,
    // robo and partner
    ObjectMoves,
    RobotRegraspRight,
    RobotRegraspLeft,
    HumanRegraspRight,
    HumanRegraspLeft,
  };

  /*! \brief Convenience string vector to print out the above indices
  */
  static std::string transitionTypeToString(unsigned int type)
  {
    switch (type)
    {
      case RotateObject:
        return "rotate object";
      case RegraspRight:
        return "regrasp right";
      case RegraspLeft:
        return "regrasp left";
      case MoveAll:
        return "move all";
      case ObjectMoves:
        return "object moves";
      case RobotRegraspRight:
        return "regrasp right (robot)";
      case RobotRegraspLeft:
        return "regrasp left (robot)";
      case HumanRegraspRight:
        return "regrasp right (human)";
      case HumanRegraspLeft:
        return "regrasp left (human)";
      case None:
        return "no change in state";
      case TransitionUndefined:
        return "undefined";
      default:
        return "undefined other";
    }
  }

  /*! \brief Convenience method that returns a printable string for a given
   *         state.
   */
  static std::string stateToString(std::vector<int> state)
  {
    std::string stateStr;

    for (size_t i=0; i<state.size(); ++i)
    {
      stateStr += std::to_string(state[i]);
      if (i!=state.size()-1)
      {
        stateStr += " ";
      }
    }

    return stateStr;
  }

  /*! \brief Sets up the geometry propoerties of the search problem from the
   *         passed graph. The graph must be specific to the problem, i.e. a
   *         set of joints and bodies must exist, and the coordinate frames
   *         must be aligned in a given way. The assumptions are evaluated in
   *         the check function, see that for details.
   *
   * \param[in] graph   Representation of the kinematics to initialize the
   *                    class's search problem. No pointer is stored, no copy
   *                    is made.
   */
  WheelStrategy7D(const RcsGraph* graph);

  /*! \brief Virtual destructor to allow proper polymorphism.
   */
  virtual ~WheelStrategy7D()
  {
  }

  /*! \brief Function that computes the heuristic cost from the given state to
   *         the goal state(s). In this implementation, the displacement and the
   *         flip angle of the wheel are considered. Ideally, the h-cost must
   *         fulfil the propoerty of being consistent. It was never really
   *         checked, therefore the found solutions might not always be optimal.
   *
   * \param[in] state     Discrete state according to enum StateElement
   * \return scalar heuristic cost from value to goal.
   */
  double heuristicCost(const std::vector<int>& state) const;

  double transitionCost(const std::vector<int>& from,
                        const std::vector<int>& to) const;

  /*! \brief Prints out the geometric properties of the problem to the console.
   */
  void print() const;

  /*! \brief Determines the discrete search state from the kinematics of the
   *         graph.
   *
   * \param[in] graph   Graph the search state is to be determined for. It must
   *                    fulfill the above mentioned assumptions, otherwise the
   *                    function will lead to a fatal exit of the program.
   */
  std::vector<int> getState(const RcsGraph* graph) const;
  void getQfromState(RcsGraph* graph, const std::vector<int> state) const;
  bool checkState(std::vector<int> state) const;
  bool checkState(std::vector<int> state, bool testMe) const;
  bool checkState(int s, int flip, int roll,
                  int robR, int robL, int humR, int humL,
                  bool testMe=false) const;
  bool checkStateFeasible(int s, int flip, int roll) const;
  bool checkTransition(int s0, int flip0, int roll0,
                       int robR0, int robL0, int humR0, int humL0,
                       int s1, int flip1, int roll1,
                       int robR1, int robL1, int humR1, int humL1) const;


  inline int getNumLinearDiscretizations() const
  {
    return linearDiscretization;
  }

  inline double getDeltaFlip() const
  {
    return 2.0*M_PI/flipDiscretization;
  }

  inline int getNumFlipDiscretizations() const
  {
    return flipDiscretization;
  }

  inline double getDeltaRoll() const
  {
    return 2.0*M_PI/rollDiscretization;
  }

  inline double getDeltaGrasp() const
  {
    return 4.0*M_PI/contactDiscretization;
  }

  inline int getNumContactDiscretizations() const
  {
    return contactDiscretization;
  }

  inline double getFlipAngle(int angularState) const
  {
    return angularState*getDeltaFlip();
  }

  inline double getRollAngle(int angularState) const
  {
    return angularState*getDeltaRoll();
  }

  inline double getGraspAngle(int angularState) const
  {
    return (angularState/2)*getDeltaGrasp();
  }

  inline int getContactDistance(int c1, int c2) const
  {
    //for (int i = -2 * contactDiscretization - 1; i < 2 * contactDiscretization + 1; ++i)
    //{
    //  printf("%d mod %d = %d\n", i, contactDiscretization, std::abs(i % contactDiscretization));
    //}

    // Project both contact states to the range [0 ... contactDiscretization [
    // We assume the contacts are spaced symmetric around state 0
    c1 = std::abs(c1 % contactDiscretization);
    c2 = std::abs(c2 % contactDiscretization);

    // Now we take the shortest distance among the three possibilities:
    int res = std::abs(c1 - c2);
    res = std::min(res, std::abs(c1 - (c2- contactDiscretization)));
    res = std::min(res, std::abs(c1 - (c2+ contactDiscretization)));

    return res;
  }

  inline int wheelHorizontal(int flip) const
  {
    return flip%(flipDiscretization/2)==0 ? true : false;
  }

  inline int wheelVertical(int flip) const
  {
    return (flip+(flipDiscretization/4))%(flipDiscretization/2)==0 ? true : false;
  }

  inline bool contactInsideRange(int c1, int c2, int cQuery) const
  {
    const int lower = std::min(c1, c2)/2;
    const int upper = std::max(c1, c2)/2;
    cQuery /= 2;

    // Lower -> upper is short connection between c1 and c2
    if (upper - lower < contactDiscretization / 4)
    {
      return (cQuery > lower && cQuery < upper) ? true : false;
    }

    // Lower -> upper is long connection between c1 and c2
    return (cQuery >= lower && cQuery <= upper) ? false : true;
  }

  inline double getWheelRadius() const
  {
    return this->wheelRadius;
  }

  inline void getWheelRotMat(double A_WheelI[3][3], int flip, int roll) const
  {
    double A_FI[3][3], A_RF[3][3];
    const double rollAngle = getRollAngle(roll);
    const double flipAngle = getFlipAngle(flip);
    Mat3d_setRotMatX(A_FI, flipAngle);
    Mat3d_setRotMatZ(A_RF, rollAngle);
    Mat3d_mul(A_WheelI, A_RF, A_FI);
  }

  inline void getHandInWorld(double x_hand[3], int contactState, double A_WheelI[3][3], double r_wheel) const
  {
    Vec3d_set(x_hand, cos(getGraspAngle(contactState)), sin(getGraspAngle(contactState)), 0.0);
    Vec3d_constMulSelf(x_hand, r_wheel);
    Vec3d_transRotateSelf(x_hand, A_WheelI);
  }

  // Robot right palm normal: positive z-axis. If robR is odd, wrist is flipped.
  inline void getRoboRightPalmNormal(double palmNormalR[3], int robR, double A_WheelI[3][3])  const
  {
    Vec3d_set(palmNormalR, 0.0, 0.0, robR % 2 == 0 ? 1.0 : -1.0);
    Vec3d_transRotateSelf(palmNormalR, A_WheelI);
  }

  // Robot left palm normal: negative z-axis. If robL is odd, wrist is flipped.
  inline void getRoboLeftPalmNormal(double palmNormalL[3], int robL, double A_WheelI[3][3])  const
  {
    Vec3d_set(palmNormalL, 0.0, 0.0, robL % 2 == 0 ? -1.0 : 1.0);
    Vec3d_transRotateSelf(palmNormalL, A_WheelI);
  }

  inline double getWheelGap(int s, int flip) const
  {
    const double wheelHeight = sState[s].org[2];
    const double flipAngle = getFlipAngle(flip);
    return wheelHeight - fabs(this->wheelRadius*sin(flipAngle));
  }

  inline double getLateralWheelGap(int s, int flip) const
  {
    const double wheelY = sState[s].org[1];
    const double flipAngle = getFlipAngle(flip);
    return wheelY - fabs(this->wheelRadius*cos(flipAngle)) - sState.back().org[1];
  }

  static inline bool isWristRotation(int contact0, int contact1)
  {
    return ((contact1-contact0) % 2==0) ? false : true;
  }

  bool goalReached(const std::vector<int>& state) const;
  bool test(const RcsGraph* graph, int numIterations) const;

  std::vector<HTr> getWheelTrajectory() const
  {
    return this->sState;
  }

  static void printSolutionPath(const std::vector<std::vector<int>>& states);
  bool checkShoulderHeightAlignment(const double palmNormalR[3],
                                    const double palmNormalL[3],
                                    double rightHandZ,
                                    double leftHandZ,
                                    double wheelHeight,
                                    bool testing=false) const;

  bool checkHuman(int humR, int humL, double A_RI[3][3], double wheelHeight, bool testing=false) const;

  inline int getLinearDiscretiztion() const
  {
    return this->linearDiscretization;
  }

  static int getNumberOfContactChanges(std::vector<int> from, std::vector<int> to);
  static unsigned int getTransitionType(std::vector<int> s0, std::vector<int> s1);
  double getTtc(std::vector<int> stateFrom, std::vector<int> stateTo, double baseTtc) const;
  void setGoal(const std::vector<int>& goal);


protected:

  bool initializeStateSpace(const RcsGraph* graph);
  std::vector<std::vector<int>> explore(const std::vector<int>& currentState) const;
  std::vector<std::vector<int>> explore3(const std::vector<int>& currentState) const;

  std::vector<std::vector<int>> exploreFlipTranslation(int s, int flip, int roll,
                                                       int robR, int robL,
                                                       int humR, int humL) const;
  std::vector<std::vector<int>> exploreTranslation(int s, int flip, int roll,
                                                   int robR, int robL,
                                                   int humR, int humL) const;
  std::vector<std::vector<int>> exploreFlip(int s, int flip, int roll,
                                            int robR, int robL,
                                            int humR, int humL) const;

  std::vector<std::vector<int>> exploreRoboRight(int s, int flip, int roll,
                                                 int robR, int robL,
                                                 int humR, int humL) const;
  std::vector<std::vector<int>> exploreRoboLeft(int s, int flip, int roll,
                                                int robR, int robL,
                                                int humR, int humL) const;

  std::vector<std::vector<int>> exploreHumanRight(int s, int flip, int roll,
                                                  int robR, int robL,
                                                  int humR, int humL) const;
  std::vector<std::vector<int>> exploreHumanLeft(int s, int flip, int roll,
                                                 int robR, int robL,
                                                 int humR, int humL) const;

  bool check(const RcsGraph* graph) const;

  std::vector<HTr> sState;
  std::vector<int> goalState;

  double wheelRadius;
  double shoulderheight;
  int linearDiscretization;
  int flipDiscretization;   // Number of tics to get once around 360 degrees
  int rollDiscretization;   // Number of tics to get once around 360 degrees
  int contactDiscretization;   // Number of tics to get once around 360 degrees
};

}   // namespace Dc

#endif // DC_WHEELSTRATEGY7D_H
