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

#include "WheelConstraint.h"
#include "WheelStrategy7D.h"
#include "EventSystem.h"

#include <Rcs_macros.h>

namespace Dc
{

NotifyConstraintSet::NotifyConstraintSet(double timeToFire, std::string notifyerName, std::function<void()> cb)
{
  setClassName(notifyerName);
  RLOG(0, "New NotifyConstraintSet with t=%5.2f", timeToFire);
  time = timeToFire;
  callback = std::move(cb);
}

void NotifyConstraintSet::clear()
{
  RLOG(0, "++++--> Clearing notify constraint!");
  ConstraintSet::clear();
}

double NotifyConstraintSet::compute(double dt)
{
  time -= dt;

  if (time > -1 && time <= TRAJECTORY1D_ALMOST_ZERO)
  {
    time = -1;

    callback();
  }
  ConstraintSet::compute(dt);
  return time;
}

double NotifyConstraintSet::getStartTime() const
{
  return time;
}

double NotifyConstraintSet::getEndTime() const
{
  return std::max(this->time, ConstraintSet::getEndTime());
}

void NotifyConstraintSet::shiftTime(double dt)
{
  time += dt;
  ConstraintSet::shiftTime(dt);
}

std::string NotifyConstraintSet::getClassName() const
{
  return std::string("NotifyConstraintSet");
}

}   // namespace Dc





namespace Dc
{

// Distance: 0.0 is engage, 0.1 is disengage
class EngageHands : public tropic::ConstraintSet
{
public:
  EngageHands(double t_goal, double distance,
              tropic::TrajectoryND* constraintRoboRight,
              tropic::TrajectoryND* constraintRoboLeft)
  {
    if (constraintRoboRight != NULL)
    {
      add(t_goal, distance, constraintRoboRight->getTrajectory1D(0)->getName());

      for (size_t i = 1; i < constraintRoboRight->getInternalDim(); ++i)
      {
        add(t_goal, 0, constraintRoboRight->getTrajectory1D(i)->getName());
      }
    }

    if (constraintRoboLeft != NULL)
    {
      add(t_goal, distance, constraintRoboLeft->getTrajectory1D(0)->getName());

      for (size_t i = 1; i < constraintRoboLeft->getInternalDim(); ++i)
      {
        add(t_goal, 0, constraintRoboLeft->getTrajectory1D(i)->getName());
      }
    }
  }

  virtual ~EngageHands()
  {
  }

  std::string getClassName() const
  {
    return std::string("EngageHands");
  }
};


class MoveFingers : public tropic::ConstraintSet
{
public:

  MoveFingers(double t_goal, const double jointAngles[7], tropic::TrajectoryND* fingerTraj)
  {
    init(t_goal, jointAngles, fingerTraj);
  }

  MoveFingers(double t_goal, const double jointAngles[7], tropic::TrajectoryND* fingerTraj1, tropic::TrajectoryND* fingerTraj2)
  {
    init(t_goal, jointAngles, fingerTraj1);
    init(t_goal, jointAngles, fingerTraj2);
  }

  void init(double t_goal, const double jointAngles[7], tropic::TrajectoryND* fingerTraj)
  {
    if (fingerTraj==NULL || fingerTraj->getInternalDim()!=7)
    {
      RLOG(1, "Moving without fingers");
      return;
    }

    for (size_t i = 0; i < fingerTraj->getInternalDim(); ++i)
    {
      add(t_goal, jointAngles[i], fingerTraj->getTrajectory1D(i)->getName());
    }
  }

  virtual ~MoveFingers()
  {
  }

  std::string getClassName() const
  {
    return std::string("MoveFingers");
  }
};





WheelConstraint::WheelConstraint(tropic::TrajectoryControllerBase* tc,
                                 const WheelStrategy7D* strategy_,
                                 ES::EventSystem* es) : strategy(strategy_), eventSystem(es)
{
  this->trajWheel = tc->getTrajectory("Wheel");
  this->trajRoboRight = tc->getTrajectory("RobotRight");
  this->trajRoboLeft = tc->getTrajectory("RobotLeft");
  this->constraintRoboRight = tc->getTrajectory("RobotRightConstraint");
  this->constraintRoboLeft = tc->getTrajectory("RobotLeftConstraint");
  this->constraintRoboSideways = tc->getTrajectory("Sideways");
  this->fingersR = tc->getTrajectory("Fingers_R");
  this->fingersL = tc->getTrajectory("Fingers_L");

  RCHECK(this->trajWheel);
  RCHECK(this->trajRoboRight);
  RCHECK(this->trajRoboLeft);
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::addSolution(std::vector<std::vector<int>> solution, double ttc)
{
  auto sln = std::make_shared<tropic::ConstraintSet>();

  if (solution.empty())
  {
    return sln;
  }

  if (solution.size() == 1)
  {
    sln->add(moveTo(solution[0], ttc));
    return sln;
  }

  double t = 0.0;

  if (!isEngaged())
  {
    sln->add(std::make_shared<EngageHands>(0.5*ttc, 0.0, constraintRoboRight, constraintRoboLeft));
    sln->add(std::make_shared<MoveFingers>(0.5*ttc, fingersOpenedAngles(), fingersR, fingersL));
    sln->add(std::make_shared<MoveFingers>(0.5*ttc, fingersClosedAngles(), fingersR, fingersL));

    RLOG_CPP(0, "[" << WheelStrategy7D::stateToString(solution[0])
             << "] at t=" << ttc << " (Engage)");

    sln->add(moveTo(solution[0], ttc));
    t = ttc;
  }




  for (size_t i = 0; i < solution.size()-1; ++i)
  {
    // Scale base ttc depending on type and distances
    double localTtc = strategy->getTtc(solution[i], solution[i+1], ttc);

    RLOG_CPP(0, "[" << WheelStrategy7D::stateToString(solution[i+1])<< "] at t="
             << t+localTtc << " (moveFromTo(" << i << ", " << i+1 << ")");

    sln->add(moveFromTo(solution[i], solution[i+1], t, localTtc));
    t += localTtc;
  }

  return sln;
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::moveFromTo(std::vector<int> from, std::vector<int> to,
                                                                   double t_start, double ttc)
{
  auto moveSet = std::make_shared<tropic::ConstraintSet>();
  moveSet->setClassName("StateChange");

  // Now some intermediate points for the hands
  const double retract = 0.14;
  const double shiftSideways = 0.14;
  const double wheelRadius = strategy->getWheelRadius();
  unsigned int tType = WheelStrategy7D::getTransitionType(from, to);


  switch (tType)
  {
    case WheelStrategy7D::RobotRegraspRight:
    {
      std::shared_ptr<tropic::Constraint1D> tc;
      moveSet->add(t_start, wheelRadius, trajRoboRight->getTrajectory1D(2)->getName());

      int robR0 = from[WheelStrategy7D::RoboContactRight];
      int robR1 = to[WheelStrategy7D::RoboContactRight];
      int robL = from[WheelStrategy7D::RoboContactLeft];
      double scaleRetract = WheelStrategy7D::isWristRotation(robR0, robR1) ? 1.0 : 0.25;
      bool handsCross = false;
      if (strategy->contactInsideRange(robR0, robR1, robL))
      {
        RLOG(0, "Left contact inside range: %d %d %d", robR0, robR1, robL);
        scaleRetract = 1.4;
        handsCross = true;
      }
      moveSet->add(t_start + 0.5*ttc, wheelRadius + scaleRetract*retract,
                   0.0, 0.0, 1, trajRoboRight->getTrajectory1D(2)->getName());

      // No wrist rotation: swing a little bit sideways
      if (!WheelStrategy7D::isWristRotation(robR0, robR1) && (constraintRoboRight!=NULL))
      {
        moveSet->add(t_start, 0.0, constraintRoboRight->getTrajectory1D(2)->getName());
        moveSet->add(t_start+0.5*ttc, -shiftSideways, constraintRoboRight->getTrajectory1D(2)->getName());
        moveSet->add(t_start+ttc, 0.0, constraintRoboRight->getTrajectory1D(2)->getName());
      }

      moveSet->add(t_start + ttc, wheelRadius, trajRoboRight->getTrajectory1D(2)->getName());
      moveSet->add(std::make_shared<MoveFingers>(t_start, fingersClosedAngles(), fingersR));

      if (eventSystem && fingersR)
      {
        moveSet->add(std::make_shared<NotifyConstraintSet>(t_start+0.3*ttc-1.0, fingersR->getTrajectory1D(0)->getName(), [this]()
        {
          RLOG(0, "REGRASPING RIGHT OMG!");
          // Notify any listeners that a regrasp will happen in the near future
          this->eventSystem->publish("NotifyRegraspRight", 1.0);
        }));

      }

      if (handsCross)
      {
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.3*ttc, fingersWideOpenedAngles(), fingersR));
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.7*ttc, fingersWideOpenedAngles(), fingersR));
      }
      else
      {
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.3*ttc, fingersOpenedAngles(), fingersR));
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.7*ttc, fingersOpenedAngles(), fingersR));
      }
      moveSet->add(std::make_shared<MoveFingers>(t_start+ttc, fingersClosedAngles(), fingersR));
      break;
    }

    case WheelStrategy7D::RobotRegraspLeft:
    {
      moveSet->add(t_start, wheelRadius, trajRoboLeft->getTrajectory1D(2)->getName());

      int robL0 = from[WheelStrategy7D::RoboContactLeft];
      int robL1 = to[WheelStrategy7D::RoboContactLeft];
      int robR = from[WheelStrategy7D::RoboContactRight];
      double scaleRetract = WheelStrategy7D::isWristRotation(robL0, robL1) ? 1.0 : 0.25;
      bool handsCross = false;
      if (strategy->contactInsideRange(robL0, robL1, robR))
      {
        RLOG(1, "Right contact inside range: %d %d %d", robL0, robL1, robR);
        scaleRetract = 1.4;
        handsCross = true;
      }
      moveSet->add(t_start+0.5*ttc, wheelRadius+scaleRetract*retract,
                   trajRoboLeft->getTrajectory1D(2)->getName());

      // No wrist rotation: swing a little bit sideways
      if (!WheelStrategy7D::isWristRotation(robL0, robL1) &&
          (constraintRoboLeft!=NULL))
      {
        moveSet->add(t_start, 0.0, constraintRoboLeft->getTrajectory1D(2)->getName());
        moveSet->add(t_start+0.5*ttc, shiftSideways, constraintRoboLeft->getTrajectory1D(2)->getName());
        moveSet->add(t_start+ttc, 0.0, constraintRoboLeft->getTrajectory1D(2)->getName());
      }

      moveSet->add(t_start+ttc, wheelRadius, trajRoboLeft->getTrajectory1D(2)->getName());

      moveSet->add(std::make_shared<MoveFingers>(t_start, fingersClosedAngles(), fingersL));

      if (eventSystem && fingersR)
      {
        moveSet->add(std::make_shared<NotifyConstraintSet>(t_start+0.3*ttc-1.0, fingersR->getTrajectory1D(0)->getName(), [this]()
        {
          RLOG(0, "REGRASPING LEFT OMG!");
          // Notify any listeners that a regrasp will happen in the near future
          this->eventSystem->publish("NotifyRegraspLeft", 1.0);
        }));
      }

      if (handsCross)
      {
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.3*ttc, fingersWideOpenedAngles(), fingersL));
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.7*ttc, fingersWideOpenedAngles(), fingersL));
      }
      else
      {
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.3*ttc, fingersOpenedAngles(), fingersL));
        moveSet->add(std::make_shared<MoveFingers>(t_start+0.7*ttc, fingersOpenedAngles(), fingersL));
      }
      moveSet->add(std::make_shared<MoveFingers>(t_start+ttc, fingersClosedAngles(), fingersL));
      break;
    }

    case WheelStrategy7D::ObjectMoves:
    {
      moveSet->add(std::make_shared<MoveFingers>(t_start, fingersClosedAngles(), fingersR, fingersL));
      moveSet->add(std::make_shared<MoveFingers>(t_start+ttc, fingersClosedAngles(), fingersR, fingersL));
      break;
    }

    default:
      break;
  }

  // These are the next contact conditions
  moveSet->add(moveTo(to, t_start+ttc));

  return moveSet;
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::moveTo(std::vector<int> state, double t_goal)
{
  RCHECK(state.size()==WheelStrategy7D::StateElement::StateMaxIndex);

  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  std::vector<HTr> wheelTraj = strategy->getWheelTrajectory();
  double coord;

  // Apply wheel coordinates
  coord = wheelTraj[state[WheelStrategy7D::WheelCoord]].org[0];   // X
  moveSet->add(t_goal, coord, trajWheel->getTrajectory1D(0)->getName());

  coord = wheelTraj[state[WheelStrategy7D::WheelCoord]].org[1];   // Y
  moveSet->add(t_goal, coord, trajWheel->getTrajectory1D(1)->getName());

  coord = wheelTraj[state[WheelStrategy7D::WheelCoord]].org[2];   // Z
  moveSet->add(t_goal, coord, trajWheel->getTrajectory1D(2)->getName());

  coord = strategy->getFlipAngle(state[WheelStrategy7D::WheelFlip]);
  moveSet->add(t_goal, coord, trajWheel->getTrajectory1D(3)->getName());

  coord = strategy->getRollAngle(state[WheelStrategy7D::WheelRoll]);
  moveSet->add(t_goal, coord, trajWheel->getTrajectory1D(4)->getName());

  // Apply robot right hand contact coordinates
  coord = strategy->getGraspAngle(state[WheelStrategy7D::RoboContactRight]);
  moveSet->add(t_goal, coord, trajRoboRight->getTrajectory1D(0)->getName());

  coord = state[WheelStrategy7D::RoboContactRight] % 2 == 0 ? 0.0 : M_PI;
  moveSet->add(t_goal, coord, trajRoboRight->getTrajectory1D(1)->getName());

  // Apply robot left hand contact coordinates
  coord = strategy->getGraspAngle(state[WheelStrategy7D::RoboContactLeft]);
  moveSet->add(t_goal, coord, trajRoboLeft->getTrajectory1D(0)->getName());

  coord = state[WheelStrategy7D::RoboContactLeft] % 2 == 0 ? 0.0 : M_PI;
  moveSet->add(t_goal, coord, trajRoboLeft->getTrajectory1D(1)->getName());

  if (constraintRoboRight != NULL)
  {
    moveSet->add(t_goal, 0.0, constraintRoboRight->getTrajectory1D(0)->getName());
  }

  if (constraintRoboLeft != NULL)
  {
    moveSet->add(t_goal, 0.0, constraintRoboLeft->getTrajectory1D(0)->getName());
  }

  return moveSet;
}

WheelConstraint::~WheelConstraint()
{
}

const double* WheelConstraint::fingersClosedAngles() const
{
  static double fngr[7] =
  {
    RCS_DEG2RAD(-5.0),   // fing3-knuck3
    RCS_DEG2RAD(25.0),   // tip3-fing3
    RCS_DEG2RAD(1.0),   // knuck1-base
    RCS_DEG2RAD(-5.0),   // fing1-knuck1
    RCS_DEG2RAD(25.0),   // tip1-fing1
    RCS_DEG2RAD(-55.0), // fing2-knuck2
    RCS_DEG2RAD(85.0)   // tip2-fing2
  };

  return fngr;
}

const double* WheelConstraint::fingersOpenedAngles() const
{
  static double fngr[7] =
  {
    RCS_DEG2RAD(-70.0),  // fing3-knuck3
    RCS_DEG2RAD(10.0),   // tip3-fing3
    RCS_DEG2RAD(1.0),    // knuck1-base
    RCS_DEG2RAD(-70.0),  // fing1-knuck1
    RCS_DEG2RAD(10.0),   // tip1-fing1
    RCS_DEG2RAD(-89.0),  // fing2-knuck2
    RCS_DEG2RAD(10.0)    // tip2-fing2 was 10
  };


  return fngr;
}

const double* WheelConstraint::fingersWideOpenedAngles() const
{
  static double fngr[7] =
  {
    RCS_DEG2RAD(-70.0),  // fing3-knuck3
    RCS_DEG2RAD(10.0),   // tip3-fing3
    RCS_DEG2RAD(1.0),    // knuck1-base
    RCS_DEG2RAD(-70.0),  // fing1-knuck1
    RCS_DEG2RAD(10.0),   // tip1-fing1
    RCS_DEG2RAD(-89.0),  // fing2-knuck2
    RCS_DEG2RAD(-30.0)    // tip2-fing2 was 10
  };


  return fngr;
}

double WheelConstraint::fingersClosedAngles(size_t idx) const
{
  return fingersClosedAngles()[idx];
}

double WheelConstraint::fingersOpenedAngles(size_t idx) const
{
  return fingersOpenedAngles()[idx];
}

double WheelConstraint::fingersWideOpenedAngles(size_t idx) const
{
  return fingersWideOpenedAngles()[idx];
}

void WheelConstraint::pruneSolution(std::vector<std::vector<int>>& solutionPath)
{
  if (solutionPath.size() < 2)
  {
    return;
  }

  for (size_t i = 1; i < solutionPath.size(); ++i)
  {
    if ((solutionPath[i - 1][0] == solutionPath[i][0]) &&
        (solutionPath[i - 1][1] == solutionPath[i][1]) &&
        (solutionPath[i - 1][2] == solutionPath[i][2]) &&
        (solutionPath[i - 1][3] == solutionPath[i][3]) &&
        (solutionPath[i - 1][4] == solutionPath[i][4]))
    {
      solutionPath.erase(solutionPath.begin() + i);
      pruneSolution(solutionPath);
    }
  }

}


std::shared_ptr<tropic::ConstraintSet> WheelConstraint::createInitMove(double ttc)
{
  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  moveSet->add(std::make_shared<EngageHands>(ttc, 0.1, constraintRoboRight, constraintRoboLeft));
  moveSet->add(std::make_shared<MoveFingers>(ttc, fingersOpenedAngles(), fingersR, fingersL));

  if (constraintRoboSideways != NULL)
  {
    for (size_t i = 0; i < constraintRoboSideways->getInternalDim(); ++i)
    {
      moveSet->add(ttc, 0.0, constraintRoboSideways->getTrajectory1D(i)->getName());
    }
  }

  return moveSet;
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::createEngageMove(double t_goal, double distance)
{
  auto moveSet = std::make_shared<tropic::ConstraintSet>();

  moveSet->add(std::make_shared<EngageHands>(t_goal, distance, constraintRoboRight, constraintRoboLeft));

  return moveSet;
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::createFingersOpenMove(double t_goal)
{
  return std::make_shared<MoveFingers>(t_goal, fingersOpenedAngles(), fingersR, fingersL);
}

std::shared_ptr<tropic::ConstraintSet> WheelConstraint::createFingersClosedMove(double t_goal)
{
  return std::make_shared<MoveFingers>(t_goal, fingersClosedAngles(), fingersR, fingersL);
}

bool WheelConstraint::fingersClosed() const
{
  return leftFingersClosed() && rightFingersClosed();
}

bool WheelConstraint::fingersOpen() const
{
  return !fingersClosed();
}

bool WheelConstraint::leftFingersClosed() const
{
  return fingersClosed(this->fingersL);
}

bool WheelConstraint::rightFingersClosed() const
{
  return fingersClosed(this->fingersR);
}

bool WheelConstraint::leftFingersOpen() const
{
  return !fingersClosed(this->fingersL);
}

bool WheelConstraint::rightFingersOpen() const
{
  return !fingersClosed(this->fingersR);
}

bool WheelConstraint::fingersClosed(const tropic::TrajectoryND* fingerTraj) const
{
  if (fingerTraj == NULL)
  {
    return true;
  }

  double* q_curr = new double[fingerTraj->getInternalDim()];

  fingerTraj->getPosition(0.0, q_curr);
  VecNd_subSelf(q_curr, fingersClosedAngles(), fingerTraj->getInternalDim());

  bool closed = false;

  if (VecNd_maxAbsEle(q_curr, fingerTraj->getInternalDim()) < RCS_DEG2RAD(1.0))
  {
    closed = true;
  }

  delete [] q_curr;

  return closed;
}

bool WheelConstraint::isEngaged() const
{
  return isEngaged(constraintRoboRight) && isEngaged(constraintRoboLeft);
}

bool WheelConstraint::isEngaged(const tropic::TrajectoryND* constraintTraj) const
{
  if (constraintTraj==NULL)
  {
    return true;
  }

  bool engaged = false;
  double* q_curr = new double[constraintTraj->getInternalDim()];
  constraintTraj->getPosition(0.0, q_curr);

  if (VecNd_maxAbsEle(q_curr, constraintTraj->getInternalDim()) < 0.001)
  {
    engaged = true;
  }

  delete [] q_curr;

  return engaged;
}




}   // namespace Dc
