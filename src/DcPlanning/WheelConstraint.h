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

#ifndef RCS_WHEELCONSTRAINT_H
#define RCS_WHEELCONSTRAINT_H

#include "EventSystem.h"

#include <TrajectoryController.h>

#include <functional>


namespace Rcs
{

class WheelStrategy7D;

class NotifyConstraintSet: public tropic::ConstraintSet
{
public:
  NotifyConstraintSet(double timeToFire, std::string notifyerName, std::function<void()> cb);

  virtual void clear();

  virtual double compute(double dt);

  virtual double getStartTime() const;

  virtual double getEndTime() const;

  virtual void shiftTime(double dt);

  std::string getClassName() const;

private:
  double time;
  std::function<void()> callback;
};


class WheelConstraint

{
public:

  WheelConstraint(tropic::TrajectoryControllerBase* tc, const WheelStrategy7D* st, ES::EventSystem* es = nullptr);
  virtual ~WheelConstraint();

  std::shared_ptr<tropic::ConstraintSet> createEngageMove(double ttc, double distance);
  std::shared_ptr<tropic::ConstraintSet> createInitMove(double ttc);
  std::shared_ptr<tropic::ConstraintSet> createFingersOpenMove(double t_goal);
  std::shared_ptr<tropic::ConstraintSet> createFingersClosedMove(double t_goal);
  std::shared_ptr<tropic::ConstraintSet> moveTo(std::vector<int> state, double t_goal);
  std::shared_ptr<tropic::ConstraintSet> addSolution(std::vector<std::vector<int>> solution, double ttc);



  void pruneSolution(std::vector<std::vector<int>>& solutionPath);
  bool fingersClosed() const;
  bool fingersOpen() const;
  bool leftFingersClosed() const;
  bool rightFingersClosed() const;
  bool leftFingersOpen() const;
  bool rightFingersOpen() const;
  bool isEngaged() const;

private:

  std::shared_ptr<tropic::ConstraintSet> moveFromTo(std::vector<int> from, std::vector<int> to, double t_start,
                                                    double ttc);
  const double* fingersClosedAngles() const;
  const double* fingersOpenedAngles() const;
  const double* fingersWideOpenedAngles() const;
  double fingersClosedAngles(size_t idx) const;
  double fingersOpenedAngles(size_t idx) const;
  double fingersWideOpenedAngles(size_t idx) const;
  bool fingersClosed(const tropic::TrajectoryND* fingerTraj) const;
  bool isEngaged(const tropic::TrajectoryND* constraintTraj) const;


  tropic::TrajectoryND* trajWheel;
  tropic::TrajectoryND* trajRoboRight;
  tropic::TrajectoryND* trajRoboLeft;
  tropic::TrajectoryND* constraintRoboRight;
  tropic::TrajectoryND* constraintRoboLeft;
  tropic::TrajectoryND* constraintRoboSideways;
  tropic::TrajectoryND* fingersR;
  tropic::TrajectoryND* fingersL;

  const WheelStrategy7D* strategy;
  ES::EventSystem* eventSystem;
};


}   // namespace Rcs

#endif   // RCS_WHEELCONSTRAINT_H
