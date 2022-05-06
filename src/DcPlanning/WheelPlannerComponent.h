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

#ifndef RCS_WHEELPLANNERCOMPONENT_H
#define RCS_WHEELPLANNERCOMPONENT_H


#include "ComponentBase.h"
#include "WheelConstraint.h"
#include "WheelStrategy7D.h"

#include <TrajectoryController.h>


namespace Rcs
{

class WheelPlannerComponent : public ComponentBase
{
public:

  WheelPlannerComponent(EntityBase* parent,
                        const ControllerBase* controller,
                        bool viaPtTrj=true,
                        double horizon=1.0);

  virtual ~WheelPlannerComponent();
  std::string getName() const;
  const MatNd* getActivationPtr() const;
  const MatNd* getTaskCommandPtr() const;
  double getMotionEndTime() const;
  std::vector<int> getState() const;
  std::vector<double> getComplianceWrench() const;
  const WheelStrategy7D* getStrategy() const;
  std::vector<double> getTrajectoryVel(const std::string& trajname) const;

private:

  void print() const;
  void stepTrajectory(RcsGraph* from);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onClearTrajectory();
  void plan(std::vector<int> from, std::vector<int> to, double ttc);
  void planAndExecute(std::vector<int> from, std::vector<int> to, double ttc);
  void onPlantoAngleAndExecute(int s, int flip);
  void onMoveToState(std::string to, double ttc);
  void onMoveTo(std::vector<int> state, double ttc);
  void moveToStartPose(double ttc);
  void onInitializeMovement(double ttc);
  void onOpenFingers(double ttc);
  void onCloseFingers(double ttc);
  void onEngage(double ttc);
  void onDisengage(double ttc);
  void executePlan(std::vector<std::vector<int> > solutionPath, double ttc);
  void plannerThread(std::vector<int> from, std::vector<int> to, bool execute,
                     double ttc);

  tropic::TrajectoryControllerBase* tc;
  WheelConstraint* tSet;
  double motionEndTime;
  MatNd* a_des;
  MatNd* x_des;
  bool eStop;
  WheelStrategy7D wheelExplorer;
  std::vector<int> searchState;

  WheelPlannerComponent(const WheelPlannerComponent&);
  WheelPlannerComponent& operator=(const WheelPlannerComponent&);
};

}

#endif   // RCS_WHEELPLANNERCOMPONENT_H
