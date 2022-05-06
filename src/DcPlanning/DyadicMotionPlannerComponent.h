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

#ifndef RCS_DYADICMOTIONPLANNERCOMPONENT_H
#define RCS_DYADICMOTIONPLANNERCOMPONENT_H


#include "DyadicTrajectorySet.h"
#include "BoxStrategy5D.h"

#include <ComponentBase.h>
#include <TrajectoryController.h>

#include <vector>
#include <cmath>



namespace Rcs
{
/*! \brief Motion planning and trajectory generation class
 *
 *         Subscribe to the following events:
 *
 *         - "PlanAndExecute": <std::vector<int>,std::vector<int>, double>
 *           arg 1 is the 5-d start state, arg 2 is the 5-d goal state, arg3 is
 *           the desired ttc. If start and goal state are valid states, the
 *           event launches a thread to search the solution. If it is found,
 *           the "ExecutePlan" event is called with the valid solution.
 *           \todo: Is this still used somewhere?
 *
 *         - "PlanToAngleAndExecute": <double, double>
 *           arg 1 is the goal object rotation angle in radians, arg 2 is the
 *           desired ttc. If there are no constraints on the trajectories, the
 *           event launches a thread to search the solution. If it is found,
 *           the "ExecutePlan" event is called with the valid solution.
 *         -
 */
class DyadicMotionPlannerComponent : public ComponentBase
{
public:

  DyadicMotionPlannerComponent(EntityBase* parent,
                               const ControllerBase* controller,
                               double deltaPhi=30.0*M_PI/180.0,
                               bool viaPtTrj=true,
                               double horizon=1.0);
  virtual ~DyadicMotionPlannerComponent();
  std::string getName() const;
  std::vector<std::vector<int>> planSynchronous(std::vector<int> from,
                                                std::vector<int> to);
  void plan(std::vector<int> from, std::vector<int> to, double ttc);
  void planToAngle(double phi, double ttc);// \todo: Still needed?
  void planAndExecute(std::vector<int> from, std::vector<int> to, double ttc);
  void planToAngleAndExecute(double phi, double ttc);

  void computeControl(MatNd* a, MatNd* x);
  void moveToStartPose(double ttc);
  void moveTo(std::vector<int> to, double ttc);
  void moveRobotTo(int phi, int ra, int la, double ttc);
  void executePlan(std::vector<std::vector<int> > solutionPath, double ttc);
  const ControllerBase* getController() const;
  void stopTrajectory(double dt);
  double getDeltaPhi() const;
  double getNextCanonicalAngle(double startAngle, int direction) const;
  int getNumPhiDiscretizations() const;
  HTr getRoboHandPose(unsigned int idx) const;
  bool isStateValid(std::vector<int> state) const;
  std::vector<int> getCurrentState() const;
  void stopMove(double deltaTime);
  const MatNd* getActivationPtr() const;
  const MatNd* getTaskCommandPtr() const;
  std::vector<int> getStableHandSupports(int phi) const;
  std::shared_ptr<tropic::ConstraintSet> createExampleTrajectory();

private:

  void print() const;
  void stepTrajectory(RcsGraph* from);
  void onStopAtNextGoal();
  void onEmergencyStop();
  void onEmergencyRecover();
  void onInitFromState(const RcsGraph* target);
  void onPlanFromTo(std::string from, std::string to);
  void onPlanToStateAndExecute(std::string to, double ttc);
  void onToggleSlideMode();
  void onProvideTrajectoryPtr(std::string taskName,
                              tropic::ViaPointTrajectoryPosition** dst);
  void onSelectObject(std::string objectName);
  void onClearTrajectory();
  void onMoveToState(std::string to, double ttc);

  void plannerThread(std::vector<int> from,
                     std::vector<int> to,
                     bool execute,
                     double ttc);

  tropic::TrajectoryControllerBase* tc;
  DyadicTrajectorySet* tSet;
  BoxStrategy5D explorer;
  double motionEndTime;
  double lastMotionEndTime;
  MatNd* a_des;
  MatNd* x_des;
  bool eStop;

  DyadicMotionPlannerComponent(const DyadicMotionPlannerComponent&);
  DyadicMotionPlannerComponent& operator=(const DyadicMotionPlannerComponent&);
};

}

#endif   // RCS_DYADICMOTIONPLANNERCOMPONENT_H
