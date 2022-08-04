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

#ifndef RRCS_DYNAMICPOSEGRAPH_H
#define RCS_DYNAMICPOSEGRAPH_H

#include <TrajectoryController.h>
#include <IkSolverRMR.h>

namespace Rcs
{

class DynamicPoseGraph
{
public:
  DynamicPoseGraph(const std::string& cfgFile);
  virtual ~DynamicPoseGraph();
  void predict(MatNd* qStack, double dt, double alpha, double lambda);
  ControllerBase* createPoseSequence(std::vector<double> times);
  void addTrajectory(std::shared_ptr<tropic::ConstraintSet> tSet);
  ControllerBase* createLinks(std::vector<double> times);
  void linkBoxes(ControllerBase* controller, std::vector<double> times);
  void linkBoxesWithCoupledJoints(ControllerBase* controller, std::vector<double> times);
  void linkHands(ControllerBase* controller);
  void linkHandsWithCoupledJoints(ControllerBase* controller);
  void allowBoxRotation(ControllerBase* controller);
  void makeHandsPolar(ControllerBase* controller);
  void removePartnerTasks(ControllerBase* controller);

  //  protected:
  tropic::TrajectoryControllerBase* tc;
  IkSolverRMR* ikSolver;

private:

  /*! \brief Private assignment operator to avoid avoid memory issues when assigning
   */
  DynamicPoseGraph& operator = (const DynamicPoseGraph&);
  DynamicPoseGraph(const DynamicPoseGraph& copyFromMe);
};


}   // namespace Rcs

#endif // RCS_DYNAMICPOSEGRAPH_H
