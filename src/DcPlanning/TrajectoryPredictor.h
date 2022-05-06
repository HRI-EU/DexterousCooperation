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

#ifndef RCS_TRAJECTORYPREDICTOR_H
#define RCS_TRAJECTORYPREDICTOR_H

#include <TrajectoryController.h>
#include <IkSolverRMR.h>


// \todo: Merge with retro

namespace Rcs
{
class TrajectoryPredictor
{
public:

  /*! \brief Constructs class with TrajectoryController instance cloned from
   *         the passed controller.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] controller Controller with all task variables and (possibly)
   *                       a collision model. It will be cloned.
   * \param[in] viaPtTrj   True for via-point polynomial trajectory, false for
   *                       zig-zag trajectory.
   * \param[in] horizon    Trajectory receding horizon length.
   */
  TrajectoryPredictor(const tropic::TrajectoryControllerBase* controller);

  /*! \brief Destroys the instance and frees all internal memory.
   */
  virtual ~TrajectoryPredictor();

  bool predict(double dt);
  void setTrajectory(tropic::TCS_sptr tSet);
  void initFromState(const MatNd* q, const MatNd* q_dot=NULL);

  //private:

  bool check(bool jointLimits=true, bool collisions=true,
             bool speedLimits=true, bool singularIK=true);
  double stepTrajectory(double dt);
  void clearTrajectory();

  tropic::TrajectoryControllerBase* tc;
  IkSolverRMR* ikSolver;
  int predSteps;
  MatNd* qStack;
  double det;

  TrajectoryPredictor(const TrajectoryPredictor&);
  TrajectoryPredictor& operator=(const TrajectoryPredictor&);
};

}

#endif   // RCS_TRAJECTORYPREDICTOR_H
