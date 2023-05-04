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

#include "TrajectoryPredictor.h"

#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>



namespace Dc
{

TrajectoryPredictor::TrajectoryPredictor(const tropic::TrajectoryControllerBase* tc_) :
  tc(NULL), ikSolver(NULL), predSteps(0), qStack(NULL), det(0.0)
{
  this->tc = new tropic::TrajectoryControllerBase(*tc_);
  this->ikSolver = new Rcs::IkSolverConstraintRMR(tc->getInternalController());
  this->qStack = MatNd_create(1, tc->getController()->getGraph()->dof);
}

TrajectoryPredictor::~TrajectoryPredictor()
{
  delete this->tc;
  delete this->ikSolver;
  MatNd_destroy(this->qStack);
}

bool TrajectoryPredictor::predict(double dt)
{
  int count = 0;
  double t_calc = Timer_getTime();
  double endTime = 1.0;
  double t = 0.0;

  // Resize the array of q-vectors
  this->qStack->m = 0;

  // We need to check the current state, otherwise we don't have any
  // information about it.
  RcsGraph_setState(tc->getInternalController()->getGraph(), NULL, NULL);
  MatNd q_row = MatNd_fromPtr(1, qStack->n,
                              tc->getController()->getGraph()->q->ele);
  MatNd_appendRows(this->qStack, &q_row);

  // Update collision model and checking
  tc->getInternalController()->computeCollisionModel();

  RcsCollisionMdl* cmdl = tc->getInternalController()->getCollisionMdl();
  int minDistPair = 0;
  double minDist = RcsCollisionMdl_getMinDist(cmdl);



  if (check(true, true, true, false)==false)   // No IK check at this point
  {
    RLOG(0, "Initial state is invalid");
    return false;
  }


  // Simulate the whole trajectory
  while (endTime > TRAJECTORY1D_ALMOST_ZERO)
  {
    endTime = stepTrajectory(dt);

    int min_i;
    double dist_i = RcsCollisionMdl_getMinDistPair(cmdl, &min_i);

    if (dist_i < minDist)
    {
      minDist = dist_i;
      minDistPair = min_i;

      RLOG(1, "minDist[%d] = %f (%s %s)", count, minDist,
           RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b1),
           RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b2));
    }


    MatNd q_row = MatNd_fromPtr(1, qStack->n,
                                tc->getController()->getGraph()->q->ele);
    MatNd_appendRows(this->qStack, &q_row);

    if (check()==false)
    {
      RLOG(0, "Invalid state at t=%f", t);
      return false;
    }

    t += dt;
    count++;
  }

  t_calc = Timer_getTime() - t_calc;

  RLOG(0, "Trajectory is fine after %d (%d) steps, took %.3f msec",
       count, qStack->m, 1.0*t_calc);
  return true;
}

double TrajectoryPredictor::stepTrajectory(double dt)
{
  double alpha = 0.05;//HACK TODO
  Rcs::ControllerBase* controller = tc->getInternalController();

  MatNd* a_des = MatNd_create(tc->getController()->getNumberOfTasks(), 1);
  MatNd* x_des = MatNd_create(tc->getController()->getTaskDim(), 1);
  double motionEndTime = tc->step(dt);
  tc->getPosition(0.0, x_des);
  tc->getActivation(a_des);

  double blending = tc->computeBlending();

  const double lambda = 1.0e-6;
  MatNd* dx_des = MatNd_create(tc->getController()->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, tc->getController()->getGraph()->nJ);
  MatNd* dH_ca = MatNd_create(1, tc->getController()->getGraph()->nJ);
  MatNd* dq_des = MatNd_create(tc->getController()->getGraph()->dof, 1);

  // Compute null space gradients
  controller->computeCollisionModel();
  controller->getCollisionGradient(dH_ca);
  RcsGraph_limitJointSpeeds(controller->getGraph(), dH_ca, dt, RcsStateIK);

  // Inverse kinematics
  controller->computeDX(dx_des, x_des);
  controller->computeJointlimitGradient(dH);
  MatNd_addSelf(dH, dH_ca);
  MatNd_constMulSelf(dH, blending*alpha);
  this->det = ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);

  double scale = RcsGraph_limitJointSpeeds(controller->getGraph(), dq_des,
                                           dt, RcsStateFull);

  if (scale<1.0)
  {
    RLOG(4, "Scaling down joint speeds by factor %f", scale);
    MatNd_constMulSelf(dq_des, 0.99999);
  }

  MatNd_addSelf(controller->getGraph()->q, dq_des);

  // Forward kinematics including velocities
  MatNd_constMulSelf(dq_des, 1.0/dt);
  RcsGraph_setState(controller->getGraph(), NULL, dq_des);

  // Update collision model and checking
  controller->computeCollisionModel();

  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dH_ca);
  MatNd_destroy(dq_des);
  MatNd_destroy(a_des);
  MatNd_destroy(x_des);

  return motionEndTime;
}

void TrajectoryPredictor::clearTrajectory()
{
  tc->clear();
}

void TrajectoryPredictor::initFromState(const MatNd* q, const MatNd* q_dot)
{
  RcsGraph_setState(tc->getInternalController()->getGraph(), q, q_dot);
  tc->clear();
  tc->init();
}

void TrajectoryPredictor::setTrajectory(tropic::TCS_sptr tSet)
{
  auto copyOfSet = tSet->clone();
  RCHECK(*copyOfSet==*(tSet.get()));
  tc->clear();
  tc->add(std::shared_ptr<tropic::ConstraintSet>(copyOfSet));
  copyOfSet->apply(tc->getTrajectoriesRef());
}

bool TrajectoryPredictor::check(bool jointLimitCheck, bool collisionCheck,
                                bool speedLimitCheck, bool singularIK)
{
  bool success = tc->getInternalController()->checkLimits(jointLimitCheck,
                                                          collisionCheck,
                                                          speedLimitCheck,
                                                          0.0, 0.0, 0.01);

  if (success==false)
  {
    RLOG(1, "limit violation");
    return false;
  }

  if (singularIK && this->det==0.0)
  {
    RLOG(1, "singular IK");
    return false;
  }

  return true;
}

}   // namespace Rcs
