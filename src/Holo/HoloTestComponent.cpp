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

#include "HoloTestComponent.h"
#include "EntityBase.h"

#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_VecNd.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>


namespace Rcs
{

HoloTestComponent::HoloTestComponent(EntityBase* parent,
                                     const ControllerBase* controller,
                                     unsigned int port_broadcast,
                                     unsigned int port_receive) :
  HoloComponent(parent, "All", port_broadcast, port_receive),
  controller(*controller),
  showTrajectory(false),
  robotLeftHandTrajectory(nullptr),
  robotRightHandTrajectory(nullptr),
  partnerLeftHandTrajectory(nullptr),
  partnerRightHandTrajectory(nullptr),
  robotLeftHandTrajectoryArray(nullptr),
  robotRightHandTrajectoryArray(nullptr),
  partnerLeftHandTrajectoryArray(nullptr),
  partnerRightHandTrajectoryArray(nullptr)

{
  setClassName("HoloTestComponent");

  getEntity()->subscribe("Start", &HoloTestComponent::onStart, this);
  getEntity()->subscribe("MotionState", &HoloTestComponent::onMotionState, this);
  getEntity()->subscribe("ToggleTrajectory", &HoloTestComponent::onToggleTrajectories, this);
  getEntity()->subscribe("SetJointCommand", &HoloTestComponent::onForwardKinematics, this);

  /// TEMPORARILY HERE
  onTrackTransform("IME_VuforiaMarker", GraphType::Current | GraphType::Desired, "All");
  onTrackTransform("PowerGrasp_R", GraphType::Desired);
  onTrackTransform("PowerGrasp_L", GraphType::Desired);
  onTrackTransform("Box_real", GraphType::Desired);
  onTrackTransform("Box_v", GraphType::Desired);
  onTrackTransform("Box_real_center", GraphType::Desired);
  onTrackTransform("ImetronPlatform", GraphType::Desired);
  onTrackTransform("RailBot", GraphType::Desired);
  onTrackTransform("sdh-base_L", GraphType::Desired);
  onTrackTransform("sdh-base_R", GraphType::Desired);
  onTrackTransform("lbr_link_0_L", GraphType::Desired);
  onTrackTransform("lbr_link_0_R", GraphType::Desired);
  onTrackTransform("lbr_link_7_L", GraphType::Desired);
  onTrackTransform("lbr_link_7_R", GraphType::Desired);
}

HoloTestComponent::~HoloTestComponent()
{
  stop();

  MatNd_destroy(this->robotLeftHandTrajectoryArray);
  MatNd_destroy(this->robotRightHandTrajectoryArray);
  MatNd_destroy(this->partnerLeftHandTrajectoryArray);
  MatNd_destroy(this->partnerRightHandTrajectoryArray);

}

void HoloTestComponent::onStart()
{
  RLOG(0, "Starting HoloTestComponent!");
  getEntity()->publish<std::string, ViaPointTrajectoryPosition**>("ProvideTrajectory", "XYZ_L",
                                                                  &robotLeftHandTrajectory);
  getEntity()->publish<std::string, ViaPointTrajectoryPosition**>("ProvideTrajectory", "XYZ_R",
                                                                  &robotRightHandTrajectory);
  getEntity()->publish<std::string, ViaPointTrajectoryPosition**>("ProvideTrajectory", "Partner_XYZ_L",
                                                                  &partnerLeftHandTrajectory);
  getEntity()->publish<std::string, ViaPointTrajectoryPosition**>("ProvideTrajectory", "Partner_XYZ_R",
                                                                  &partnerRightHandTrajectory);
}

void HoloTestComponent::onForwardKinematics(const MatNd* q_des)
{

  // Copy the kinematic state (comprised in the vector q_des) into this classes
  // kinematic graph. The forward kinematics on the graph is calculated in the
  // callback() function, which is running concurrently with this function. We
  // therefore use the mutex.
  mtx.lock();
  MatNd_copy(controller.getGraph()->q, q_des);
  mtx.unlock();

  // If the trajectory of the left hand has been found, we compute an array
  // from time 0 until the last valid time of the trajectory, with a sampling
  // of dt.
  if (robotLeftHandTrajectory)
  {
    mtx.lock();
    ViaPointTrajectoryPosition traj(*robotLeftHandTrajectory);
    mtx.unlock();

    for (size_t i = 0; i < traj.getInternalDim(); ++i)
    {
      ViaPointTrajectory1D *viaTraj = dynamic_cast<ViaPointTrajectory1D *>(traj[i]);
      viaTraj->getViaSequence()->setTurboMode(false);
    }

    const double t_goal = 5.0;//traj.getTimeOfFirstGoal();
    const double dt = 0.05;
    traj.computeTrajectory(robotLeftHandTrajectoryArray, 0.0, t_goal, dt);
  }
  if (robotRightHandTrajectory)
  {
    mtx.lock();
    ViaPointTrajectoryPosition traj(*robotRightHandTrajectory);
    mtx.unlock();

    for (size_t i=0; i<traj.getInternalDim(); ++i)
    {
      ViaPointTrajectory1D* viaTraj = dynamic_cast<ViaPointTrajectory1D*>(traj[i]);
      viaTraj->getViaSequence()->setTurboMode(false);
    }

    const double t_goal = 5.0;//traj.getTimeOfFirstGoal();
    const double dt = 0.05;
    traj.computeTrajectory(robotRightHandTrajectoryArray, 0.0, t_goal, dt);
  }

  if (partnerLeftHandTrajectory)
  {
    mtx.lock();
    ViaPointTrajectoryPosition traj(*partnerLeftHandTrajectory);
    mtx.unlock();

    for (size_t i=0; i<traj.getInternalDim(); ++i)
    {
      ViaPointTrajectory1D* viaTraj = dynamic_cast<ViaPointTrajectory1D*>(traj[i]);
      viaTraj->getViaSequence()->setTurboMode(false);
    }

    const double t_goal = 5.0;//traj.getTimeOfFirstGoal();
    const double dt = 0.05;
    traj.computeTrajectory(partnerLeftHandTrajectoryArray, 0.0, t_goal, dt);
  }
  if (partnerRightHandTrajectory)
  {
    mtx.lock();
    ViaPointTrajectoryPosition traj(*partnerRightHandTrajectory);
    mtx.unlock();

    for (size_t i=0; i<traj.getInternalDim(); ++i)
    {
      ViaPointTrajectory1D* viaTraj = dynamic_cast<ViaPointTrajectory1D*>(traj[i]);
      viaTraj->getViaSequence()->setTurboMode(false);
    }

    const double t_goal = 5.0;//traj.getTimeOfFirstGoal();
    const double dt = 0.05;
    traj.computeTrajectory(partnerRightHandTrajectoryArray, 0.0, t_goal, dt);
  }
}


void HoloTestComponent::onMotionState(bool isMoving)
{
  static double toggleDir = 1.0;

  if (isMoving == false)
  {
    RLOG(0, "Trajectory finished");
    getEntity()->publish("PlanToAngleAndExecute", toggleDir*1.57, 3.0);
    toggleDir *= -1;
  }

}

void HoloTestComponent::onToggleTrajectories()
{
  showTrajectory = !showTrajectory;
  RLOG(0, "%s trajectories", showTrajectory ? "Showing" : "Hiding");
}

// This callback function gets called after the start() method has been called.
// It is repeated with the given update frequency. In such a thread, the
// communication with the HoloLens could for instance be made.
void HoloTestComponent::callback()
{
  HoloComponent::callback();

  Sphere sp("PowerGrasp_R_sphere", "PowerGrasp_R", {255, 128, 64}, 0.05);
  update(sp, GraphType::Desired);

  sp.parent = "PowerGrasp_L";
  sp.name = "PowerGrasp_L_sphere";
  update(sp, GraphType::Desired);

  auto bdy = RcsGraph_getBodyByName(controller.getGraph(), "Box_real");
  RCHECK(bdy);
  RCHECK(RcsBody_numShapes > 0  && bdy->shape[0]->type == RCSSHAPE_BOX);

  Box box("Box_real_BOX", "Box_real_center", {64, 160, 220, 128}, bdy->shape[0]->extents);
  update(box, GraphType::Desired);

  // Compute forward kinematics: updates all transformations
  mtx.lock();
  RcsGraph_setState(controller.getGraph(), NULL, NULL);
  mtx.unlock();

  PosedModel sdh_L;
  sdh_L.parent = "sdh-base_L";
  sdh_L.name = "LeftHandPoseable";
  sdh_L.setPose("SDH", controller.getGraph()->q, RcsGraph_getJointByName(controller.getGraph(),
                                                                         "knuck3-base_L")->jointIndex, 8);
  update(sdh_L, GraphType::Desired);

  PosedModel sdh_R;
  sdh_R.parent = "sdh-base_R";
  sdh_R.name = "RightHandPoseable";
  sdh_R.setPose("SDH", controller.getGraph()->q, RcsGraph_getJointByName(controller.getGraph(),
                                                                         "knuck3-base_R")->jointIndex, 8);
  update(sdh_R, GraphType::Desired);

  PosedModel lbr_L;
  lbr_L.parent = "lbr_link_0_L";
  lbr_L.name = "LBR_LEFT";
  lbr_L.setPose("KukaLBR", controller.getGraph()->q, RcsGraph_getJointByName(controller.getGraph(),
                                                                             "lbr_joint_1_L")->jointIndex, 7);
  update(lbr_L, GraphType::Desired);

  PosedModel lbr_R;
  lbr_L.parent = "lbr_link_0_R";
  lbr_L.name = "LBR_RIGHT";
  lbr_L.setPose("KukaLBR", controller.getGraph()->q, RcsGraph_getJointByName(controller.getGraph(),
                                                                             "lbr_joint_1_R")->jointIndex, 7);
  update(lbr_L, GraphType::Desired);
//  update(robot, GraphType::Desired);
//  robot.setPose("HondaRobot", "RightArm", controller.getGraph()->q, RcsGraph_getJointByName(controller.getGraph(),
  //"lbr_joint_1_R")->jointIndex, 7);
//  update(robot, GraphType::Desired);

  {
    static int count = 0;
    count++;

    if (count%6==0)
    {

      if (this->showTrajectory)
      {
        // Mutex needed since trajectory calculation takes place in process thread.
        mtx.lock();
        TrajectoryPoints trajectory;
        trajectory.name = "LeftHandTrajectory(des)";
        trajectory.parent = "Box_v";
        trajectory.setPoints(robotLeftHandTrajectoryArray);
        update(trajectory, GraphType::Desired);

        trajectory.name = "RightHandTrajectory(des)";
        trajectory.setPoints(robotRightHandTrajectoryArray);
        update(trajectory, GraphType::Desired);
        mtx.unlock();
      }
      else
      {
        clear("LeftHandTrajectory", "Trajectories", GraphType::Desired);
        clear("RightHandTrajectory", "Trajectories", GraphType::Desired);
      }
    }

  }
}





}   // namespace Rcs
