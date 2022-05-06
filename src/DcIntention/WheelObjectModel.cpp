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

#include "WheelObjectModel.h"

#include <Rcs_VecNd.h>
#include <Rcs_basicMath.h>
#include <Rcs_geometry.h>
#include <Rcs_kinematics.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>

#include "ActionProgressState.h"
#include "ActionProgressTransition.h"


#define TURN_BOTH_DIRECTIONS


namespace Rcs
{

WheelObjectModel::WheelObjectModel(const ControllerBase* controller_,
                                   EntityBase* entity_) :
  controller(*controller_),
  entity(entity_)
{
  wheelExplorer = std::make_shared<WheelStrategy7D>(controller.getGraph());

  initCanonicalStates();

  // complianceHigh.resize(12, 0.0);
  // complianceHigh[0] = 300.0;
  // complianceHigh[1] = 300.0;
  // complianceHigh[2] = 900.0;
  // complianceHigh[3] = 30.0;
  // complianceHigh[4] = 30.0;
  // complianceHigh[5] = 30.0;


  // complianceLow.resize(12, 0.0);
  // complianceLow[0] = 5000.0;
  // complianceLow[1] = 5000.0;
  // complianceLow[2] = 5000.0;
  // complianceLow[3] = 300.0;
  // complianceLow[4] = 300.0;
  // complianceLow[5] = 300.0;

  // for (int i = 0; i < 6; i++)
  // {
  //   complianceHigh[i+6] = complianceHigh[i];
  //   complianceLow[i+6]  = complianceLow[i];
  // }
  // complianceVelocityMax = 0.08;
}

WheelObjectModel::~WheelObjectModel()
{
}

int WheelObjectModel::getStateDimension() const
{
  RCHECK(wheelExplorer);
  return wheelExplorer->StateMaxIndex;
}

bool WheelObjectModel::isStateValid(std::vector<int> state) const
{
  bool valid = wheelExplorer->checkState(state);
  RLOG(0, "Checking state: %d %d --> %s",
       state[0], state[1], valid ? "valid" : "invalid");
  return valid;
}

std::vector<int> WheelObjectModel::getEmptyState()
{
  std::vector<int> state(getStateDimension(), 0);
  return state;
}

std::vector<int> WheelObjectModel::getCurrentState(const RcsGraph* graph)
{
  return wheelExplorer->getState(graph);
}

std::vector<int> WheelObjectModel::getCurrentState(std::vector< std::vector<double> >& offsets, const RcsGraph* graph)
{
  //TODO: for now we just use the method in WheelStrategy7D to not break TestWheel.
  return getCurrentState(graph);
}

std::vector<int> WheelObjectModel::getGoalFromIntention(std::vector<int> currentState, int intention_id) const
{
  std::vector<int> nextGoalState(currentState);
  //  std::vector<int> currentClosestCanonicalState = getClosestCanonicalState(currentState);
  //TODO: to be done

  if (intention_id >= 0 && intention_id < (int) Intention::NUM_INTENTIONS)
  {
    RLOG(0, "Received intention '%s' (%d)", intentionName.at(intention_id).c_str(), intention_id);
  }
  else
  {
    RLOG(0, "Received intention with ID outside range of known intentions: %d", intention_id);
    return nextGoalState;
  }

  switch (intention_id)
  {
    case (int) Intention::MOVE_RIGHT:
    {
      if (currentState[0] == wheelExplorer->getNumLinearDiscretizations() - 1)
        //|| currentState[0] == wheelExplorer->getNumLinearDiscretizations()-2)
      {
        //        nextGoalState[0] -= 1;
        nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;
      }
      else if (currentState[0] <= 1)
      {
        RLOG(0, "Cannot move right here.");
      }
      else
      {
        nextGoalState[0] = 4;
      }

      break;
    }
    case (int) Intention::MOVE_LEFT:
    {
      // cannot move left at each endpoint
      if (currentState[0] <= 2 || currentState[0] == wheelExplorer->getNumLinearDiscretizations() - 1)
      {
        RLOG(0, "Cannot move left here.");
      }
      // everywhere else
      else if (currentState[0] >= wheelExplorer->getNumLinearDiscretizations()- 3)
      {
        if (!wheelExplorer->wheelVertical(currentState[1]))
          // % wheelExplorer->getNumFlipDiscretizations() == 1) ||
          // (currentState[1] % wheelExplorer->getNumFlipDiscretizations() == 3))
        {
          RLOG(0, "Need to rotate before you can move left here.");
        }
        else
        {
          //          nextGoalState[0] += 1; //wheelExplorer->getNumLinearDiscretizations()-1;
          nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 1;
        }
      }
      else
      {
        nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;
      }

      break;
    }
    case (int) Intention::MOVE_UP:
    {
      if (currentState[0] <= 1)
      {
        //        nextGoalState[0] += 1;
        nextGoalState[0] = 1;
      }
      else if (currentState[0] >= wheelExplorer->getNumLinearDiscretizations() - 3)
      {
        RLOG(0, "Cannot move further up here...");
      }
      else
      {
        nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;
      }

      break;
    }
    case (int) Intention::MOVE_DOWN:
    {
      if (currentState[0] == 0)
      {
        RLOG(0, "Cannot move further down here...");
      }
      else if (currentState[0] <= 2)
      {
        //        nextGoalState[0] -= 1;
        nextGoalState[0] = 0;
      }
      else if (currentState[0] >= wheelExplorer->getNumLinearDiscretizations() - 3)
      {
        RLOG(0, "Cannot move further down here...");
      }
      else
      {
        nextGoalState[0] = 1;
      }

      break;
    }
    case (int) Intention::FLIP_RIGHT:
    {
      if (currentState[0] < 1)
      {
        RLOG(0, "Cannot flip right here...");
      }
      else if (currentState[0] >= wheelExplorer->getNumLinearDiscretizations() - 2)
      {
        RLOG(0, "Cannot flip right here...");
      }
      //      else if (currentState[0] == 1)
      //      {
      //        nextGoalState[1] = -3;
      //        nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;
      //      }
      else
      {
        if ((double) currentState[0]/ (double) wheelExplorer->getNumLinearDiscretizations() < 0.5)
        {
          nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;

          while (true)
          {
            nextGoalState[1]--;

            if (wheelExplorer->wheelVertical(nextGoalState[1]))
            {
              break;
            }
          }
          RLOG(0, "Lower part: %d, %5.2f", currentState[0],
               (double) currentState[0]/(double) wheelExplorer->getNumLinearDiscretizations());
        }
        else
        {
          nextGoalState[0] = 1;

          while (true)
          {
            nextGoalState[1]--;

            if (wheelExplorer->wheelHorizontal(nextGoalState[1]))
            {
              break;
            }
          }
        }


      }

      break;
    }
    case (int) Intention::FLIP_LEFT:
    {
      if (currentState[0] == 0)
      {
        RLOG(0, "Cannot flip left here...");
      }
      else if (currentState[0] >= wheelExplorer->getNumLinearDiscretizations() - 2)
      {
        RLOG(0, "Cannot flip left here...");
      }
      //      else if (currentState[0] == 1)
      //      {
      //        nextGoalState[1] = -3;
      //        nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;
      //      }
      else
      {
        if ((double) currentState[0]/ (double) wheelExplorer->getNumLinearDiscretizations() < 0.5)
        {
          nextGoalState[0] = wheelExplorer->getNumLinearDiscretizations() - 3;

          while (true)
          {
            nextGoalState[1]++;

            if (wheelExplorer->wheelVertical(nextGoalState[1]))
            {
              break;
            }
          }
        }
        else
        {
          nextGoalState[0] = 1;

          while (true)
          {
            nextGoalState[1]++;

            if (wheelExplorer->wheelHorizontal(nextGoalState[1]))
            {
              break;
            }
          }
        }
      }


      break;
    }
    case (int) Intention::ROTATE_RIGHT:
    {
      break;
    }
    case (int) Intention::ROTATE_LEFT:
    {
      break;
    }
    default:
    {
      RLOG(0, "Unhandled intention id: %d", intention_id);
      break;
    }
  }

  return nextGoalState;
}


std::vector<int> WheelObjectModel::getClosestCanonicalState(std::vector<int> state_in) const
{
  //TODO: to be done
  //  int canonPhi = lround(getNextCanonicalAngle(state_in[0] * deltaPhi, 0) / deltaPhi);
  std::vector<int> stableState = state_in;// = getStableHandSupports(canonPhi);




  return stableState;
}

double WheelObjectModel::getTtc(std::vector<int> stateFrom, std::vector<int> stateTo, double baseTtc) const
{
  //TODO: ttc scaling is done in the WheelConstraints by using the function from the explorer
  // commenting this out to keep it compatible with box rotating. needs to be adjusted there before fixing here
  return baseTtc;
  //  return wheelExplorer->getTtc(stateFrom, stateTo, baseTtc);
}

bool WheelObjectModel::confirmationNeeded(std::vector<int> stateFrom, std::vector<int> stateTo) const
{
  unsigned int type = getTransitionType(stateFrom, stateTo);
  if ((type == WheelStrategy7D::TransitionType::HumanRegraspLeft) ||
      (type == WheelStrategy7D::TransitionType::HumanRegraspRight))
  {
    return false;
  }

  return true;
}

void WheelObjectModel::triggerConfirmationVisualization(std::vector<int> state, int id) const
{
  if (entity)
  {
    //TODO: visualization stuff
  }
}

void WheelObjectModel::triggerExecutionVisualization(std::vector<int> from, std::vector<int> to, RcsGraph* graph) const
{
}

std::map<std::string, double> WheelObjectModel::computeSensorSignals(RcsGraph* desiredGraph, RcsGraph* currentGraph)
{
  //  RLOG(0, "===============================================================");
  //  RLOG(0, "comp sensor signals");
  //  sensorData["Test"] = 1.23;
  //
  //  RCSGRAPH_TRAVERSE_BODIES(currentGraph)
  //  {
  //    RLOG(0, "Body: %s", BODY->name);
  //  }
  //  RcsGraph_setState(this->graph, currentGraph->q, NULL);
  //  std::vector<std::pair<std::string, double> > sensorSignals;

  RcsSensor* sensor = NULL;

  // retrieve joint torques for arms and endeffector forces
  //  double armJointTorques[2][7];
  //  double handNormal[2][3];
  //  double forceVectorHand[2][3];
  //  double forceVector[2][3];
  double forceVectorFTS[2][6];
  double forceVectorHandFTS[2][3];
  double normalForceTwoFinger[2];
  double forceNonGravityFTS[2];


  //////// calculate FT signals

  Vec3d_setZero(forceVectorFTS[0]);
  Vec3d_setZero(forceVectorFTS[1]);
  Vec3d_setZero(forceVectorHandFTS[0]);
  Vec3d_setZero(forceVectorHandFTS[1]);

  HTr A_SB;

  //TODO: this is currently using the passed in graph!
  RcsBody* handBody[2];
  handBody[0] = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_R");
  handBody[1] = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_L");

  if (handBody[0] && handBody[1])
  {
    // right side
    sensor = RcsGraph_getSensorByName(currentGraph, "WristLoadCellLBR_R");
    VecNd_copy(forceVectorFTS[0], sensor->rawData->ele, 6);

    // transform from sensor to world frame
    const RcsBody* mountBdy = RCSBODY_BY_ID(currentGraph, sensor->bodyId);
    const HTr* A_MB = &mountBdy->A_BI;
    const HTr* A_SM = &sensor->A_SB;
    HTr_transform(&A_SB, A_MB, A_SM);

    // care: this is just performing the rotation part to world frame
    Vec3d_transRotateSelf(&forceVectorFTS[0][0], A_SB.rot);
    Vec3d_transRotateSelf(&forceVectorFTS[0][3], A_SB.rot);

    // rotate force part to match powergrasp frame
    Vec3d_rotate(forceVectorHandFTS[0], handBody[0]->A_BI.rot, &forceVectorFTS[0][0]);


    //left side
    sensor = RcsGraph_getSensorByName(currentGraph, "WristLoadCellLBR_L");
    VecNd_copy(forceVectorFTS[1], sensor->rawData->ele, 6);

    mountBdy = RCSBODY_BY_ID(currentGraph, sensor->bodyId);
    const HTr* A_MB2 = &mountBdy->A_BI;
    const HTr* A_SM2 = &sensor->A_SB;
    HTr_transform(&A_SB, A_MB2, A_SM2);

    // this is just performing the rotation part to world frame
    Vec3d_transRotateSelf(&forceVectorFTS[1][0], A_SB.rot);
    Vec3d_transRotateSelf(&forceVectorFTS[1][3], A_SB.rot);

    // rotate force part to match powergrasp frame
    Vec3d_rotate(forceVectorHandFTS[1], handBody[1]->A_BI.rot, &forceVectorFTS[1][0]);

    normalForceTwoFinger[0] = - forceVectorHandFTS[0][1];
    normalForceTwoFinger[1] = - forceVectorHandFTS[1][1];

    forceNonGravityFTS[0] =
      sqrt(forceVectorFTS[0][0] * forceVectorFTS[0][0] + forceVectorFTS[0][1] * forceVectorFTS[0][1]);
    forceNonGravityFTS[1] =
      sqrt(forceVectorFTS[1][0] * forceVectorFTS[1][0] + forceVectorFTS[1][1] * forceVectorFTS[1][1]);

    //  RLOG(0, "[%2d requests remaining] dt: %6.4f, Normal Force: % 5.3f, % 5.3f",
    //  (int) requests.size(), dt, normalForceTwoFinger[0], normalForceTwoFinger[1]);

    // store all values in map
    sensorData["normalForceRight"] = normalForceTwoFinger[0];
    sensorData["normalForceLeft"] = normalForceTwoFinger[1];
    sensorData["forceNonGravityRight"] = forceNonGravityFTS[0];
    sensorData["forceNonGravityLeft"] = forceNonGravityFTS[1];
    sensorData["pushForce"] = fmax(- forceVectorFTS[0][0], - forceVectorFTS[1][0]);
    sensorData["normalForceDifference"] = normalForceTwoFinger[0] - normalForceTwoFinger[1];

    sensorData["forceGravityRight"] = - forceVectorFTS[0][2];
    sensorData["forceGravityLeft"] = - forceVectorFTS[1][2];

    sensorData["forceHandZRight"] = forceVectorHandFTS[0][2];
    sensorData["forceHandZLeft"] = forceVectorHandFTS[1][2];

    sensorData["torqueX_right"] = forceVectorFTS[0][3];
    sensorData["torqueY_right"] = forceVectorFTS[0][4];
    sensorData["torqueZ_right"] = forceVectorFTS[0][5];

    sensorData["torqueX_left"] = forceVectorFTS[1][3];
    sensorData["torqueY_left"] = forceVectorFTS[1][4];
    sensorData["torqueZ_left"] = forceVectorFTS[1][5];

  }

  //=======================
  // use vicon data to determine object (mis-)alignment and human hand poses
  //  const RcsBody* objBdy_cur = RcsGraph_getBodyByName(currentGraph, "Wheel");
  const RcsBody* objBdy_des = RcsGraph_getBodyByName(desiredGraph, "Wheel");

  const RcsBody* shoulderBdy_cur = RcsGraph_getBodyByName(currentGraph, "RailBot");
  const RcsBody* shoulderBdy_des = RcsGraph_getBodyByName(desiredGraph, "RailBot");


  const RcsBody* handBdy_des_R = RcsGraph_getBodyByName(desiredGraph, "PowerGrasp_R");
  const RcsBody* handBdy_des_L = RcsGraph_getBodyByName(desiredGraph, "PowerGrasp_L");

  const RcsBody* handBdy_cur_R = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_R");
  const RcsBody* handBdy_cur_L = RcsGraph_getBodyByName(currentGraph, "PowerGrasp_L");

  {
    if (objBdy_des && shoulderBdy_cur && shoulderBdy_des && handBdy_des_R &&
        handBdy_des_L && handBdy_cur_R && handBdy_cur_L)
    {
      //    const HTr* cur_A_OBJ_I = objBdy_cur->A_BI;
      const HTr* des_A_OBJ_I = &objBdy_des->A_BI;

      const HTr* cur_A_Shoulder_I = &shoulderBdy_cur->A_BI;
      const HTr* des_A_Shoulder_I = &shoulderBdy_des->A_BI;

      const HTr* des_A_RH_I = &handBdy_des_R->A_BI;
      const HTr* des_A_LH_I = &handBdy_des_L->A_BI;

      const HTr* cur_A_RH_I = &handBdy_cur_R->A_BI;
      const HTr* cur_A_LH_I = &handBdy_cur_L->A_BI;

      HTr* des_A_RH_Shoulder = HTr_create();
      HTr_invTransform(des_A_RH_Shoulder, des_A_Shoulder_I, des_A_RH_I);

      HTr* des_A_LH_Shoulder = HTr_create();
      HTr_invTransform(des_A_LH_Shoulder, des_A_Shoulder_I, des_A_LH_I);

      HTr* cur_A_RH_Shoulder = HTr_create();
      HTr_invTransform(cur_A_RH_Shoulder, cur_A_Shoulder_I, cur_A_RH_I);

      HTr* cur_A_LH_Shoulder = HTr_create();
      HTr_invTransform(cur_A_LH_Shoulder, cur_A_Shoulder_I, cur_A_LH_I);

      HTr* des_A_OBJ_Shoulder = HTr_create();
      HTr_invTransform(des_A_OBJ_Shoulder, des_A_Shoulder_I, des_A_OBJ_I);

      //TODO: measure displayment in "RailBot" frame to remove effect of IME and IEF not being precise


      // define a "virtual center" frame (vc) between the hands aligned with the object center
      HTr* des_A_vc_Shoulder = HTr_create();


      // find origin on line between both hands at the closest point to the object position
      double lineStart_des[3];
      Vec3d_copy(lineStart_des, des_A_RH_Shoulder->org);

      double lineDir_des[3];
      Vec3d_sub(lineDir_des, des_A_LH_Shoulder->org, des_A_RH_Shoulder->org);

      double virtCenter_des[3];
      Math_sqrDistPointLine(des_A_OBJ_Shoulder->org, lineStart_des, lineDir_des, virtCenter_des);

      Vec3d_copy(des_A_vc_Shoulder->org, virtCenter_des);

      // define x-axis towards object
      double unitX[3];
      Vec3d_sub(unitX, des_A_OBJ_Shoulder->org, des_A_vc_Shoulder->org);
      double distHands_des = Vec3d_getLength(unitX);
      Vec3d_normalizeSelf(unitX);


      // define y-axis from direction RH to LH
      double unitY[3];
      Vec3d_normalize(unitY, lineDir_des);

      // calculate matching z-axis
      double unitZ[3];
      Vec3d_crossProduct(unitZ, unitX, unitY);
      Vec3d_normalizeSelf(unitZ);

      for (int i = 0; i < 3; i++)
      {
        des_A_vc_Shoulder->rot[0][i] = unitX[i];
        des_A_vc_Shoulder->rot[1][i] = unitY[i];
        des_A_vc_Shoulder->rot[2][i] = unitZ[i];
      }

      HTr* cur_A_vc_Shoulder = HTr_create();

      HTr* des_A_vc_LH = HTr_create();
      HTr_invTransform(des_A_vc_LH, des_A_LH_Shoulder, des_A_vc_Shoulder);

      HTr* des_A_vc_RH = HTr_create();
      HTr_invTransform(des_A_vc_RH, des_A_RH_Shoulder, des_A_vc_Shoulder);

      HTr* delta_A_vccur_vcdes = HTr_create();

      HTr_transform(cur_A_vc_Shoulder, cur_A_LH_Shoulder, des_A_vc_LH);

      HTr_transform(cur_A_vc_Shoulder, cur_A_RH_Shoulder, des_A_vc_RH);


      RFREE(delta_A_vccur_vcdes);
      RFREE(des_A_vc_RH);
      RFREE(des_A_vc_LH);
      RFREE(cur_A_vc_Shoulder);

      double offset_R_Shoulder[3];
      Vec3d_sub(offset_R_Shoulder, des_A_RH_Shoulder->org, cur_A_RH_Shoulder->org);

      double offset_L_Shoulder[3];
      Vec3d_sub(offset_L_Shoulder, des_A_LH_Shoulder->org, cur_A_LH_Shoulder->org);

      double offset_R_I[3];
      Vec3d_rotate(offset_R_I, (double (*)[3])des_A_Shoulder_I->rot, offset_R_Shoulder);

      double offset_L_I[3];
      Vec3d_rotate(offset_L_I, (double (*)[3])des_A_Shoulder_I->rot, offset_L_Shoulder);

      //compensated offset of L if R is at zero
      double offsetRel_Shoulder[3];
      Vec3d_sub(offsetRel_Shoulder, offset_L_Shoulder, offset_R_Shoulder);

      double offsetRel_Vc[3];
      Vec3d_rotate(offsetRel_Vc, des_A_vc_Shoulder->rot, offsetRel_Shoulder);

      sensorData["roll"] = offsetRel_Vc[0] / distHands_des;
      sensorData["flip"] = offsetRel_Vc[2] / distHands_des;

      sensorData["hand_pos_R_x"] = offset_R_I[0];
      sensorData["hand_pos_R_y"] = offset_R_I[1];
      sensorData["hand_pos_R_z"] = offset_R_I[2];
      sensorData["hand_pos_L_x"] = offset_L_I[0];
      sensorData["hand_pos_L_y"] = offset_L_I[1];
      sensorData["hand_pos_L_z"] = offset_L_I[2];

      RFREE(des_A_vc_Shoulder);

      RFREE(des_A_RH_Shoulder);
      RFREE(cur_A_RH_Shoulder);
      RFREE(des_A_LH_Shoulder);
      RFREE(cur_A_LH_Shoulder);
      RFREE(des_A_OBJ_Shoulder);
    }
  }

  {
    Task* rot = controller.getTask("RobotLeftConstraint");
    RcsGraph_setState(controller.getGraph(), desiredGraph->q, NULL);
    MatNd* dq = MatNd_create(desiredGraph->dof, 1);
    MatNd_sub(dq, desiredGraph->q, currentGraph->q);
    RcsGraph_stateVectorToIKSelf(desiredGraph, dq);
    MatNd* J = MatNd_create(rot->getDim(), desiredGraph->nJ);
    rot->computeJ(J);
    MatNd* dx = MatNd_create(rot->getDim(), 1);
    MatNd_mul(dx, J, dq);

    double a[3][3];
    Vec3d_setUnitVector(a[0], 0);
    Vec3d_setUnitVector(a[1], 1);
    Vec3d_setUnitVector(a[2], 2);

    double dAngle[3];
    MatNd* J_r = MatNd_create(3, desiredGraph->nJ);
    MatNd* J_l = MatNd_create(3, desiredGraph->nJ);
    RcsBody* rhBdy = RcsGraph_getBodyByName(controller.getGraph(), "WheelGrasp_R");
    RcsBody* lhBdy = RcsGraph_getBodyByName(controller.getGraph(), "WheelGrasp_L");
    MatNd* J2 = MatNd_create(J_r->m, J_r->n);
    MatNd* dx2 = MatNd_create(1, 1);

    for (int i = 0; i < 3; i++)
    {
      double A_rel[3][3];
      Mat3d_fromVec(A_rel, a[i], 2);
      //      printf("Rot: %d\n", rot->getDim());

      RcsGraph_rotationJacobian(controller.getGraph(), rhBdy, A_rel, J_r);
      RcsGraph_rotationJacobian(controller.getGraph(), lhBdy, A_rel, J_l);

      MatNd_add(J2, J_r, J_l);
      MatNd Jz = MatNd_getRowView(J2, 2);
      MatNd_mul(dx2, &Jz, dq);
      dAngle[i] = dx2->ele[0];
    }


    MatNd_destroy(J_r);
    MatNd_destroy(J_l);
    MatNd_destroy(J2);
    MatNd_destroy(dx2);

    MatNd_destroy(dq);
    MatNd_destroy(J);
    MatNd_destroy(dx);

    sensorData["rotAngleX"] = dAngle[0];
    sensorData["rotAngleY"] = dAngle[1];
    sensorData["rotAngleZ"] = dAngle[2];
  }


  //  const RcsBody* headBdy_cur = RcsGraph_getBodyByName(currentGraph, "human_head");
  const std::string headBody = "Vicon head";
  const RcsBody* headBdy_cur = RcsGraph_getBodyByName(currentGraph, headBody.c_str());
  if (headBdy_cur)
  {
    const HTr* cur_A_head_I = &headBdy_cur->A_BI;

    //    const RcsBody* shoulderBdy_cur = RcsGraph_getBodyByName(currentGraph, "RailBot");
    const HTr* cur_A_bot_I = &shoulderBdy_cur->A_BI;

    double vec_head_bot[3];
    Vec3d_invTransform(vec_head_bot, cur_A_head_I, cur_A_bot_I->org);
    //  Vec3d_sub(vec_head_bot, cur_A_bot_I->org, cur_A_head_I->org);
    Vec3d_normalizeSelf(vec_head_bot);
    double vec_unit_x[3];
    Vec3d_set(vec_unit_x, 1.0, 0.0, 0.0);
    double gazeAngle = acos(Vec3d_innerProduct(vec_unit_x, vec_head_bot));

    sensorData["gazeError"] = gazeAngle;
  }
  else
  {
    RLOG(5, "Body '%s' not found in graph. 'gazeError' fixed to 0.0!", headBody.c_str());
    sensorData["gazeError"] = 0.0;
  }

  return sensorData;
}


/************************************************************************
 * Monitors
 ************************************************************************/

std::vector<MonitorRequest> WheelObjectModel::configureMonitorsForState(std::vector<int> state) const
{
  NLOG(0, "Requesting monitors for state %s", vecToStr(state).c_str());

  std::vector<MonitorRequest> monitors;

  double thresGravForceHigh = 8.0;
  double thresDisplacementSideMove = 0.02;
  double thresDisplacementUpMove = 0.015;
  double thresDisplacementDownMove = 0.025;
  double thresDisplacementFlipMove = 0.04;//0.05;
  double thresAngleFlipMove = 0.06; //0.08;
  double thresAngleMoveUpDown = 0.1; //0.1
  double thresAngleMoveSide = 0.07; //0.1

  //  double thresForceMovePull = 0.5;
  double thresTorqueUpMove = 0.4;
  double thresTorqueSideMove = 0.4;//0.5;
  //  double thresTranslateRoll = 0.04;

  MonitorRequest flip_right("Intention", (int) Intention::FLIP_RIGHT, 10000, "Flip right");
  flip_right.addSignal("flip", thresDisplacementFlipMove, false);
  flip_right.addSignal("rotAngleY", thresAngleFlipMove, false);

  MonitorRequest flip_left("Intention", (int) Intention::FLIP_LEFT, 10000, "Flip left");
  flip_left.addSignal("flip", -thresDisplacementFlipMove, true);
  flip_left.addSignal("rotAngleY", -thresAngleFlipMove, true);

  MonitorRequest flip_left2("Intention", (int) Intention::FLIP_LEFT, 10000, "Flip left2");
  flip_left2.addSignal("rotAngleY", -thresAngleFlipMove, true);

  MonitorRequest move_right("Intention", (int) Intention::MOVE_RIGHT, 10000, "Move right");
  move_right.addSignal("rotAngleZ", -thresAngleMoveSide, true);

  MonitorRequest move_right2("Intention", (int) Intention::MOVE_RIGHT, 10000, "Move right");
  move_right2.addSignal("hand_pos_R_x", - thresDisplacementSideMove / 2.0, true);
  move_right2.addSignal("hand_pos_L_x", - thresDisplacementSideMove / 2.0, true);
  move_right2.addSignal("torqueZ_right", thresTorqueSideMove, false);
  move_right2.addSignal("torqueZ_left", thresTorqueSideMove, false);

  MonitorRequest move_hanging_lift_left_hand_bottom("Intention", (int) Intention::MOVE_RIGHT, 10000,
                                                    "Move right (lift)");
  move_hanging_lift_left_hand_bottom.addSignal("forceNonGravityLeft", thresGravForceHigh, false);

  MonitorRequest move_left("Intention", (int) Intention::MOVE_LEFT, 10000, "Move left");
  move_left.addSignal("rotAngleZ", thresAngleMoveSide, false);

  MonitorRequest move_left2("Intention", (int) Intention::MOVE_LEFT, 10000, "Move left");
  move_left2.addSignal("hand_pos_R_x", thresDisplacementSideMove / 2.0, false);
  move_left2.addSignal("hand_pos_L_x", thresDisplacementSideMove / 2.0, false);
  move_left2.addSignal("torqueZ_right", -thresTorqueSideMove, true);
  move_left2.addSignal("torqueZ_left", -thresTorqueSideMove, true);

  MonitorRequest move_up("Intention", (int) Intention::MOVE_UP, 10000, "Move up");
  move_up.addSignal("hand_pos_R_z", thresDisplacementUpMove, true);
  move_up.addSignal("hand_pos_L_z", thresDisplacementUpMove, true);
  move_up.addSignal("torqueX_right", thresTorqueUpMove, false);
  move_up.addSignal("torqueX_left", thresTorqueUpMove, false);

  MonitorRequest move_up2("Intention", (int) Intention::MOVE_UP, 10000, "Move up");
  move_up2.addSignal("rotAngleX", -thresAngleMoveUpDown, true);
  move_up2.addSignal("torqueX_right", thresTorqueUpMove, false);
  move_up2.addSignal("torqueX_left", thresTorqueUpMove, false);

  MonitorRequest move_down("Intention", (int) Intention::MOVE_DOWN, 10000, "Move down");
  move_down.addSignal("hand_pos_R_z", thresDisplacementDownMove, false);
  move_down.addSignal("hand_pos_L_z", thresDisplacementDownMove, false);
  move_down.addSignal("torqueX_right", -thresTorqueUpMove, true);
  move_down.addSignal("torqueX_left", -thresTorqueUpMove, true);

  MonitorRequest move_down2("Intention", (int) Intention::MOVE_DOWN, 10000, "Move down");
  move_down2.addSignal("rotAngleX", thresAngleMoveUpDown, false);
  move_down2.addSignal("torqueX_right", -thresTorqueUpMove, true);
  move_down2.addSignal("torqueX_left", -thresTorqueUpMove, true);

  //on hanging mount
  if (state[0] == wheelExplorer->getNumLinearDiscretizations() - 1)
  {
    monitors.push_back(move_right);
    monitors.push_back(move_right2);
  }
  else if (state[0] == wheelExplorer->getNumLinearDiscretizations() - 2)
  {
    monitors.push_back(move_left);
    monitors.push_back(move_right);
    monitors.push_back(move_left2);
    monitors.push_back(move_right2);
  }
  // before hanging mount only allow left if already vertical
  else if (state[0] == wheelExplorer->getNumLinearDiscretizations() - 3)
  {
    if (wheelExplorer->wheelVertical(state[1]))
    {
      monitors.push_back(move_left);
      monitors.push_back(move_left2);
    }

    monitors.push_back(flip_left);

#if defined (TURN_BOTH_DIRECTIONS)
    monitors.push_back(flip_right);
#endif
  }

  else if (state[0] == 1)
  {
    if (wheelExplorer->wheelHorizontal(state[1]))
    {
      monitors.push_back(move_down2);
      monitors.push_back(move_down);
    }

    if (state[1] == 0)
    {
      monitors.push_back(flip_right);
    }
#if defined (TURN_BOTH_DIRECTIONS)
    if (state[1] == -6)
    {
      monitors.push_back(flip_left);
    }
#endif
  }

  else if (state[0] == 0)
  {
    monitors.push_back(move_up2);
    monitors.push_back(move_up);
  }
  else
  {
    NLOG(0, "Nothing special here. State: %s. Permitting all intentions.", vecToStr(state).c_str());
  }

  return monitors;
}


std::vector<MonitorRequest> WheelObjectModel::configureConfirmationMonitorsForTransition(std::vector<int> startState,
    std::vector<int> endState) const
{
  std::vector<MonitorRequest> monitors;

  RLOG(0, "Confirmation monitors requested");

  MonitorRequest mon("Confirmation", 0, 0, "confirm all");
  monitors.push_back(mon);

  return monitors;
}


std::vector<MonitorRequest> WheelObjectModel::configureMonitorsForTransition(std::vector<int> startState,
    std::vector<int> endState) const
{
  std::vector<MonitorRequest> monitors;

  unsigned int type = getTransitionType(startState, endState);

  RLOG(0, "Transition type: %d", (int) type);

  return monitors;
}




/************************************************************************
 * Action Progress Graphs
 ************************************************************************/

std::vector<Apg_ptr> WheelObjectModel::getActionProgressGraphs() const
{
  std::vector<Apg_ptr> actionProgressGraphs;

  actionProgressGraphs.push_back(createRegraspAPG(true));
  actionProgressGraphs.push_back(createRegraspAPG(false));
  actionProgressGraphs.push_back(createRollObjectAPG());
  actionProgressGraphs.push_back(createFlipObjectAPG());
  actionProgressGraphs.push_back(createMoveObjectAPG());

  return actionProgressGraphs;
}

Apg_ptr WheelObjectModel::createRegraspAPG(bool rightHandRegrasp) const
{
  std::string side = rightHandRegrasp ? "right" : "left";

  Apg_ptr tmp(new ActionProgressGraph(std::to_string(1), ("regrasp " + side)));

  //TODO: fill in regrasp graph

  return tmp;
}

Apg_ptr WheelObjectModel::createRollObjectAPG() const
{
  Apg_ptr tmp(new ActionProgressGraph(std::to_string(0), ("roll object")));

  //TODO: fill in roll graph

  return tmp;
}

Apg_ptr WheelObjectModel::createFlipObjectAPG() const
{
  Apg_ptr tmp(new ActionProgressGraph(std::to_string(0), ("flip object")));

  //TODO: fill in flip graph

  return tmp;
}

Apg_ptr WheelObjectModel::createMoveObjectAPG() const
{
  Apg_ptr tmp(new ActionProgressGraph(std::to_string(0), ("move object")));

  //TODO: fill in move graph

  return tmp;
}




/*******************************************************************************
 * Transition Types
 ******************************************************************************/

unsigned int WheelObjectModel::getTransitionType(std::vector<int> from, std::vector<int> to) const
{
  return WheelStrategy7D::getTransitionType(from, to);
}

std::string WheelObjectModel::transitionTypeToString(unsigned int type) const
{
  return wheelExplorer->transitionTypeToString(type);
}

int WheelObjectModel::getNumberOfContactChanges(std::vector<int> from, std::vector<int> to) const
{
  return WheelStrategy7D::getNumberOfContactChanges(from, to);
}

void WheelObjectModel::initCanonicalStates()
{
  canonicalStates.clear();
  canonicalStates.reserve(30);

  std::vector<int> vert(2, 0);
  vert[0] = wheelExplorer->getNumLinearDiscretizations();
  vert[1] = (wheelExplorer->getNumFlipDiscretizations() / 4);
  canonicalStates.push_back(vert);

  std::vector<int> vert2(2, 0);
  vert2[0] = wheelExplorer->getNumLinearDiscretizations();
  vert2[1] = (wheelExplorer->getNumFlipDiscretizations() / 4) * 3;
  canonicalStates.push_back(vert2);

  std::vector<int> horizontal(2, 0);
  horizontal[0] = 0;
  horizontal[1] = 0;
  canonicalStates.push_back(horizontal);

  std::vector<int> horizontal2(2, 0);
  horizontal2[0] = 0;
  horizontal2[1] = (wheelExplorer->getNumFlipDiscretizations() / 4) * 2;
  canonicalStates.push_back(horizontal2);

  for (int i = 1; i < wheelExplorer->getNumLinearDiscretizations(); i++)
  {
    for (int j = 0; j < 4; j++)
    {
      std::vector<int> tmp(2, 0);
      tmp[0] = i;
      tmp[1] = j * (wheelExplorer->getNumFlipDiscretizations() / 4);
      canonicalStates.push_back(tmp);
    }
  }

  RLOG(0, "Finished creating %d canonical states", (int) canonicalStates.size());
}

// std::vector<double> WheelObjectModel::getComplianceWrench2(const double xd_r[3],
//                                                           const double xd_l[3]) const
// {
//   double xd[3];
//   Vec3d_add(xd, xd_r, xd_l);
//   double handTaskVel = Vec3d_sqrLength(xd);

//   double ratio = (std::min)(handTaskVel, this->complianceVelocityMax) / this->complianceVelocityMax;

//   std::vector<double> complianceWrench(12, 0.0);

//   for (size_t i = 0; i < complianceWrench.size(); i++)
//   {
//     complianceWrench[i] = complianceLow[i]*ratio + complianceHigh[i]*(1.0-ratio);
//   }

//   return complianceWrench;
// }

}
