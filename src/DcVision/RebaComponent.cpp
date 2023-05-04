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

#include "RebaComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>

namespace Dc
{

RebaComponent::RebaComponent(EntityBase* parent, const char* cfgFile) :
  ComponentBase(parent), controller(cfgFile), ikSolver(&controller),
  rebaGraph(controller.getGraph()), a_des(NULL), x_des(NULL)
{
  this->a_des = MatNd_create(controller.getNumberOfTasks(), 1);
  controller.readActivationsFromXML(this->a_des);

  this->x_des = MatNd_create(controller.getTaskDim(), 1);
  controller.computeX(this->x_des);

  subscribe("ComputeTrajectory", &RebaComponent::onComputeReba);
  subscribe("Render", &RebaComponent::onRender);
}

RebaComponent::~RebaComponent()
{
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
}

void RebaComponent::onComputeReba(RcsGraph* graph)
{
  const RcsBody* handR = RcsGraph_getBodyByName(graph, "PartnerGrasp_R");
  const RcsBody* handL = RcsGraph_getBodyByName(graph, "PartnerGrasp_L");
  RCHECK(handR);
  RCHECK(handL);
  int idxR = controller.getTaskArrayIndex("Partner_HAND_R");
  int idxL = controller.getTaskArrayIndex("Partner_HAND_L");

  const double lambda = 0.01;
  const double alpha = 0.05;

  HTr_to6DVector(MatNd_getElePtr(x_des, idxR, 0), &handR->A_BI);
  HTr_to6DVector(MatNd_getElePtr(x_des, idxL, 0), &handL->A_BI);

  MatNd* dx_des = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* dH_ca = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* dq_des = MatNd_create(controller.getGraph()->dof, 1);

  // Inverse kinematics
  controller.computeDX(dx_des, x_des);
  controller.computeJointlimitGradient(dH);
  MatNd_addSelf(dH, dH_ca);
  MatNd_constMulSelf(dH, alpha);
  ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
  MatNd_addSelf(controller.getGraph()->q, dq_des);

  // Forward kinematics including velocities
  MatNd_constMulSelf(dq_des, 1.0 / getEntity()->getDt());
  RcsGraph_setState(controller.getGraph(), NULL, dq_des);

  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dH_ca);
  MatNd_destroy(dq_des);
}

void RebaComponent::onRender()
{
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph",
                                                    "Ergonomics", rebaGraph);
}


}  // namespace
