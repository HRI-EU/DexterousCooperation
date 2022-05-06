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

#include "DynamicPoseGraph.h"

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <TaskJoints.h>
#include <TaskFactory.h>

//#define PREDICT_DEBUG
//#define SNAPSHOT_DEBUG
//#define LINK_DEBUG

#if defined (PREDICT_DEBUG) ||  defined (SNAPSHOT_DEBUG) ||  defined (LINK_DEBUG)
#include <RcsViewer.h>
#include <GraphNode.h>
#endif

using namespace tropic;

namespace Rcs
{

DynamicPoseGraph::DynamicPoseGraph(const std::string& cfgFile) :
  tc(NULL), ikSolver(NULL)
{
  const double horizon = 1.0;
  bool via = false;//true;

  if (via)
  {
    this->tc = new TrajectoryController<ViaPointTrajectory1D>(cfgFile, horizon);
  }
  else
  {
    this->tc = new TrajectoryController<ZigZagTrajectory1D>(cfgFile, horizon);
  }

  tc->readActivationsFromXML();
  tc->setTurboMode(false);
  this->ikSolver = new IkSolverRMR(tc->getInternalController());
}

DynamicPoseGraph::~DynamicPoseGraph()
{
  delete this->tc;
  delete this->ikSolver;
}

void DynamicPoseGraph::predict(MatNd* qStack, double dt, double alpha, double lambda)
{
#if defined (PREDICT_DEBUG)
  Rcs::Viewer viewer;
  viewer.add(new Rcs::GraphNode(tc->getInternalController()->getGraph()));
  viewer.runInThread();
  RPAUSE_DL(1);
#endif

  tc->step(0.0001);// \todo BOOOOH ! Fix me.
  ControllerBase* controller = tc->getInternalController();
  MatNd* q_reset = MatNd_clone(tc->getInternalController()->getGraph()->q);
  MatNd* a_des = MatNd_create(controller->getNumberOfTasks(), 1);
  tc->getActivation(a_des);
  MatNd* x_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller->getGraph()->nJ);
  MatNd* dq_des = MatNd_create(controller->getGraph()->dof, 1);

  // Resize the array of q-vectors
  const double endTime = tc->getTimeOfLastGoal();
  const size_t nSteps = lround(endTime/dt)+1;

  qStack = MatNd_realloc(qStack, nSteps, controller->getGraph()->dof);
  MatNd_reshape(qStack, 0, qStack->n);

  // We need to check the current state, otherwise we don't have any
  // information about it.
  RcsGraph_setState(controller->getGraph(), NULL, NULL);
  MatNd q_row = MatNd_fromPtr(1, controller->getGraph()->dof, controller->getGraph()->q->ele);
  MatNd_appendRows(qStack, &q_row);


  // Simulate the whole trajectory
  for (double ti = 0; ti < endTime; ti+=dt)
  {
    tc->getPosition(ti, x_des);
    controller->computeDX(dx_des, x_des);
    controller->computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha);
    ikSolver->solveRightInverse(dq_des, dx_des, dH, a_des, lambda);
    MatNd_addSelf(controller->getGraph()->q, dq_des);
    RcsGraph_setState(controller->getGraph(), NULL, NULL);
    MatNd_appendRows(qStack, &q_row);
#if defined (PREDICT_DEBUG)
    RLOG(0, "t = %.3f   row=%u   check is %s",
         ti, qStack->m, controller->checkLimits() ? "OK" : "BAD");
    RPAUSE_DL(1);
#endif
  }

  // Reset to original state
  RcsGraph_setState(controller->getGraph(), q_reset, NULL);

  MatNd_destroy(a_des);
  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dq_des);
  MatNd_destroy(x_des);
  MatNd_destroy(q_reset);
}

void DynamicPoseGraph::addTrajectory(std::shared_ptr<ConstraintSet> tSet)
{
  RLOG(1, "addTrajectory end time: %f", tSet->getEndTime());
  bool success = tc->addAndApply(tSet, true);

  if (!success)
  {
    RLOG(1, "Failed to add trajectory constraint");
  }
  else
  {
    RLOG(1, "Trajectory is %f sec long", tc->getRootSet().getEndTime());
  }
}

ControllerBase* DynamicPoseGraph::createPoseSequence(std::vector<double> times)
{
  const double dt = 0.05;
  const double alpha = 0.05;
  const double lambda = 1.0e-8;
  MatNd* qStack = MatNd_create(1,1);

  RLOG(1, "Predicting ...");
  predict(qStack, dt, alpha, lambda);
  RLOG(1, "... %d steps", qStack->m);

#if defined (SNAPSHOT_DEBUG)
  Rcs::Viewer viewer;
  viewer.add(new Rcs::GraphNode(tc->getInternalController()->getGraph()));
  viewer.runInThread();
  RPAUSE_DL(1);
#endif

  ControllerBase* source = tc->getInternalController();
  MatNd* q_init = MatNd_create(source->getGraph()->dof, 1);
  RcsGraph_getDefaultState(source->getGraph(), q_init);
  RcsGraph_changeInitState(source->getGraph(), q_init);
  RcsGraph_changeDefaultState(source->getGraph(), q_init);
  ControllerBase* controller = new ControllerBase(*source);

  HTr offset;
  HTr_setIdentity(&offset);

  for (size_t i=0; i<times.size(); ++i)
  {
#if defined (SNAPSHOT_DEBUG)
    RPAUSE_DL(1);
#endif
    size_t row = lround(times[i]/dt);
    RCHECK(row<qStack->m);
    MatNd q_i = MatNd_getRowViewTranspose(qStack, row);
    RcsGraph_setState(source->getGraph(), &q_i, NULL);

    if (i>0)
    {
      char a[16];
      snprintf(a, 16, "_%d", (int) i);
      offset.org[0] = i*(-2.5);
      controller->add(source, a, &offset);
    }

  }

  MatNd_destroy(qStack);
  MatNd_destroy(q_init);

  return controller;
}

// Constraint sequence
// 0   (0,  14,  10)
//                     LH swing
// 1   (0,  14,   7)
//                     ROT: LH, RH fixed
// 2   (3,  14,   7)
//                     RH swing
// 3   (3,   9,   7)
//                     LH swing
// 4   (3,   9,   2)
//                     ROT: LH, RH
// 5   (6,   9,   2)
//                     RH swing
// 6   (6,   6,   2)
ControllerBase* DynamicPoseGraph::createLinks(std::vector<double> times)
{
  ControllerBase* controller = createPoseSequence(times);
  controller->toXML("controller1.xml");

#if defined (LINK_DEBUG)
  Rcs::Viewer viewer;
  viewer.add(new Rcs::GraphNode(controller->getGraph()));
  viewer.runInThread();
  RPAUSE_DL(1);
#endif


  // Link all boxes to previous ones
  linkBoxesWithCoupledJoints(controller, times);

  // Link all non-changing hands to previous ones
  linkHandsWithCoupledJoints(controller);

  // Exchange orientation constraints of changed hand to Polar
  makeHandsPolar(controller);

  // Remove all partner tasks
  removePartnerTasks(controller);

  // Write modified controller to xml file
  controller->toXML("cSequencePolar.xml");

#if defined (LINK_DEBUG)
  RPAUSE_MSG("Hit enter to finish linking");
#endif

  return controller;
}

void DynamicPoseGraph::linkHands(ControllerBase* controller)
{
  const RcsGraph* graph = controller->getGraph();

  // 0->1: Link right hand
  {
    RLOG(0, "0->1: Link right hand: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = controller->getTask("XYZ_R_1")->getEffector();
    const RcsBody* ref = controller->getTask("XYZ_R")->getEffector();
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    const RcsBody* refParent = RCSBODY_BY_ID(graph, ref->parentId);
    Task* tsk = new TaskJoints(efParent, refParent, controller->getGraph());
    tsk->print();
    tsk->setName("XYZ_R_1");
    RCHECK(controller->eraseTask("ABC_R_1"));
    RCHECK(controller->replaceTask("XYZ_R_1", tsk));

    // RCHECK(controller->eraseTask("XYZ_R_1"));
    // RCHECK(controller->eraseTask("ABC_R_1"));
    // controller->add(tsk);
  }

  // 1->2: Link both hands
  {
    RLOG(0, "1->2: Link both hands: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = controller->getTask("XYZ_L_2")->getEffector();
    const RcsBody* ref = controller->getTask("XYZ_L_1")->getEffector();
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    const RcsBody* refParent = RCSBODY_BY_ID(graph, ref->parentId);
    TaskJoints* tsk = new TaskJoints(efParent, refParent, controller->getGraph());
    tsk->setName("XYZ_L_2");
    controller->replaceTask("XYZ_L_2", tsk);
    controller->eraseTask("ABC_L_2");

    RLOG(0, "1->2: Linking XYZ_R_2 to XYZ_R_1");
    TaskJoints* tjPrev = dynamic_cast<TaskJoints*>(controller->getTask("XYZ_R_1"));
    RCHECK(tjPrev);
    ef = controller->getTask("XYZ_R_2")->getEffector();
    RCHECK(ef);
    tsk = new TaskJoints(efParent, NULL, controller->getGraph());
    tsk->setRefJoints(tjPrev->getJoints());
    tsk->setName("XYZ_R_2");
    controller->replaceTask("XYZ_R_2", tsk);
    controller->eraseTask("ABC_R_2");
  }

  // 2->3: Link left hand
  {
    RLOG(0, "2->3: Link left hand");
    const RcsBody* ef = controller->getTask("XYZ_L_3")->getEffector();
    RCHECK(ef);
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    TaskJoints* tj = new TaskJoints(efParent, NULL, controller->getGraph());
    TaskJoints* tjPrev = dynamic_cast<TaskJoints*>(controller->getTask("XYZ_L_2"));
    RCHECK(tjPrev);
    tj->setRefJoints(tjPrev->getJoints());
    tj->setName("XYZ_L_3");
    controller->replaceTask("XYZ_L_3", tj);
    controller->eraseTask("ABC_L_3");
  }

  // 3->4: Link right hand
  {
    RLOG(0, "3->4: Link right hand: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = controller->getTask("XYZ_R_4")->getEffector();
    const RcsBody* ref = controller->getTask("XYZ_R_3")->getEffector();
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    const RcsBody* refParent = RCSBODY_BY_ID(graph, ref->parentId);
    TaskJoints* tj = new TaskJoints(efParent, refParent, controller->getGraph());
    tj->setName("XYZ_R_4");
    controller->replaceTask("XYZ_R_4", tj);
    controller->eraseTask("ABC_R_4");
  }

  // 4->5: Link both hands
  {
    RLOG(0, "4->5: Link both hands: dim=%zu", controller->getNumberOfTasks());

    const RcsBody* ef = controller->getTask("XYZ_L_5")->getEffector();
    const RcsBody* ref = controller->getTask("XYZ_L_4")->getEffector();
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    const RcsBody* refParent = RCSBODY_BY_ID(graph, ref->parentId);
    TaskJoints* tj = new TaskJoints(efParent, refParent, controller->getGraph());
    tj->setName("XYZ_L_5");
    controller->replaceTask("XYZ_L_5", tj);
    controller->eraseTask("ABC_L_5");

    TaskJoints* tjPrev = dynamic_cast<TaskJoints*>(controller->getTask("XYZ_R_4"));
    RCHECK(tjPrev);
    ef = controller->getTask("XYZ_R_5")->getEffector();
    RCHECK(ef);
    tj = new TaskJoints(efParent, NULL, controller->getGraph());
    tj->setRefJoints(tjPrev->getJoints());
    tj->setName("XYZ_R_5");
    controller->replaceTask("XYZ_R_5", tj);
    controller->eraseTask("ABC_R_5");
  }

  // 5->6: Link left hand
  {
    RLOG(0, "5->6: Link left hand");
    TaskJoints* tjPrev = dynamic_cast<TaskJoints*>(controller->getTask("XYZ_L_5"));
    RCHECK(tjPrev);
    const RcsBody* ef = controller->getTask("XYZ_L_6")->getEffector();
    RCHECK(ef);
    const RcsBody* efParent = RCSBODY_BY_ID(graph, ef->parentId);
    TaskJoints* tj = new TaskJoints(efParent, NULL, controller->getGraph());
    tj->setRefJoints(tjPrev->getJoints());
    tj->setName("XYZ_L_6");
    controller->replaceTask("XYZ_L_6", tj);
    controller->eraseTask("ABC_L_6");
  }

}

void DynamicPoseGraph::allowBoxRotation(ControllerBase* controller)
{
  TaskJoints* tj = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_2"));
  RCHECK(tj);
  tj->removeTask(2);
  controller->recomputeIndices();

  tj = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_5"));
  RCHECK(tj);
  tj->setRefJoints(std::vector<const RcsJoint*>());
  tj->setRefGains(std::vector<double>());

  tj = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_6"));
  RCHECK(tj);
  tj->setRefJoints(std::vector<const RcsJoint*>());
  tj->setRefGains(std::vector<double>());
}

void DynamicPoseGraph::makeHandsPolar(ControllerBase* controller)
{
#if 0
  // Replace ABC_R with Polar task
  {
    RLOG(5, "Replacing ABC_R with Polar task");
    Task* tsk = TaskFactory::createTask("<Task name=\"POLAR_R\" controlVariable= \"POLAR\" effector=\"PowerGrasp_R\" refBdy=\"Box_v\" axisDirection=\"Y\" active=\"true\" />", controller->getGraph());
    RCHECK(tsk);
    controller->replaceTask("ABC_R", tsk);
  }
#endif

  // Replace ABC_R_3 with Polar task
  {
    RLOG(5, "Replacing ABC_R_3 Euler with Polar task");
    Task* tsk = TaskFactory::createTask("<Task name=\"POLAR_R_3\" controlVariable= \"POLAR\" effector=\"PowerGrasp_R_3\" refBdy=\"Box_v_3\" axisDirection=\"Y\" active=\"true\" />", controller->getGraph());
    RCHECK(tsk);
    controller->replaceTask("ABC_R_3", tsk);
  }

  // Replace ABC_L_1 with Polar task
  {
    RLOG(5, "Replacing ABC_L_1 Euler with Polar task");
    Task* tsk = TaskFactory::createTask("<Task name=\"POLAR_L_1\" controlVariable= \"POLAR\" effector=\"PowerGrasp_L_1\" refBdy=\"Box_v_1\" axisDirection=\"Y\" active=\"true\" />", controller->getGraph());
    RCHECK(tsk);
    controller->replaceTask("ABC_L_1", tsk);
  }

  // Replace ABC_L_4 with Polar task
  {
    RLOG(5, "Replacing ABC_L_4 with Polar task");
    Task* tsk = TaskFactory::createTask("<Task name=\"POLAR_L_4\" controlVariable= \"POLAR\" effector=\"PowerGrasp_L_4\" refBdy=\"Box_v_4\" axisDirection=\"Y\" active=\"true\" />", controller->getGraph());
    RCHECK(tsk);
    controller->replaceTask("ABC_L_4", tsk);
  }
}

void DynamicPoseGraph::removePartnerTasks(ControllerBase* controller)
{
  size_t nErased = 0;
  for (size_t i = 0; i < controller->getNumberOfTasks(); ++i)
  {
    if (STRNEQ("Partner", controller->getTask(i)->getName().c_str(), 7))
    {
      RLOG_CPP(5, "Erasing task " << controller->getTask(i)->getName());
      controller->eraseTask(i);
      i = 0;
      nErased++;
    }
  }
}

static int getNonCoupledParent(const RcsGraph* graph, const RcsJoint* jnt)
{
  if (jnt==NULL)
  {
    return -1;
  }

  if (jnt->coupledToId!=-1)
  {
    return jnt->coupledToId;
  }

  RcsJoint* jPtr = (RcsJoint*) jnt;

  while (jPtr->coupledToId!=-1)
  {
    jPtr = &graph->joints[jPtr->coupledToId];
  }

  return jPtr->id;
}

void DynamicPoseGraph::linkHandsWithCoupledJoints(ControllerBase* controller)
{
  RcsGraph* g = controller->getGraph();

  // 0->1: Link right hand
  {
    RLOG(0, "0->1: Link right hand: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_R_1");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_R");
    RCHECK(ef);
    RCHECK(ref);
    RCHECK(RcsBody_numJoints(g, ef)==RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);
    RCHECK(jef && jref);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("ABC_R_1");
    controller->eraseTask("XYZ_R_1");
  }

  // 1->2: Link both hands
  {
    RLOG(0, "1->2: Link both hands: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_L_2");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_L_1");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_L_2");
    controller->eraseTask("ABC_L_2");

    RLOG(0, "1->2: Linking XYZ_R_2 to XYZ_R_1");
    ef = RcsGraph_getBodyByName(g, "PowerAnchor_R_2");
    ref = RcsGraph_getBodyByName(g, "PowerAnchor_R_1");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    jef = RCSJOINT_BY_ID(g, ef->jntId);
    jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_R_2");
    controller->eraseTask("ABC_R_2");
  }

  // 2->3: Link left hand
  {
    RLOG(0, "2->3: Link left hand");
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_L_3");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_L_2");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_L_3");
    controller->eraseTask("ABC_L_3");
  }

  // 3->4: Link right hand
  {
    RLOG(0, "3->4: Link right hand: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_R_4");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_R_3");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_R_4");
    controller->eraseTask("ABC_R_4");
  }

  // 4->5: Link both hands
  {
    RLOG(0, "4->5: Link both hands: dim=%zu", controller->getNumberOfTasks());
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_L_5");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_L_4");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_L_5");
    controller->eraseTask("ABC_L_5");

    ef = RcsGraph_getBodyByName(g, "PowerAnchor_R_5");
    ref = RcsGraph_getBodyByName(g, "PowerAnchor_R_4");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    jef = RCSJOINT_BY_ID(g, ef->jntId);
    jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_R_5");
    controller->eraseTask("ABC_R_5");

  }

  // 5->6: Link left hand
  {
    RLOG(0, "5->6: Link left hand");
    const RcsBody* ef = RcsGraph_getBodyByName(g, "PowerAnchor_L_6");
    const RcsBody* ref = RcsGraph_getBodyByName(g, "PowerAnchor_L_5");
    RCHECK(RcsBody_numJoints(g, ef) == RcsBody_numJoints(g, ref));

    RcsJoint* jef = RCSJOINT_BY_ID(g, ef->jntId);
    RcsJoint* jref = RCSJOINT_BY_ID(g, ref->jntId);

    while (jef)
    {
      jef->coupledToId = getNonCoupledParent(g, jref);
      jef->couplingPoly[0] = 1.0;
      jef->nCouplingCoeff = 1;
      jef = RCSJOINT_BY_ID(g, jef->nextId);
      RCHECK(jref);
      jref = RCSJOINT_BY_ID(g, jref->nextId);
    }
    controller->eraseTask("XYZ_L_6");
    controller->eraseTask("ABC_L_6");
  }

}

// Link all boxes to previous ones
void DynamicPoseGraph::linkBoxes(ControllerBase* controller,
                                 std::vector<double> times)
{
  ControllerBase* source = tc->getInternalController();

  // Box position missing here
  size_t nTasks = source->getNumberOfTasks();
  int t_phi_b = source->getTaskIndex("Phi_Box");
  RCHECK(t_phi_b!=-1);

  for (size_t i=1; i<times.size(); ++i)
  {
    RLOG_CPP(0, "Linking box rotation " << i << " against " << i - 1);
    TaskJoints* curr = dynamic_cast<TaskJoints*>(controller->getTask(i*nTasks+t_phi_b));
    TaskJoints* prev = dynamic_cast<TaskJoints*>(controller->getTask((i-1)*nTasks+t_phi_b));
    RCHECK(curr);
    RCHECK(prev);
    std::vector<const RcsJoint*> refJnts = prev->getJoints();
    curr->setRefJoints(refJnts);
    curr->setRefGains(-1.0);
  }

  // Remove joint from box rotation about 90 degrees
  allowBoxRotation(controller);
}

// Link all boxes using coupled joints
void DynamicPoseGraph::linkBoxesWithCoupledJoints(ControllerBase* controller,
                                                  std::vector<double> times)
{
  const RcsGraph* g = controller->getGraph();

  TaskJoints* tsk0 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box"));
  TaskJoints* tsk1 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_1"));
  TaskJoints* tsk2 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_2"));
  TaskJoints* tsk3 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_3"));
  TaskJoints* tsk4 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_4"));
  TaskJoints* tsk5 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_5"));
  TaskJoints* tsk6 = dynamic_cast<TaskJoints*>(controller->getTask("Phi_Box_6"));
  RCHECK(tsk0 && tsk1 && tsk2 && tsk3 && tsk4 && tsk5 && tsk6);

  // 0->1: All fixed
  std::vector<const RcsJoint*> jPrev = tsk0->getJoints();
  std::vector<const RcsJoint*> jCurr = tsk1->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<3; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }

  // 1->2: Allow rotation
  jPrev = tsk1->getJoints();
  jCurr = tsk2->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<2; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }

  // 2->3: All fixed
  jPrev = tsk2->getJoints();
  jCurr = tsk3->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<3; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }

  // 3->4: All fixed
  jPrev = tsk3->getJoints();
  jCurr = tsk4->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<3; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }

  // 4->5: Allow rotation
  jPrev = tsk4->getJoints();
  jCurr = tsk5->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<2; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }

  // 5->6: All fixed
  jPrev = tsk5->getJoints();
  jCurr = tsk6->getJoints();
  RCHECK(jPrev.size()==jCurr.size() && jPrev.size()==3);
  for (size_t i=0; i<3; ++i)
  {
    RcsJoint* jPtr = (RcsJoint*) jCurr[i];
    jPtr->coupledToId = getNonCoupledParent(g, jPrev[i]);
    jPtr->couplingPoly[0] = 1.0;
    jPtr->nCouplingCoeff = 1;
  }


  //controller->eraseTask(tsk0->getName());
  controller->eraseTask(tsk1->getName());
  controller->eraseTask(tsk2->getName());
  controller->eraseTask(tsk3->getName());
  controller->eraseTask(tsk4->getName());
  controller->eraseTask(tsk5->getName());
  //controller->eraseTask(tsk6->getName());
  tsk6->removeTask(0);
  tsk6->removeTask(0);
  controller->recomputeIndices();





  // Replace all box position tasks with coupled joints
  std::vector<std::string> tBoxNames;
  tBoxNames.push_back("XYZ_Box");
  tBoxNames.push_back("XYZ_Box_1");
  tBoxNames.push_back("XYZ_Box_2");
  tBoxNames.push_back("XYZ_Box_3");
  tBoxNames.push_back("XYZ_Box_4");
  tBoxNames.push_back("XYZ_Box_5");
  tBoxNames.push_back("XYZ_Box_6");

  for (size_t i=1; i<tBoxNames.size(); ++i)
  {
    Task* curr = controller->getTask(tBoxNames[i]);
    Task* prev = controller->getTask(tBoxNames[i-1]);
    RCHECK(curr && prev);
    RcsJoint* jCurr = RCSJOINT_BY_ID(g, curr->getEffector()->jntId);
    RcsJoint* jPrev = RCSJOINT_BY_ID(g, prev->getEffector()->jntId);

    for (int j=0; j<3; ++j)
    {
      RCHECK(jCurr && jPrev);
      jCurr->coupledToId = getNonCoupledParent(g, jPrev);
      jCurr->couplingPoly[0] = 1.0;
      jCurr->nCouplingCoeff = 1;
      jCurr = RCSJOINT_BY_ID(g, jCurr->nextId);
      jPrev = RCSJOINT_BY_ID(g, jPrev->nextId);
    }

  }


  controller->eraseTask(tBoxNames[1]);
  controller->eraseTask(tBoxNames[2]);
  controller->eraseTask(tBoxNames[3]);
  controller->eraseTask(tBoxNames[4]);
  controller->eraseTask(tBoxNames[5]);
  controller->eraseTask(tBoxNames[6]);
}

}   // namespace Rcs
