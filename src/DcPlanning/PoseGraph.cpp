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

#include "PoseGraph.h"

#include <Rcs_macros.h>
#include <Rcs_joint.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_graphParser.h>
#include <IkSolverRMR.h>

#include <algorithm>
#include <string>



namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
PoseGraph::Result PoseGraph::create(const ControllerBase* src,
                                    const std::vector<Adjacency>& adja,
                                    const std::vector<StepSpec>& stepSpecs,
                                    const MatNd* postures,
                                    const double offset[3],
                                    SequenceAlgo algo)
{
  Result res;

  switch (algo)
  {
    case Coupled:
      res = createCoupled(src, adja, postures, offset);
      break;

    case NaiiveFirstPose:
      res = createNaiive(src, adja, postures, offset, false);
      break;

    case NaiiveSequence:
      res = createNaiive(src, adja, postures, offset, true);
      break;

    case Duplicated:
      res = createCoupled(src, adja, postures, offset, true);
      break;

    default:
      RFATAL("Unknown algorithm: %d", algo);
  }

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
PoseGraph::Result PoseGraph::createNaiive(const ControllerBase* cSingle,
                                          const std::vector<Adjacency>& adja,
                                          const MatNd* postures,
                                          const double offset[3],
                                          bool avgSeq)
{
  MatNd* postureArray = NULL;

  // If no postures array is passed, we create one with as many postures as
  // there are steps in the adjacencies, all with the state of the graph.
  if (postures == NULL)
  {
    size_t nPoses = 0;

    for (size_t i = 0; i < adja.size(); ++i)
    {
      nPoses = std::max(nPoses, adja[i].adjacencyList.size());
    }

    postureArray = MatNd_create(nPoses, cSingle->getGraph()->dof);
    postures = postureArray;

    for (size_t i = 0; i < nPoses; ++i)
    {
      double* row = MatNd_getRowPtr(postureArray, i);
      VecNd_copy(row, cSingle->getGraph()->q->ele, cSingle->getGraph()->dof);
    }
  }

  // Create the controller with a clone of the original controller for
  // each posture.
  // We change the initial state to the q0 state. That's the one that is
  // set from the model_state default in the graph's xml file.
  MatNd* q0 = MatNd_createLike(cSingle->getGraph()->q);
  RcsGraph_getDefaultState(cSingle->getGraph(), q0);
  ControllerBase* cNaiive = createGraph(cSingle, postures, offset, q0);
  MatNd_destroy(q0);

  REXEC(1)
  {
    int nw = 0, ne = 0;
    RcsGraph_check(cNaiive->getGraph(), &ne, &nw);
    RLOG(1, "Graph check: %d warnings, %d errors", nw, ne);
    RLOG(1, "Writing graph file");
    cNaiive->toXML("cNaiive.xml");
    RLOG(1, "Writing dot file");
    RcsGraph_writeDotFile(cNaiive->getGraph(), "gNaiive.dot");
    RCHECK(ne == 0);
  }

  // Acquire the activation vector from the xml file so that we can also
  // deal with inactive tasks.
  RLOG(1, "Reading activations");
  MatNd* aSingle = MatNd_create(cSingle->getNumberOfTasks(), 1);
  cSingle->readActivationsFromXML(aSingle);

  // Create activation vector for the augmented controller by appending the
  // activation vector to itself until the dinemsions match.
  MatNd* aNaiive = MatNd_clone(aSingle);

  for (unsigned int i = 0; i < postures->m - 1; ++i)
  {
    MatNd_appendRows(aNaiive, aSingle);
  }

  RCHECK_MSG(aNaiive->m == cNaiive->getNumberOfTasks(), "%u   %zu",
             aNaiive->m, cNaiive->getNumberOfTasks());


  // Relax constraints
  ControllerBase* cRelaxed = new ControllerBase(*cNaiive);
  MatNd* aRelaxed = MatNd_clone(aNaiive);
  MatNd* x_curr = MatNd_create(cRelaxed->getTaskDim(), 1);
  cRelaxed->computeX(x_curr);

  for (size_t i = 0; i < adja.size(); ++i)
  {
    relaxAll(cRelaxed, adja[i], aRelaxed);
  }

  // Converge to minimum of decoupled and relaxed poses
  convergeTaskConstraints(cRelaxed, aRelaxed, x_curr);
  cRelaxed->computeX(x_curr);

  Result res;
  res.controller = cRelaxed;
  res.activation = aRelaxed;

  // Returning here returns a sequence with each step locally converged to its
  // minimum.
  //return res;


  MatNd* x_des = MatNd_clone(x_curr);

  for (size_t i = 0; i < adja.size(); ++i)
  {
    RLOG_CPP(0, "Adjacency " << i);
    computeAverageXdes(cRelaxed, adja[i], aRelaxed, x_des, avgSeq);
  }


  // Converge to minimum of averaged, non-relaxed poses
  convergeTaskConstraints(cNaiive, aNaiive, x_des);
  cNaiive->printX(x_des, aNaiive);

  // Clean up
  MatNd_destroy(postureArray);
  MatNd_destroy(aSingle);
  MatNd_destroy(x_des);
  MatNd_destroy(x_curr);
  // MatNd_destroy(aNaiive);
  // delete cNaiive;

  res.controller = cNaiive;
  res.activation = aNaiive;

  return res;
}

/*******************************************************************************
 *
 ******************************************************************************/
PoseGraph::Result PoseGraph::createCoupled(const ControllerBase* src,
                                           const std::vector<Adjacency>& adjacencies,
                                           const MatNd* postures,
                                           const double offset[3],
                                           bool duplicatedOnly)
{
  MatNd* postureArray = NULL;

  // If no postures array is passed, we create one with as many postures as
  // there are steps in the adjacencies, all with the state of the graph.
  if (postures == NULL)
  {
    size_t nPoses = 0;

    for (size_t i=0; i<adjacencies.size(); ++i)
    {
      nPoses = std::max(nPoses, adjacencies[i].adjacencyList.size());
    }

    postureArray = MatNd_create(nPoses, src->getGraph()->dof);
    postures = postureArray;

    for (size_t i=0; i<nPoses; ++i)
    {
      double* row = MatNd_getRowPtr(postureArray, i);
      VecNd_copy(row, src->getGraph()->q->ele, src->getGraph()->dof);
    }
  }

  // Create the controller with a clone of the original controller for
  // each posture.
  // We change the initial state to the q0 state. That's the one that is
  // set from the model_state default in the graph's xml file.
  MatNd* q0 = MatNd_createLike(src->getGraph()->q);
  RcsGraph_getDefaultState(src->getGraph(), q0);
  ControllerBase* controller = createGraph(src, postures, offset, q0);
  MatNd_destroy(q0);
  REXEC(1)
  {
    int nw = 0, ne = 0;
    RcsGraph_check(controller->getGraph(), &ne, &nw);
    RLOG(1, "Graph check: %d warnings, %d errors", nw, ne);
    RLOG(1, "Writing graph file");
    controller->toXML("cMultiplied.xml");
    RLOG(1, "Writing dot file");
    RcsGraph_writeDotFile(controller->getGraph(), "gMultiplied.dot");
    RCHECK(ne == 0);
  }

  // Acquire the activation vector from the xml file so that we can also
  // deal with inactive tasks.
  RLOG(1, "Reading activations");
  MatNd* src_activations = MatNd_create(src->getNumberOfTasks(), 1);
  src->readActivationsFromXML(src_activations);

  // Create activation vector for the augmented controller by appending the
  // activation vector to itself until the dinemsions match.
  MatNd* activations = MatNd_clone(src_activations);

  for (unsigned int i=0; i<postures->m-1; ++i)
  {
    MatNd_appendRows(activations, src_activations);
  }

  RCHECK_MSG(activations->m==controller->getNumberOfTasks(), "%u   %zu",
             activations->m, controller->getNumberOfTasks());

  Result res;
  res.controller = controller;
  res.activation = activations;

  if (duplicatedOnly)
  {
    MatNd_destroy(postureArray);
    MatNd_destroy(src_activations);
    return res;
  }

  // Connect task constraints between poses
  for (size_t i=0; i<adjacencies.size(); ++i)
  {
    linkTasks(controller, adjacencies[i], activations);
  }

  // Connect joint constraints between poses
  for (size_t i = 0; i < adjacencies.size(); ++i)
  {
    linkBodyJoints(controller, adjacencies[i], activations);
  }

  // Relax constraints
  for (size_t i=0; i<adjacencies.size(); ++i)
  {
    relax(controller, adjacencies[i], activations);
  }

  // Remove all inactive tasks. That's more a cosmetic step.
  //eraseInactiveTasks(controller, activations);

  MatNd* x_curr = MatNd_create(controller->getTaskDim(), 1);
  controller->computeX(x_curr);
  convergeTaskConstraints(controller, activations, x_curr);
  MatNd_destroy(x_curr);

  // Clean up
  MatNd_destroy(postureArray);
  MatNd_destroy(src_activations);

  return res;
}

/*******************************************************************************
 * Constructs a new graph with as many poses as in there are rows in the
 * postures array.
 ******************************************************************************/
ControllerBase* PoseGraph::createGraph(const ControllerBase* src,
                                       const MatNd* postures,
                                       const double offset[3],
                                       const MatNd* optionalInitState)
{
  ControllerBase ci(*src);
  ControllerBase* controller = new ControllerBase(*src);

  if (optionalInitState)
  {
    RcsGraph_changeInitState(controller->getGraph(), optionalInitState);
    RcsGraph_changeInitState(ci.getGraph(), optionalInitState);
  }

  HTr A_offset;
  HTr_setIdentity(&A_offset);

  RLOG_CPP(1, "Creating " << postures->m << " postures");

  for (size_t i=0; i<postures->m; ++i)
  {
    MatNd q_i = MatNd_getRowViewTranspose(postures, i);

    if (i==0)
    {
      RcsGraph_setState(controller->getGraph(), &q_i, NULL);
    }
    else
    {
      RcsGraph_setState(ci.getGraph(), &q_i, NULL);
      char suffix[64];
      snprintf(suffix, 64, "_%zu", i);
      Vec3d_constMul(A_offset.org, offset, (double) i);
      controller->add(&ci, suffix, &A_offset);
    }

  }


  return controller;
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<size_t> PoseGraph::getTasksForEffector(const ControllerBase* controller,
                                                   const std::string& effectorName,
                                                   const MatNd* activation) const
{
  RLOG_CPP(1, "Searching tasks for effector " << effectorName);
  RCHECK(activation->m==controller->getNumberOfTasks());

  std::vector<size_t> tix;

  for (size_t i=0; i<controller->getNumberOfTasks(); ++i)
  {
    if (activation->ele[i]==0.0)
    {
      continue;
    }

    const RcsBody* effector = controller->getTask(i)->getEffector();

    if (!effector)
    {
      continue;
    }

    if (effectorName == std::string(effector->name))
    {
      tix.push_back(i);
    }
  }

  RLOG_CPP(1, "Found " << tix.size());

  return tix;
}

/*******************************************************************************
 * Must be called with the complete graph including all poses.
 * If there are several tasks with the same effector, the function performs the
 * re-linking for each of them. This is just a bit overhead, but does not
 * change anything in the topology. All tasks with the given effector are
 * scheduled for deletion.
 ******************************************************************************/
void PoseGraph::linkBodyJoints(ControllerBase* controller,
                               const Adjacency& connection,
                               MatNd* activation)
{
  if (connection.bdyName.empty())
  {
    return;
  }
  RcsGraph* graph = controller->getGraph();

  // Now we traverse all adjacencies and create the connections.
  for (size_t bdyId=0; bdyId<connection.adjacencyList.size(); ++bdyId)
  {
    int parentId = connection.adjacencyList[bdyId];

    // If the id to connect is -1, there is no connection and we go to the
    // next entry.
    if (parentId == -1)
    {
      continue;
    }

    // Compute the body name of the connection according to the id of the pose
    std::string bdyName = connection.bdyName;
    if (bdyId>0)
    {
      bdyName += "_" + std::to_string(bdyId);
    }

    // Assemble a vector of all RcsJoints of this body
    std::vector<RcsJoint*> jBdy;

    RcsBody* bdy = RcsGraph_getBodyByName(graph, bdyName.c_str());
    RCHECK_MSG(bdy, "Not found: \"%s\"", bdyName.c_str());
    RCSBODY_FOREACH_JOINT(graph, bdy)
    {
      jBdy.push_back(JNT);
    }

    // Compute the name of the parent body according to the id of the pose
    std::string parentName = connection.bdyName;
    if (parentId>0)
    {
      parentName += "_" + std::to_string(parentId);
    }

    // Assemble a vector of all RcsJoints of the parent body. We enforce that
    // the parent body has exactly the same amount of joints as the body that
    // we want to connect.
    std::vector<const RcsJoint*> jParent;

    RcsBody* parent = RcsGraph_getBodyByName(graph, parentName.c_str());
    RCHECK_MSG(parent, "Not found: \"%s\"", parentName.c_str());
    RCSBODY_FOREACH_JOINT(graph, parent)
    {
      jParent.push_back(JNT);
    }
    RLOG_CPP(0, "Parent-body " << parentName << ": Connecting " << jBdy.size()
             << " joints from body " << bdyName);
    RCHECK(jBdy.size() == jParent.size());

    // Here we change the joint so that it is kinematically connected to the
    // joints of the parent body, or in case the parent body joints are already
    // coupled, to the joints of the recursive parent that does not have coupled
    // joints. We need to do this, since currently it is not possible to couple
    // joints to coupled joints. From the underlying math, it doesn't make a
    // difference. At some point this might get fixed.
    for (size_t i=0; i<jBdy.size(); ++i)
    {
      jBdy[i]->coupledToId = RcsJoint_getNonCoupledParentId(graph, jParent[i]);
      jBdy[i]->couplingPoly[0] = 1.0;
      jBdy[i]->nCouplingCoeff = 1;
      RLOG(0, "Connecting joint \"%s\" to \"%s\"",
           jBdy[i]->name, RCSJOINT_NAME_BY_ID(graph, jBdy[i]->coupledToId));
    }

    std::vector<size_t> toDelete = getTasksForEffector(controller, bdyName,
                                                       activation);
    for (size_t  i=0; i<toDelete.size(); ++i)
    {
      RLOG(1, "Erasing task \"%s\"", controller->getTaskName(toDelete[i]).c_str());
      RLOG(1, "Posture %zu, bdyName \"%s\"", bdyId, bdyName.c_str());
      MatNd_set(activation, toDelete[i], 0, 0.0);
    }

  }

}

/*******************************************************************************
 * We relax postures that are not connected to any preceding pose. This is
 * indicated by the adjacency list entry -1.
 * To exclude the first pose, we simply start going through the postures from
 * index 1.
 * To exclude the last pose, we look for the last entry with -1.
 ******************************************************************************/
void PoseGraph::relax(ControllerBase* controller, const Adjacency& connection,
                      MatNd* activation)
{
  // We ignore the first posture, therefore we must make sure that the
  // adjacency has one or more entries.
  if (connection.adjacencyList.empty())
  {
    return;
  }

  // \todo: Check this for circular topologies
  size_t firstPose = connection.fixFirstPose ? 1 : 0;
  size_t lastPose = connection.adjacencyList.size();

  if (connection.fixLastPose)
  {
    for (int i=connection.adjacencyList.size()-1; i>=0; --i)
    {
      RLOG(1, "Checking index %d: %d", i, connection.adjacencyList[i]);
      if (connection.adjacencyList[i]==-1)
      {
        lastPose = i;
        break;
      }
    }
  }

  RLOG(1, "Last pose for %s is %zu", connection.bdyName.c_str(), lastPose);

  // Now we traverse all adjacencies and look for non-coupled postures
  for (size_t bdyId=firstPose; bdyId<lastPose; ++bdyId)
  {
    int parentId = connection.adjacencyList[bdyId];

    // If the id to connect is different from -1, there is no connection and we
    // can relax the contraints of the adjacency instance.
    if (parentId != -1)
    {
      continue;
    }





    RCHECK(connection.originalTasks.size()==connection.relaxedTasks.size());

    for (size_t i=0; i<connection.originalTasks.size(); ++i)
    {

      std::string old = connection.originalTasks[i];
      std::string relax = connection.relaxedTasks[i];

      if (bdyId>0)
      {
        old += std::string("_") + std::to_string(bdyId);
        relax += std::string("_") + std::to_string(bdyId);
      }

      int tidxOld = controller->getTaskIndex(old.c_str());
      int tidxRelax = controller->getTaskIndex(relax.c_str());

      RLOG(1, "Relaxing task %s - %s", old.c_str(), relax.c_str());

      if (tidxOld!=-1)
      {
        if (tidxRelax != -1)
        {
          activation->ele[tidxRelax] = 1.0;
        }

        activation->ele[tidxOld] = 0.0;
        RLOG(1, "Erasing task \"%s\"", controller->getTaskName(tidxOld).c_str());
        RLOG(1, "Success: index %d = 0, index %d = 1", tidxOld, tidxRelax);
      }
      else
      {
        RLOG(1, "Failed: idx_old=%d idx_new=%d", tidxOld, tidxRelax);
      }

    }




  }   // for (size_t bdyId=firstPose; bdyId<lastPose; ++bdyId)

}

/*******************************************************************************
 * Remove all inactive tasks and adjust activation vector. This could be done
 * more efficiently inside the ControllerBase class, without re-ordering arrays
 * after each erase call.
 ******************************************************************************/
void PoseGraph::eraseInactiveTasks(ControllerBase* controller,
                                   MatNd* activation)
{
  std::vector<size_t> toDelete;

  for (int i=activation->m-1; i>=0; --i)
  {
    if (activation->ele[i] == 0.0)
    {
      toDelete.push_back(i);
    }
  }

  for (size_t i=0; i<toDelete.size(); ++i)
  {
    RLOG_CPP(1, "Erasing task " << toDelete[i] << " ("
             << controller->getTaskName(toDelete[i]) << ")");
    controller->eraseTask(toDelete[i]);
    MatNd_deleteRow(activation, toDelete[i]);
  }

}

/*******************************************************************************
 *
 ******************************************************************************/
void PoseGraph::linkTasks(ControllerBase* controller,
                          const Adjacency& connection,
                          MatNd* activation)
{
  if (connection.taskName.empty())
  {
    return;
  }

  RLOG(1, "Start linking tasks");
  RcsGraph* graph = controller->getGraph();

  // Now we traverse all adjacencies and create the connections.
  for (size_t step=0; step<connection.adjacencyList.size(); ++step)
  {
    int prevStep = connection.adjacencyList[step];

    // If the id to connect is -1, there is no connection and we go to the
    // next entry.
    if (prevStep == -1)
    {
      continue;
    }

    // Compute the body name of the connection according to the id of the pose
    std::string taskName = connection.taskName;
    if (step>0)
    {
      taskName += "_" + std::to_string(step);
    }

    // Compute the name of the parent body according to the id of the pose
    std::string prevTaskName = connection.taskName;
    if (prevStep>0)
    {
      prevTaskName += "_" + std::to_string(prevStep);
    }

    // We just replace refBdy and refFrame with the preceeding task
    Rcs::Task* currTask = controller->getTask(taskName);
    const Rcs::Task* prevTask = controller->getTask(prevTaskName);
    RCHECK_MSG(currTask, "Current task \"%s\" not found", taskName.c_str());
    RCHECK_MSG(prevTask, "Previous task \"%s\" not found", prevTaskName.c_str());
    RLOG(1, "Changing task %s: effector id: %d -> %d",
         taskName.c_str(), currTask->getEffectorId(), prevTask->getEffectorId());
    currTask->setRefBodyId(prevTask->getEffectorId());
    currTask->setRefFrameId(prevTask->getEffectorId());
  }

  RLOG(1, "Done linking tasks");
}

/*******************************************************************************
 *
 ******************************************************************************/
MatNd* PoseGraph::posturesFromModelState(const RcsGraph* graph,
                                         const std::string& msName)
{
  MatNd* postures = MatNd_create(0, graph->dof);
  MatNd* qi = MatNd_createLike(graph->q);
  std::vector<int> ts;
  ts = RcsGraph_getModelStateTimeStamps(graph, msName);

  // Sort in ascending order
  sort(ts.begin(), ts.end());

  for (size_t i=0; i<ts.size(); ++i)
  {
    //RLOG(0, "time_stamp: %d", ts[i]);
    MatNd_copy(qi, graph->q);
    bool success = RcsGraph_getModelStateFromXML(qi, graph, msName.c_str(), ts[i]);

    if (success)
    {
      MatNd_transposeSelf(qi);
      MatNd_appendRows(postures, qi);
      MatNd_transposeSelf(qi);
    }

  }

  MatNd_destroy(qi);

  return postures;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool PoseGraph::convergeTaskConstraints(ControllerBase* controller,
                                        const MatNd* activation,
                                        const MatNd* x_des,
                                        int maxIter,
                                        double clipLimit)
{
  const double convergenceEps = RCS_DEG2RAD(0.1);   // make larger if it takes too long
  const double alpha = 0.1;               // Null space gain - reduce if jitter
  const double lambda = 1.0e-8;           // Make IK a bit singularity-robust
  double maxDq = 0.0, maxDx = 0.0;
  int iter = 0;
  IkSolverRMR solver(controller);

  MatNd* dq_des = MatNd_create(controller->getGraph()->dof, 1);
  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller->getGraph()->nJ);

  // Inverse kinematics until convergence
  do
  {
    controller->computeDX(dx_des, x_des, activation);
    maxDx = MatNd_maxAbsEle(dx_des);
    controller->computeJointlimitGradient(dH);   // Add other null space components if desired
    MatNd_constMulSelf(dH, alpha);

    if (clipLimit>0.0)
    {
      MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
      MatNd_saturateSelf(dx_des, &clipArr);
    }

    solver.solveRightInverse(dq_des, dx_des, dH, activation, lambda);
    maxDq = MatNd_maxAbsEle(dq_des);
    MatNd_addSelf(controller->getGraph()->q, dq_des);
    RcsGraph_setState(controller->getGraph(), NULL, NULL);
    RLOG(0, "Iteration %d of %d: maxDx = %f   maxDq = %f",
         iter++, maxIter, maxDx, maxDq);
  }
  while ((maxDq>convergenceEps) && (iter<maxIter));

  return true;
}

/*******************************************************************************
 * We relax all postures.
 * To exclude the first pose, we simply start going through the postures from
 * index 1.
 * To exclude the last pose, we look for the last entry with -1.
 ******************************************************************************/
void PoseGraph::relaxAll(ControllerBase* controller,
                         const Adjacency& connection,
                         MatNd* activation)
{
  // We ignore the first posture, therefore we must make sure that the
  // adjacency has one or more entries.
  if (connection.adjacencyList.empty())
  {
    return;
  }

  // \todo: Check this for circular topologies
  size_t firstPose = connection.fixFirstPose ? 1 : 0;
  size_t lastPose = connection.adjacencyList.size();

  if (connection.fixLastPose)
  {
    for (int i = connection.adjacencyList.size() - 1; i >= 0; --i)
    {
      RLOG(1, "Checking index %d: %d", i, connection.adjacencyList[i]);
      if (connection.adjacencyList[i] == -1)
      {
        lastPose = i;
        break;
      }
    }
  }

  RLOG(1, "Last pose for %s is %zu", connection.bdyName.c_str(), lastPose);

  // Now we traverse all adjacencies and look for non-coupled postures
  for (size_t bdyId = firstPose; bdyId < lastPose; ++bdyId)
  {





    RCHECK(connection.originalTasks.size()==connection.relaxedTasks.size());

    for (size_t i=0; i<connection.originalTasks.size(); ++i)
    {


      std::string old = connection.originalTasks[i];
      std::string relax = connection.relaxedTasks[i];

      if (bdyId > 0)
      {
        old += std::string("_") + std::to_string(bdyId);
        relax += std::string("_") + std::to_string(bdyId);
      }

      int tidxOld = controller->getTaskIndex(old.c_str());
      int tidxRelax = controller->getTaskIndex(relax.c_str());

      RLOG(1, "Relaxing task %s - %s", old.c_str(), relax.c_str());

      if (tidxOld != -1)
      {
        if (tidxRelax != -1)
        {
          activation->ele[tidxRelax] = 1.0;
        }

        activation->ele[tidxOld] = 0.0;
        RLOG(1, "Erasing task \"%s\"", controller->getTaskName(tidxOld).c_str());
        RLOG(1, "Success: index %d = 0, index %d = 1", tidxOld, tidxRelax);
      }
      else
      {
        RLOG(1, "Failed: idx_old=%d idx_new=%d", tidxOld, tidxRelax);
      }

    }









  }   // for (size_t bdyId = firstPose; bdyId < lastPose; ++bdyId)

}

/*******************************************************************************
 * This is for one task only.
 ******************************************************************************/
void PoseGraph::computeAverageXdes(ControllerBase* controller,
                                   const Adjacency& connection,
                                   MatNd* activation,
                                   MatNd* x_des,
                                   bool avgSeq)
{
  // We ignore the first posture, therefore we must make sure that the
  // adjacency has one or more entries.
  if (connection.adjacencyList.empty() || connection.originalTasks.empty())
  {
    return;
  }

  // \todo: Check this for circular topologies
  size_t firstPose = connection.fixFirstPose ? 1 : 0;
  size_t lastPose = connection.adjacencyList.size();

  if (connection.fixLastPose)
  {
    for (int i = connection.adjacencyList.size() - 1; i >= 0; --i)
    {
      RLOG(1, "Checking index %d: %d", i, connection.adjacencyList[i]);
      if (connection.adjacencyList[i] == -1)
      {
        lastPose = i;
        break;
      }
    }
  }

  RLOG(1, "Last pose for %s is %zu", connection.bdyName.c_str(), lastPose);

  // The vector seqs looks like this:
  // a.adjacencyList = {-1, 0, -1, 2, 3, -1, 5};
  // seqs[0] = {0, 1}   - > step 0 and 1 need to be averaged
  // seqs[1] = {2, 3, 4} -> step 2, 3 and 4 need to be averaged
  // seqs[2] = {5, 6}    -> step 5 and 6 need to be averaged
  std::vector<std::vector<int>> seqs;

  for (size_t i = 0; i<connection.adjacencyList.size(); ++i)
  {
    if (connection.adjacencyList[i] == -1)
    {
      seqs.push_back(std::vector<int>());
      seqs.back().push_back(i);
    }
    else if (!seqs.empty() && !seqs.back().empty())
    {
      seqs.back().push_back(i);
    }
  }

  // Erase entries with only 1 element
  for (int i = seqs.size() - 1; i >= 0; --i)
  {
    if (seqs[i].size() == 1)
    {
      seqs.erase(seqs.begin() + i);
    }
  }

  // Print out what we got
  for (size_t i = 0; i < seqs.size(); ++i)
  {
    RLOG(0, "Cluster %zu:", i);
    for (size_t j = 0; j < seqs[i].size(); ++j)
    {
      RLOG_CPP(0, seqs[i][j]);
    }
  }

  // Compute the current task state that goes into the averaging
  MatNd* x_curr = MatNd_create(controller->getTaskDim(), 1);
  controller->computeX(x_curr);

  RCHECK(connection.originalTasks.size()==connection.relaxedTasks.size());


  // Average per cluster.
  for (int i = 0; i < seqs.size(); ++i)
  {

    // Several tasks can be subject to one cluster
    for (int tsk = 0; tsk < connection.originalTasks.size(); ++tsk)
    {
      const Rcs::Task* t0 = controller->getTask(connection.originalTasks[tsk].c_str());
      RCHECK(t0);
      const size_t nx = t0->getDim();

      // Array xi has as many rows as postures to be averaged, and tak-dim cols
      MatNd* xi = MatNd_create(seqs[i].size(), nx);

      // Computation of averages per cluster
      for (int j = 0; j < seqs[i].size(); ++j)
      {
        std::string tName = connection.originalTasks[tsk];
        if (seqs[i][j] != 0)// No suffix for the first step
        {
          tName += std::string("_") + std::to_string(seqs[i][j]);
        }

        int tidx = controller->getTaskArrayIndex(tName.c_str());
        RCHECK(tidx!=-1);
        VecNd_copy(MatNd_getRowPtr(xi, j), &x_curr->ele[tidx], nx);
      }

      // Compute task average and assign to x_des
      std::vector<double> taskAvg(nx, 0.0);

      for (unsigned int j = 0; j < xi->m; ++j)
      {
        std::string tName = connection.originalTasks[tsk];
        if (seqs[i][j] != 0)// No suffix for the first step
        {
          tName += std::string("_") + std::to_string(seqs[i][j]);
        }

        const Rcs::Task* ti = controller->getTask(tName.c_str());
        RCHECK_MSG(ti, "for task %s", tName.c_str());
        VecNd_copy(taskAvg.data(), xi->ele, xi->n);   // Initialize with first step's task values

        // If avgSet is true, the pose will be averaged from all consecutive
        // poses of a sub-sequence.
        if (avgSeq)
        {
          if (ti->getClassName() == "ABC")
          {
            // This is an iterative algorithm that quickly converges. A few
            // iterations should do. \todo: Check convergence.
            for (size_t k = 0; k < 5; ++k)
            {
              Math_weightedMeanEulerAngles(taskAvg.data(), (double(*)[3]) xi->ele,
                                           NULL, nx);
            }
          }
          else if (ti->getClassName() == "POLAR")
          {
            RFATAL("Not supported: POLAR");
          }
          else   // Just component-wise mean
          {
            for (size_t k = 0; k < xi->n; ++k)
            {
              taskAvg[k] = MatNd_columnSum(xi, k)/xi->m;
            }
          }
        }   // if (avgSeq)

      }   // for (int j = 0; j < xi->m; ++j)

      // Assign task average to x_des corresponding to clusters
      for (int j = 0; j < seqs[i].size(); ++j)
      {
        std::string tName = connection.originalTasks[tsk];
        if (seqs[i][j] != 0)// No suffix for the first step
        {
          tName += std::string("_") + std::to_string(seqs[i][j]);
        }

        int tidx = controller->getTaskArrayIndex(tName.c_str());
        RCHECK(tidx!=-1);

        char buf[64];
        VecNd_toStr(taskAvg.data(), taskAvg.size(), buf);
        RLOG(0, "Copying avg %s to task %s",
             VecNd_toStr(taskAvg.data(), taskAvg.size(), buf), tName.c_str());
        VecNd_copy(MatNd_getRowPtr(x_des, tidx), taskAvg.data(), nx);
      }


      MatNd_destroy(xi);

    }   // for (int tsk = 0; tsk < connection.originalTasks.size(); ++tsk)

  }   // for (int i = 0; i < seqs.size(); ++i)

  MatNd_destroy(x_curr);
}


/*******************************************************************************
 *
 ******************************************************************************/
PoseGraph::Result PoseGraph::createNaiive_1(const ControllerBase* cSingle,
                                            const std::vector<Adjacency>& adja,
                                            const MatNd* postures,
                                            const double offset[3],
                                            bool avgSeq,
                                            MatNd* x_relaxed)
{
  MatNd* postureArray = NULL;

  // If no postures array is passed, we create one with as many postures as
  // there are steps in the adjacencies, all with the state of the graph.
  if (postures == NULL)
  {
    size_t nPoses = 0;

    for (size_t i = 0; i < adja.size(); ++i)
    {
      nPoses = std::max(nPoses, adja[i].adjacencyList.size());
    }

    postureArray = MatNd_create(nPoses, cSingle->getGraph()->dof);
    postures = postureArray;

    for (size_t i = 0; i < nPoses; ++i)
    {
      double* row = MatNd_getRowPtr(postureArray, i);
      VecNd_copy(row, cSingle->getGraph()->q->ele, cSingle->getGraph()->dof);
    }
  }

  // Create the controller with a clone of the original controller for
  // each posture.
  // We change the initial state to the q0 state. That's the one that is
  // set from the model_state default in the graph's xml file.
  MatNd* q0 = MatNd_createLike(cSingle->getGraph()->q);
  RcsGraph_getDefaultState(cSingle->getGraph(), q0);
  ControllerBase* cNaiive = createGraph(cSingle, postures, offset, q0);
  MatNd_destroy(q0);

  REXEC(1)
  {
    int nw = 0, ne = 0;
    RcsGraph_check(cNaiive->getGraph(), &ne, &nw);
    RLOG(1, "Graph check: %d warnings, %d errors", nw, ne);
    RLOG(1, "Writing graph file");
    cNaiive->toXML("cNaiive.xml");
    RLOG(1, "Writing dot file");
    RcsGraph_writeDotFile(cNaiive->getGraph(), "gNaiive.dot");
    RCHECK(ne == 0);
  }

  // Acquire the activation vector from the xml file so that we can also
  // deal with inactive tasks.
  RLOG(1, "Reading activations");
  MatNd* aSingle = MatNd_create(cSingle->getNumberOfTasks(), 1);
  cSingle->readActivationsFromXML(aSingle);

  // Create activation vector for the augmented controller by appending the
  // activation vector to itself until the dinemsions match.
  MatNd* aNaiive = MatNd_clone(aSingle);

  for (unsigned int i = 0; i < postures->m - 1; ++i)
  {
    MatNd_appendRows(aNaiive, aSingle);
  }

  RCHECK_MSG(aNaiive->m == cNaiive->getNumberOfTasks(), "%u   %zu",
             aNaiive->m, cNaiive->getNumberOfTasks());


  // Relax constraints
  ControllerBase* cRelaxed = new ControllerBase(*cNaiive);
  MatNd* aRelaxed = MatNd_clone(aNaiive);
  MatNd* x_curr = MatNd_create(cRelaxed->getTaskDim(), 1);
  cRelaxed->computeX(x_curr);

  for (size_t i = 0; i < adja.size(); ++i)
  {
    relaxAll(cRelaxed, adja[i], aRelaxed);
  }

  // Converge to minimum of decoupled and relaxed poses
  convergeTaskConstraints(cRelaxed, aRelaxed, x_curr);
  cRelaxed->computeX(x_curr);
  MatNd_resizeCopy(x_relaxed, x_curr);

  // We return the sequence controller that has not been relaxed, long with the
  // activations and x_des values according to the relaxation.
  Result res;
  res.controller = cNaiive;
  res.activation = aNaiive;

  // Here we perform the averaging and update the x_relaxed vector. It can be
  // augmented with other entries before being passed into createNaiive_2.
  for (size_t i = 0; i < adja.size(); ++i)
  {
    RLOG_CPP(0, "Adjacency " << i);
    computeAverageXdes(cRelaxed, adja[i], aRelaxed, x_relaxed, avgSeq);
  }

  MatNd_destroy(aRelaxed);
  delete cRelaxed;

  // Returning here returns a sequence with each step locally converged to its
  // minimum.
  return res;
}

void PoseGraph::createNaiive_2(ControllerBase* cNaiive,
                               const MatNd* aNaiive,
                               const MatNd* x_des)
{
  // Converge to minimum of averaged, non-relaxed poses
  convergeTaskConstraints(cNaiive, aNaiive, x_des);
  cNaiive->printX(x_des, aNaiive);
}

}   // namespace Rcs
