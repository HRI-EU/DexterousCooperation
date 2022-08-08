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

#include <algorithm>




namespace Rcs
{

/*******************************************************************************
 *
 ******************************************************************************/
ControllerBase* PoseGraph::create(const ControllerBase* src,
                                  const std::vector<Adjacency>& adjacencies,
                                  const MatNd* postures,
                                  const double offset[3])
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
  ControllerBase* controller = createGraph(src, postures, offset);
  REXEC(1)
  {
    int nw = 0, ne = 0;
    RcsGraph_check(controller->getGraph(), &ne, &nw);
    RLOG(1, "Graph check: %d warnings, %d errors", nw, ne);
    RCHECK(ne == 0);
    RLOG(1, "Writing graph file");
    controller->toXML("cMultiplied.xml");
    RLOG(1, "Writing dot file");
    RcsGraph_writeDotFile(controller->getGraph(), "gMultiplied.dot");
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
  eraseInactiveTasks(controller, activations);

  // Write config files
  REXEC(1)
  {
    RcsGraph_writeXmlFile(controller->getGraph(), "PoseGraph.xml");
  }

  controller->toXML("cPoseGraph.xml", activations);

  // Clean up
  MatNd_destroy(postureArray);
  MatNd_destroy(src_activations);
  MatNd_destroy(activations);

  return controller;
}

/*******************************************************************************
 * Constructs a new graph with as many poses as in there are rows in the
 * postures array.
 ******************************************************************************/
ControllerBase* PoseGraph::createGraph(const ControllerBase* src,
                                       const MatNd* postures,
                                       const double offset[3])
{
  ControllerBase ci(*src);
  ControllerBase* controller = new ControllerBase(*src);

  HTr A_offset;
  HTr_setIdentity(&A_offset);

  RLOG_CPP(1, "Creating " << postures->m << " postures");

  for (size_t i=0; i<postures->m; ++i)
  {
    MatNd q_i = MatNd_getRowViewTranspose(postures, i);
    RcsGraph_setState(ci.getGraph(), &q_i, NULL);

    if (i>0)
    {
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
  std::vector<size_t> tix;

  RCHECK(activation->m==controller->getNumberOfTasks());

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
    RLOG_CPP(1, "Parent-body " << parentName << ": Connecting " << jBdy.size()
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

    std::string old = connection.originalTask;
    std::string relax = connection.relaxedTask;

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



}   // namespace Rcs
