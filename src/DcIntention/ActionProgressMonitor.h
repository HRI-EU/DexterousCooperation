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

#ifndef RCS_ACTIONPROGRESSMONITOR_H
#define RCS_ACTIONPROGRESSMONITOR_H

#include "ActionProgressGraph.h"
#include "ActionProgressState.h"
#include "ActionProgressTransition.h"
#include "EntityBase.h"

#include <map>

namespace Rcs
{

typedef std::vector<int> SearchNode_ptr;

/*!
 *  \brief The ActionProgressMonitor monitors the execution of actions with respect to the expected progress and
 *         outcome. This enables to estimate if normal performance is achieved or if something is going wrong.
 *         Depending on the granularity of the graph, the system might be able to react and compensate to small
 *         errors early on and thus avoid failures. This also allows to detect changed intentions (or false-positive
 *         intention estimates) quickly and adapt the behaviour.
 *
 *         The ActionProgressMonitor gets filled with ActionProgressGraph (APG) for each known action type.
 *         Each APG has a string ID that needs to match the modeled action/state transition and is used to find the
 *         corresponding APGs later when executing actions.
 *         The APG models the behavior of a single type of action with respect to expected sensor cues for succeeding
 *         or failing outcomes. The nodes in the graph are of type ActionProgressState and the edges are of type
 *         ActionProgressTransition. These states represent various phases of the execution with respect to
 *         the expected normal behavior and anticipated possible problem cases. Each state has a list of outgoing
 *         transitions based on different events that can occur. For example, the state could change
 *         from "moving normally" to "unintended collision" if a large external force has been sensed.
 *
 *         For each transition, there can be several sensor cues indicating that the transition is happening. These
 *         sensor cues and conditions upon them are provided as MonitorRequest. By submitting them to a
 *         MonitorComponent they are constantly monitored. When all conditions of the MonitorRequest are fulfilled,
 *         an appropriate event is published and the ActionProgressState is changed to the new ActionProgressState.
 *         More information on how to define the sensor cues and conditions is documented in the MonitorRequest.
 *
 *         The ActionProgressGraph for each action are usually created in the application
 *         (e.g. \ref BoxObjectModel::getActionProgressGraphs or \ref WheelObjectModel::getActionProgressGraphs) or
 *         can be read from an object or task model.
 *
 *         The class publishes the following events:
 *         - StopActions: In "triggerStateChangeActions"
 *         - ResumePlan: In "triggerStateChangeActions"
 *         - PausePlan: In "triggerStateChangeActions"
 *         - [generic topic possible]: In "triggerStateChangeActions"
 *
 *         The class does NOT subscribe to any events, but is interfaced through function calls.
 */

class ActionProgressMonitor
{

public:
  /*!
   *  \brief Creates and empty ActionProgressMonitor without any models (ActionProgressGraph) yet.
   */
  ActionProgressMonitor();
  ~ActionProgressMonitor();


  // bool update(double timeToGoal);

  /*!
   *  \brief Configure the ActionProgressMonitor to load action with provided ID, set the action goal, duration,
   *         and start time.
   *
   *  \param[in] action                 Action to monitor
   *  \param[in] goal                   Action goal
   *  \param[in] duration               Expected duration
   *  \param[in] startTime              Start time
   *  \returns                          True if action was found
   */
  bool monitor(std::string action, Rcs::SearchNode_ptr goal, double duration, double startTime);

  /*!
   *  \brief Configure the ActionProgressMonitor to load action with provided ID, set the action goal, duration,
   *         and start time.
   *
   *  \param[in] action                 Action to monitor
   *  \param[in] goal                   Action goal
   *  \param[in] duration               Expected duration
   *  \param[in] startTime              Start time
   *  \returns                          True if action was found
   */
  bool monitor(Apg_ptr action, Rcs::SearchNode_ptr goal, double duration, double startTime);

  /*!
   *  \brief Add new actions to the list of known actions.
   *
   *  \param[in] actions_new            APG for actions to add to list of known actions
   */
  void addActions(std::vector<Apg_ptr> actions_new);

  /*!
   *  \brief Get the MonitorRequest that are necessary for monitoring action progress based on the current state.
   *
   *  \returns                          List of MonitorRequest
   */
  std::vector<MonitorRequest> getRequests();

  /*!
   *  \brief Print current state [Currently not implemented]
   */
  void print();

  /*!
   *  \brief Stop tracking the current action
   */
  void stop();

  /*!
   *  \brief Callback function if a MonitorRequest is fulfilled and a state transition will happen. This function is
   *         called from outside and the class does not directly subscribe to any events.
   *
   *  \param[in] id                     ID of the transition that belonged to the triggered MonitorRequest
   *  \returns                          True for valid ID
   */
  bool onMonitorTrigger(int id);

  /*!
   *  \brief Trigger any activities that might have to happen when the state changes (specified in the APG)
   *
   *  \param[in] entity                 Pointer to the EntityBase to provide access in case something needs to be
   *                                    published.
   */
  void triggerStateChangeActions(EntityBase* entity);

  /*!
   *  \brief Start time of the current action
   */
  double actionStartTime_;

  /*!
   *  \brief ActionProgressGraph for the current action
   */
  Apg_ptr currentAction_;

  /*!
   *  \brief Current state (ActionProgressState) during execution of current action.
   */
  ActionProgressState_ptr currentState_;

  /*!
   *  \brief List of ActionProgressGraph for known actions
   */
  std::vector<std::shared_ptr<ActionProgressGraph> > actions_;

private:

};

}

#endif //RCS_ACTIONPROGRESSMONITOR_H
