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

#ifndef RCS_ACTIONPROGRESSSTATE_H
#define RCS_ACTIONPROGRESSSTATE_H

#include "MonitorRequest.h"

#include <memory>
#include <vector>
#include <SearchNode.h>

namespace Rcs
{


//forward declarations
class ActionProgressGraph;
class ActionProgressTransition;

class ActionProgressState;
typedef std::shared_ptr<ActionProgressState> ActionProgressState_ptr;
typedef std::weak_ptr<ActionProgressState> ActionProgressState_wptr;


/*!
 *  \brief An ActionProgressState is a state in an ActionProgressGraph. The states of a graph represent various
 *         phases of the execution with respect to the expected normal behavior and anticipated possible problem cases.
 *         Each state has a list of outgoing transitions based on different events that can occur. For example, the
 *         state could change from "moving normally" to "unintended collision" if a large external force has been
 *         sensed.
 *         For each transition, there can be several sensor cues indicating that the transition is happening. These
 *         sensor cues and conditions upon them are provided as MonitorRequest. By submitting them to a
 *         MonitorComponent they are constantly monitored. When all conditions of the MonitorRequest are fulfilled,
 *         an appropriate event is published and the ActionProgressState is changed to the new ActionProgressState.
 *         Therefore, each state maintains a list of MonitorRequest to cover all outgoing transitions.
 */
class ActionProgressState
{
public:

  /*!
   *  \brief Creates an empty ActionProgressState
   */
  ActionProgressState();
  ~ActionProgressState();

  //  void setPlanningState(Rcs::SearchNode_ptr planningState);

  /*!
   *  \brief Returns a list of all MonitorRequest to cover all outgoing transitions.
   *
   *  \returns                List of MonitorRequest
   */
  std::vector<MonitorRequest> getRequests();

  /*!
   *  \brief Prints the name and description of the state together will all in- and outgoing transitions
   *         (ActionProgressTransition) and all monitors (MonitorRequest).
   */
  void print();

  /*!
   *  \brief ID of the state
   */
  std::string id_;

  /*!
   *  \brief Description of the state
   */
  std::string description_;

  /*!
   *  \brief Pointer to the ActionProgressGraph that this state belongs to
   */
  std::weak_ptr<ActionProgressGraph> action_;

  /*!
   *  \brief List of all incoming transitions (ActionProgressTransition)
   */
  std::vector< std::shared_ptr<ActionProgressTransition> > transitions_in_;

  /*!
   *  \brief List of all outgoing transitions (ActionProgressTransition)
   */
  std::vector< std::shared_ptr<ActionProgressTransition> > transitions_out_;

  /*!
   *  \brief List of monitors (MonitorRequest) to cover sensor cues for all outgoing transitions (ActionProgressTransition)
   */
  std::vector< std::shared_ptr<MonitorRequest> > monitors_;

  //  std::shared_ptr<Rcs::SearchNode> planningState_;

  /*!
   *  \brief List of events to be emitted when the state is entered. Each event consists of a pair of strings,
   *         e.g. std::pair("myTopic", "myContent").
   *         To execute an event, string #1 is used as event topic, string #2 as message content. For the example
   *         this "myContent" would be published on topic "myTopic".
   */
  std::vector< std::pair<std::string, std::string> > enterEvents;

  //  void onEnter();

  /*!
   *  \brief Flag to determine if this is a goal state (successful or unsuccessful) meaning that execution of the
   *         action is done after reaching this state.
   */
  bool goal_;

  /*!
   *  \brief Flag to determine if a goal state indicates successful or unsuccessful termination of the action.
   */
  bool success_;

private:


};

}

#endif //RCS_ACTIONPROGRESSSTATE_H
