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

#ifndef RCS_ACTIONPROGRESSGRAPH_H
#define RCS_ACTIONPROGRESSGRAPH_H

#include "MonitorRequest.h"



namespace Rcs
{

class ActionProgressState;
class ActionProgressTransition;


/*!
 *  \brief The ActionProgressGraph (APG) models the behavior of a single type of action with respect to expected sensor
 *         cues for succeeding or failing outcomes. Each APG has a string ID that needs to match the modeled
 *         action/state transition and is used to find the corresponding APGs later when executing actions.
 *         The nodes in the graph are of type ActionProgressState and the edges are of type
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
 */
class ActionProgressGraph
{

public:
  /*!
   *  \brief Creates an empty ActionProgressGraph
   */
  ActionProgressGraph();

  /*!
   *  \brief Creates an empty ActionProgressGraph, but sets ID and description according to the input parameters.
   *
   *  \param[in] id                 String ID of the action to be modeled
   *  \param[in] desc               Description of the action to be modeled
   */
  ActionProgressGraph(std::string id, std::string desc = "");
  ~ActionProgressGraph();

  /*!
   *  \brief Sets start and goal state for the current instantiation of this action
   *
   *  \param[in] start              Start state
   *  \param[in] stop               Goal state
   */
  void setStartStop(const std::vector<int>& start,
                    const std::vector<int>& stop);

  /*!
   *  \brief Prints the name and description of the APG together will all states (ActionProgressState) and all
   *         monitors (MonitorRequest).
   */
  void print();

  /*!
   *  \brief Outputs a graph representation of the APG in .dot format to the provided file.
   *
   *  \param[in] filename           Filename for the .dot file
   */
  void writeDot(std::string filename);

  //bool updateMonitor(std::string id, double value);

  /*!
   *  \brief ID of the modeled action
   */
  std::string id_;

  /*!
   *  \brief Description of the modeled action
   */
  std::string description_;

  /*!
   *  \brief List of the states of the APG (of type ActionProgressState)
   */
  std::vector< std::shared_ptr<ActionProgressState> > states_;

  /*!
   *  \brief List of the transitions of the APG (of type ActionProgressTransition)
   */
  std::vector< std::shared_ptr<ActionProgressTransition> > transitions_;

  /*!
   *  \brief List of the monitors of the APG (of type MonitorRequest)
   */
  std::vector< std::shared_ptr<MonitorRequest> > monitors_;

  /*!
   *  \brief Start state of the current instantiation of this action
   */
  std::vector<int> planningStart_;

  /*!
   *  \brief Stop state of the current instantiation of this action
   */
  std::vector<int> planningStop_;

  /*!
   *  \brief Expected duration of the current instantiation of the action
   */
  double duration_;
};

typedef std::shared_ptr<ActionProgressGraph> Apg_ptr;
typedef std::weak_ptr<ActionProgressGraph> Apg_wptr;

}

#endif //RCS_ACTIONPROGRESSGRAPH_H
