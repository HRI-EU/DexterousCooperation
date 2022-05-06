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

#ifndef RCS_ACTIONPROGRESSTRANSITION_H
#define RCS_ACTIONPROGRESSTRANSITION_H

#include "MonitorRequest.h"

#include <memory>
#include <vector>


namespace Rcs
{

//forward declarations
class ActionProgressState;

class ActionProgressTransition;
typedef std::shared_ptr<ActionProgressTransition> Transition_ptr;
typedef std::weak_ptr<ActionProgressTransition> Transition_wptr;

/*!
 *  \brief An ActionProgressTransition represents the transition between two ActionProgressState in an
 *         ActionProgressGraph. The ActionProgressState of a graph represent various
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
class ActionProgressTransition
{

public:

  /*!
   *  \brief Creates an empty ActionProgressTransition
   */
  ActionProgressTransition();

  /*!
   *  \brief Creates an ActionProgressTransition and sets ID, description, start and goal state according to the
   *         input parameters.
   *
   *  \param[in] id                 String ID
   *  \param[in] start              Start state
   *  \param[in] stop               Stop state
   *  \param[in] description        Description of the transition
   */
  ActionProgressTransition(int id,
                           std::shared_ptr<ActionProgressState> start,
                           std::shared_ptr<ActionProgressState> stop,
                           std::string description = "");
  ~ActionProgressTransition();

  /*!
   *  \brief Prints the start and goal states of the transition.
   */
  void print();

  /*!
   *  \brief ID of the transition
   */
  int id_;

  /*!
   *  \brief Description of the transition
   */
  std::string description_;

  /*!
   *  \brief Start state
   */
  std::weak_ptr<ActionProgressState> start_;

  /*!
   *  \brief Stop state
   */
  std::weak_ptr<ActionProgressState> stop_;

  /*!
   *  \brief Allows to define the minimum duration that has to elapse since beginning of action before this transition
   *         can be taken. As the action duration is not predetermined, this value will be multiplied with the expected
   *         duration of the action to get the correct value. (e.g. deltaTmin = 0.1 with a action duration of 20s will
   *         result in a minimum duration of 0.1 x 20s = 2s.)
   */
  double deltaTmin_;

  /*!
   *  \brief Allows to define the maximum duration that may elapse since beginning of action until this transition
   *         is taken. If the duration is exceeded then the transition cannot be taken any more. As the action
   *         duration is not predetermined, this value will be multiplied with the expected
   *         duration of the action to get the correct value. (e.g. deltaTmax = 1.1 with a action duration of 20s will
   *         result in a minimum duration of 1.1 x 20s = 22s.)
   */
  double deltaTmax_;

  /*!
   *  \brief List of the monitors (MonitorRequest) for this transition
   */
  std::vector<std::shared_ptr<MonitorRequest> > monitors_;

  //  std::vector<double> monitor_values_;

private:

};

}

#endif //RCS_ACTIONPROGRESSTRANSITION_H
