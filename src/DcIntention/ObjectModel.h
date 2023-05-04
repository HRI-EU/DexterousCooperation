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

#ifndef DC_OBJECTMODEL_H
#define DC_OBJECTMODEL_H

#include "ActionProgressGraph.h"
#include <Rcs_graph.h>



namespace Dc
{

/*!
 *  \brief ObjectModel is a prototype that can be implemented by other classes
 *         to define interaction capabilities and behavior for working with an
 *         object. (See examples BoxObjectModel and WheelObjectModel.) The model
 *         has to define a state space, possible state transitions, intention
 *         models, how to synchronize execution with a human partner, what
 *         sensor cues are of interest, and what visualizations should be shown.
 *         It offers the capability to calculate all of these dependent on the
 *         current state. An ObjectModel can be used to configure various
 *         components such as ManipulationComponent, StateEstimatorComponent,
 *         and MonitorComponent.
 */
class ObjectModel
{

public:

  /*! \brief ObjectModel is a prototype that can be implemented by other classes
   *         to define interaction capabilities and behavior for working with
   *         an object. (See examples BoxObjectModel and WheelObjectModel.) The
   *         model has to define a state space, possible state transitions,
   *         intention models, how to synchronize execution with a human
   *         partner, what sensor cues are of interest, and what visualizations
   *         should be shown. It offers the capability to calculate all of these
   *         dependent on the current state. An ObjectModel can be used to
   *         configure various components such as ManipulationComponent,
   *         StateEstimatorComponent, and MonitorComponent.
   */
  ObjectModel();

  virtual ~ObjectModel();

  /*! \brief Provides an empty state with the right dimensionality for this
   *         object type.
   *
   *  \returns          vector of ints with right dimensionality
   */
  virtual std::vector<int> getEmptyState() = 0;

  /*! \brief Computes the discretized state based on the given graph.
   *
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  virtual std::vector<int> getCurrentState(const RcsGraph* graph) = 0;

  /*! \brief Computes the discretized state based on the given graph. In
   *          addition, the offset between continuous state and the discretize
   *          state is published calculated.
   *
   *  \param[out] offsets       Variable to hold the calculate offsets
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  virtual std::vector<int>
  getCurrentState(std::vector<std::vector<double>>& offsets,
                  const RcsGraph* graph) = 0;

  /*! \brief Determines what the next goal should be based on a state and an
   *         intention.
   *
   *  \param[in] currentState          State
   *  \param[in] intention_id          Intention id
   *  \returns                         Goal state
   */
  virtual std::vector<int> getGoalFromIntention(std::vector<int> currentState,
                                                int intention_id) const = 0;

  /*! \brief Calculates the time to completion (TTC) for the transition between
   *         two states given a base TTC.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \param[in] baseTtc               Base TTC (duration)
   *  \returns                         TTC for transition
   */
  virtual double getTtc(std::vector<int> stateFrom, std::vector<int> stateTo,
                        double baseTtc) const;

  /*! \brief Returns if a transition requires a confirmation before being
   *         executed.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \returns                         Boolean (true if confirmation is needed)
   */
  virtual bool confirmationNeeded(std::vector<int> stateFrom,
                                  std::vector<int> stateTo) const;

  /*! \brief Trigger any visualizations that might be necessary when a
   *         confirmation has been triggered.
   *
   *  \param[in] state                State
   *  \param[in] id                   ID of confirmation
   */
  virtual void triggerConfirmationVisualization(std::vector<int> state,
                                                int id) const = 0;

  /*! \brief Trigger any visualizations that might be necessary when an
   *         action/transition is executed.
   *
   *  \param[in] from                Start state
   *  \param[in] to                  Stop state
   *  \param[in] graph               Graph
   */
  virtual void triggerExecutionVisualization(std::vector<int> from,
                                             std::vector<int> to,
                                             RcsGraph* graph) const = 0;


  /*! \brief Get all MonitorRequest that are necessary to estimating intentions
   *         for a state. All possibly recognizable intentions and the
   *         respective sensor cues are defined here.
   *
   *  \param[in] state       State
   *  \returns               List of MonitorRequest to send to MonitorComponent
   */
  virtual std::vector<MonitorRequest>
  configureMonitorsForState(std::vector<int> state) const = 0;

  /*! \brief Get all MonitorRequest that are necessary to detect confirmation
   *         to start a waiting transition. All required confirmations and
   *         respective sensor cues are defined here.
   *
   *  \param[in] startState   Start state
   *  \param[in] endState     Stop state
   *  \returns                List of MonitorRequest to send to MonitorComponent
   */
  virtual std::vector<MonitorRequest>
  configureConfirmationMonitorsForTransition(std::vector<int> startState,
                                             std::vector<int> endState) const = 0;

  /*! \brief Get all MonitorRequest that are necessary to monitor the progress
   *         of a transition (to be able to detect if things go as planned).
   *
   *  \param[in] startState   Start state
   *  \param[in] endState     Stop state
   *  \returns                List of MonitorRequest to send to MonitorComponent
   */
  virtual std::vector<MonitorRequest>
  configureMonitorsForTransition(std::vector<int> startState,
                                 std::vector<int> endState) const = 0;

  /*! \brief Uses the desired and current graph to calculate all sensor signals
   *         that are needed by any MonitorRequest defined in this ObjectModel.
   *         This function contains how the sensor signals are computed from
   *         the graphs.
   *
   *  \param[in] desiredGraph     Desired graph
   *  \param[in] currentGraph     Current graph
   *  \returns                    Hashmap with ID and value of all sensor cues
   */
  virtual std::map<std::string, double>
  computeSensorSignals(RcsGraph* desiredGraph,
                       RcsGraph* currentGraph) = 0;

  /*! \brief Gets the type of transition for a pair of states. Transition types
   *         are defined in the ObjectModel.
   *
   *  \param[in] s0                  Start state
   *  \param[in] s1                  Stop state
   *  \returns                       int (enum) of TransitionType
   */
  virtual unsigned int getTransitionType(std::vector<int> s0,
                                         std::vector<int> s1) const = 0;

  /*! \brief Gets a string representation for a transition type. Transition
   *         types and names are defined in the ObjectModel.
   *
   *  \param[in] type                TransitionType enum
   *  \returns                       String name of type
   */
  virtual std::string transitionTypeToString(unsigned int type) const = 0;

  /*! \brief Checks the validity of a state.
   *
   *  \param[in] state               State
   *  \returns                       True if valid
   */
  virtual bool isStateValid(std::vector<int> state) const = 0;

  /*! \brief Returns the closest discrete state (as defined by this ObjectModel)
   *         to the current full continuous state of the system.
   *
   *  \param[in] state_in            State
   *  \returns                       Closest discrete state
   */
  virtual std::vector<int>
  getClosestCanonicalState(std::vector<int> state_in) const = 0;

  /*! \brief Returns all ActionProgressGraph for all defined transitions.
   *
   *  \returns                       List of ActionProgressGraph
   */
  virtual std::vector<std::shared_ptr<ActionProgressGraph> >
  getActionProgressGraphs() const = 0;


  /*! \brief Returns the dimensionality of the state defined for this object.
   *
   *  \returns                       Dimensionality of state
   */
  virtual int getStateDimension() const = 0;
};

}

#endif // DC_OBJECTMODEL_H
