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

#ifndef RCS_WHEELOBJECTMODEL_H
#define RCS_WHEELOBJECTMODEL_H

#include "EntityBase.h"
#include "ObjectModel.h"
#include "WheelStrategy7D.h"

#include <ControllerBase.h>

#include <map>


namespace Rcs
{

/*! \brief WheelObjectModel implements ObjectModel to define interaction
 *         capabilities and behavior for working with the wheel object. It
 *         defines a state space, possible state transitions, intention models,
 *         how to synchronize execution with a human partner, what sensor cues
 *         are of interest, and what visualizations should be shown. It offers
 *         the capability to calculate all of these dependent on the current
 *         state. An ObjectModel can be used to configure various components
 *         such as ManipulationComponent, StateEstimatorComponent, and
 *         MonitorComponent.
 */
class WheelObjectModel : public ObjectModel
{

public:

  /*! \brief WheelObjectModel implements ObjectModel to define interaction
   *         capabilities and behavior for working with the wheel object. It
   *         defines a state space, possible state transitions, intention
   *         models, how to synchronize execution with a human partner, what
   *         sensor cues are of interest, and what visualizations should be
   *         shown. It offers the capability to calculate all of these dependent
   *         on the current state. An ObjectModel can be used to configure
   *         various components such as ManipulationComponent,
   *         StateEstimatorComponent, and MonitorComponent.
   *
   *  \param[in] controller   Controller to clone its own graph from
   *  \param[in] entity       EntityBase that can be passed to components that
   *                          need to publish (e.g. visualization in
   *                          triggerExecutionVisualization())
   */
  WheelObjectModel(const ControllerBase* controller, EntityBase* entity=NULL);
  ~WheelObjectModel();

  /*! \brief Provides an empty state with the right dimensionality for this
   *         object type.
   *
   *  \returns          vector of ints with right dimensionality
   */
  std::vector<int> getEmptyState();
  /* std::vector<double> getComplianceWrench2(const double xd_r[3], */
  /*                                         const double xd_l[3]) const; */

  bool isStateValid(std::vector<int> state) const;
  int getStateDimension() const;

  /*! \brief Computes the discretized state based on the given graph.
   *
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  std::vector<int> getCurrentState(const RcsGraph* graph);

  /*! \brief Computes the discretized state based on the given graph. In
   *         addition, the offset between continuous state and the discretize
   *         state is published calculated.
   *
   *  \param[out] offsets       Variable to hold the calculate offsets
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  std::vector<int> getCurrentState(std::vector< std::vector<double> >& offsets,
                                   const RcsGraph* graph);

  /*! \brief Determines what the next goal should be based on a state and an
   *         intention.
   *
   *  \param[in] currentState          State
   *  \param[in] intention_id          Intention id
   *  \returns                         Goal state
   */
  std::vector<int> getGoalFromIntention(std::vector<int> currentState,
                                        int intention_id) const;

  /*! \brief Returns the closest discrete state (as defined by WheelObjectModel)
   *         to the current full continuous state of the system.
   *
   *  \param[in] state_in            State
   *  \returns                       Closest discrete state
   */
  virtual std::vector<int> getClosestCanonicalState(std::vector<int> state_in) const;


  /*! \brief Calculates the time to completion (TTC) for the transition between
   *         two states given a base TTC.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \param[in] baseTtc               Base TTC (duration)
   *  \returns                         TTC for transition
   */
  double getTtc(std::vector<int> stateFrom, std::vector<int> stateTo,
                double baseTtc) const;

  /*! \brief Returns if a transition requires a confirmation before being
   *         executed.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \returns                         Boolean (true if confirmation is needed)
   */
  bool confirmationNeeded(std::vector<int> stateFrom,
                          std::vector<int> stateTo) const;


  /*! \brief Trigger any visualizations that might be necessary when a
   *         confirmation has been triggered.
   *
   *  \param[in] state                State
   *  \param[in] id                   ID of confirmation
   */
  void triggerConfirmationVisualization(std::vector<int> state, int id) const;

  /*! \brief Trigger any visualizations that might be necessary when an
   *         action/transition is executed.
   *
   *  \param[in] from                Start state
   *  \param[in] to                  Stop state
   *  \param[in] graph               Graph
   */
  void triggerExecutionVisualization(std::vector<int> from,
                                     std::vector<int> to,
                                     RcsGraph* graph) const;


  /*! \brief Uses the desired and current graph to calculate all sensor signals
   *         that are needed by any MonitorRequest defined in this ObjectModel.
   *         This function contains how the sensor signals are computed from
   *         the graphs.
   *
   *  \param[in] desiredGraph    Desired graph
   *  \param[in] currentGraph    Current graph
   *  \returns                   Hashmap with ID and value of all sensor cues
   */
  std::map<std::string, double> computeSensorSignals(RcsGraph* desiredGraph,
                                                     RcsGraph* currentGraph);

  /************************************************************************
   * Monitors
   ************************************************************************/

  /*! \brief Get all MonitorRequest that are necessary to estimating intentions
   *         for a state. All possibly recognizable intentions and the
   *         respective sensor cues are defined here.
   *
   *  \param[in] state    State
   *  \returns            List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureMonitorsForState(std::vector<int> state) const;

  /*! \brief Get all MonitorRequest that are necessary to detect confirmation
   *         to start a waiting transition. All required confirmations and
   *         respective sensor cues are defined here.
   *
   *  \param[in] startState   Start state
   *  \param[in] endState     Stop state
   *  \returns                List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureConfirmationMonitorsForTransition(std::vector<int> startState,
      std::vector<int> endState) const;

  /*! \brief Get all MonitorRequest that are necessary to monitor the progress
   *         of a transition (to be able to detect if things go as planned).
   *
   *  \param[in] startState  Start state
   *  \param[in] endState    Stop state
   *  \returns               List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureMonitorsForTransition(std::vector<int> startState,
                                                             std::vector<int> endState) const;

  /************************************************************************
   * Action Progress Graphs
   ************************************************************************/

  /*! \brief Returns all ActionProgressGraph for all defined transitions.
   *
   *  \returns                       List of ActionProgressGraph
   */
  virtual std::vector<std::shared_ptr<ActionProgressGraph> > getActionProgressGraphs() const;

  /*! \brief Create the ActionProgressGraph that will be used for monitoring
   *         action execution of regrasp actions
   *
   *  \param[in] rightHandRegrasp  Boolean to create graphs for left or right
   *                               hand regrasps
   *  \returns                     ActionProgressGraph for regrasp transistions
   */
  std::shared_ptr<ActionProgressGraph> createRegraspAPG(bool rightHandRegrasp) const;

  /*! \brief Create the ActionProgressGraph that will be used for monitoring
   *         action execution of rotation of the wheel (roll) actions
   *
   *  \returns             ActionProgressGraph for roll transistions
   */
  std::shared_ptr<ActionProgressGraph> createRollObjectAPG() const;

  /*! \brief Create the ActionProgressGraph that will be used for monitoring
   *         action execution of rotation of the wheel (flip) actions
   *
   *  \returns            ActionProgressGraph for flip transistions
   */
  std::shared_ptr<ActionProgressGraph> createFlipObjectAPG() const;

  /*! \brief Create the ActionProgressGraph that will be used for monitoring
   *         action execution of translating the wheel
   *
   *  \returns           ActionProgressGraph for move transistions
   */
  std::shared_ptr<ActionProgressGraph> createMoveObjectAPG() const;


  /************************************************************************
   * Transition Types
   ************************************************************************/

  /*! \brief Gets the type of transition for a pair of states for the Wheel
   *         object.
   *
   *  \param[in] s0                  Start state
   *  \param[in] s1                  Stop state
   *  \returns                       int (enum) of TransitionType
   */
  unsigned int getTransitionType(std::vector<int> s0,
                                 std::vector<int> s1) const;

  /*! \brief Gets a string representation for a transition type for the Wheel
   *         object.
   *
   *  \param[in] type                TransitionType enum
   *  \returns                       String name of type
   */
  std::string transitionTypeToString(unsigned int type) const;

  /*! \brief Get number of hand contact changes for a transition.
   *
   *  \param from                     Start state
   *  \param to                       Stop state
   *  \returns                        Number of contact changes
   */
  int getNumberOfContactChanges(std::vector<int> from,
                                std::vector<int> to) const;

  /************************************************************************
   * Intention Types
   ************************************************************************/

  /*! \brief Definition of enums for all possible intentions for the wheel
   *         object.
   */
  enum class Intention
  {
    ROTATE_RIGHT = 0,
    ROTATE_LEFT,
    FLIP_RIGHT,
    FLIP_LEFT,
    MOVE_UP,
    MOVE_DOWN,
    MOVE_LEFT,
    MOVE_RIGHT,
    NUM_INTENTIONS
  };

  /*! \brief Names for all possible intentions for the wheel object.
   */
  std::vector<std::string> intentionName =
  {
    "Rotate right",
    "Rotate left",
    "Flip right",
    "Flip left",
    "Move up",
    "Move down",
    "Move left",
    "Move right"
  };

  /*! \brief Pointer to the ExplorationStrategy for the wheel object
   */
  std::shared_ptr<WheelStrategy7D> wheelExplorer;

private:

  /*!
   *  \brief Values for high compliance of the robot arms
   */
  /* std::vector<double> complianceHigh; */

  /*!
   *  \brief Values for low compliance of the robot arms
   */
  /* std::vector<double> complianceLow; */

  /*!
   *  \brief Velocity to which arm compliance is being scaled
   */
  /* double complianceVelocityMax; */

private:

  void initCanonicalStates();

  ControllerBase controller;

  std::vector< std::vector<int> > canonicalStates;
  std::vector<HTr> sState;
  std::map<std::string, double> sensorData;

  EntityBase* entity;
};

}

#endif //RCS_OBJECT_MODEL_H
