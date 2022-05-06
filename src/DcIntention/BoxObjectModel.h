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

#ifndef RCS_BOXOBJECTMODEL_H
#define RCS_BOXOBJECTMODEL_H

#include "EntityBase.h"
#include "ObjectModel.h"
#include "BoxStrategy5D.h"

#include <ControllerBase.h>

#include <map>



namespace Rcs
{

/*!
 *  \brief Helper function to perform modulo and ensure the result is positive.
 *
 *  \param[in] i              Operand 1
 *  \param[in] n              Operand 2
 *  \returns                  i % n
 */
inline int positive_modulo(int i, int n)
{
  return (n + (i % n)) % n;
}

/*!
 *  \brief BoxObjectModel implements ObjectModel to define interaction capabilities and behavior for working
 *         with the box object. It defines a state space, possible state transitions, intention models,
 *         how to synchronize execution with a human partner, what sensor cues are of interest, and what
 *         visualizations should be shown.
 *         It offers the capability to calculate all of these dependent on the current state.
 *         An ObjectModel can be used to configure various components such as ManipulationComponent,
 *         StateEstimatorComponent, and MonitorComponent.
 *
 *         The class publishes the following events:
 *         - Prompt.confirm: In getGoalFromIntention()
 *         - Prompt.lean_right: In getGoalFromIntention()
 *         - Prompt.lean_left: In getGoalFromIntention()
 *         - Prompt.clear: In getGoalFromIntention()
 *         - DesiredStateChanged: In triggerConfirmationVisualization(), triggerExecutionVisualization()
 *         - HoloUpdateSprite: In triggerExecutionVisualization()
 *         - HoloUpdateArrowPlain: In triggerExecutionVisualization()
 *
 */
class BoxObjectModel : public ObjectModel
{

public:

  /*!
   *  \brief BoxObjectModel implements ObjectModel to define interaction capabilities and behavior for working
   *         with the box object. It defines a state space, possible state transitions, intention models,
   *         how to synchronize execution with a human partner, what sensor cues are of interest, and what
   *         visualizations should be shown.
   *         It offers the capability to calculate all of these dependent on the current state.
   *         An ObjectModel can be used to configure various components such as ManipulationComponent,
   *         StateEstimatorComponent, and MonitorComponent.
   *
   *  \param[in] controller                 Controller to clone its own graph from
   *  \param[in] object                     Object type: box, cylinder, L-shape
   *  \param[in] numPhiDiscretizations      Number of discretizations along the 2D perimeter of the object
   *  \param[in] entity                     EntityBase that can be passed to components that need to publish
   *                                        (e.g. visualization in triggerExecutionVisualization())
   */
  BoxObjectModel(const ControllerBase* controller,
                 BoxStrategy5D::ObjectType object,
                 int numPhiDiscretizations,
                 Rcs::EntityBase* entity=NULL);
  ~BoxObjectModel();

  /*!
   *  \brief Provides an empty state with the right dimensionality for this object type.
   *
   *  \returns          vector of ints with right dimensionality
   */
  std::vector<int> getEmptyState();

  /*!
   *  \brief Computes the discretized state based on the given graph.
   *
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  std::vector<int> getCurrentState(const RcsGraph* graph);

  /*!
   *  \brief Computes the discretized state based on the given graph. In addition, the offset between continuous state
   *         and the discretize state is published calculated.
   *
   *  \param[out] offsets       Variable to hold the calculate offsets
   *  \param[in] graph          Input graph
   *  \returns                  State
   */
  std::vector<int> getCurrentState(std::vector< std::vector<double> >& offsets, const RcsGraph* graph);

  /*!
   *  \brief Determines what the next goal should be based on a state and an intention.
   *
   *  \param[in] currentState          State
   *  \param[in] intention_id          Intention id
   *  \returns                         Goal state
   */
  std::vector<int> getGoalFromIntention(std::vector<int> currentState, int intention_id) const;

  /*!
   *  \brief Returns the closest discrete state (as defined by BoxObjectModel) to the current full continuous
   *         state of the system.
   *
   *  \param[in] state_in            State
   *  \returns                       Closest discrete state
   */
  virtual std::vector<int> getClosestCanonicalState(std::vector<int> state_in) const;

  std::vector<int> getStableHandSupports(int phi) const;
  int getSupportPoint(int phi, bool left, bool robot = true, bool print = false) const;
  double getNextCanonicalAngle(double startAngle, int direction) const;
  bool isStateValid(std::vector<int> state) const;
  int getStateDimension() const;

  /*!
   *  \brief Calculates the time to completion (TTC) for the transition between two states given a base TTC.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \param[in] baseTtc               Base TTC (duration)
   *  \returns                         TTC for transition
   */
  double getTtc(std::vector<int> stateFrom, std::vector<int> stateTo, double baseTtc) const;

  /*!
   *  \brief Returns if a transition requires a confirmation before being executed.
   *
   *  \param[in] stateFrom             Start state
   *  \param[in] stateTo               Goal state
   *  \returns                         Boolean (true if confirmation is needed)
   */
  bool confirmationNeeded(std::vector<int> stateFrom, std::vector<int> stateTo) const;


  /*!
   *  \brief Trigger any visualizations that might be necessary when a confirmation has been triggered.
   *
   *  \param[in] state                State
   *  \param[in] id                   ID of confirmation
   */
  void triggerConfirmationVisualization(std::vector<int> state, int id) const;

  /*!
   *  \brief Trigger any visualizations that might be necessary when an action/transition is executed.
   *
   *  \param[in] from                Start state
   *  \param[in] to                  Stop state
   *  \param[in] graph               Graph
   */
  void triggerExecutionVisualization(std::vector<int> from, std::vector<int> to, RcsGraph* graph) const;


  /*!
   *  \brief Uses the desired and current graph to calculate all sensor signals that are needed by any MonitorRequest
   *         defined in this ObjectModel. This function contains how the sensor signals are computed from the graphs.
   *
   *  \param[in] desiredGraph        Desired graph
   *  \param[in] currentGraph        Current graph
   *  \returns                       Hashmap with ID and value of all sensor cues
   */
  std::map<std::string, double> computeSensorSignals(RcsGraph* desiredGraph, RcsGraph* currentGraph);



  std::vector<HTr> getRoboContacts3D() const
  {
    return contactPointsRobot;
  }
  std::vector<HTr> getPartnerContacts3D() const
  {
    return contactPointsPartner;
  }



  /************************************************************************
   * Monitors
   ************************************************************************/

  /*!
   *  \brief Get all MonitorRequest that are necessary to estimating intentions for a state. All possibly recognizable
   *         intentions and the respective sensor cues are defined here.
   *
   *  \param[in] state               State
   *  \returns                       List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureMonitorsForState(std::vector<int> state) const;

  /*!
   *  \brief Get all MonitorRequest that are necessary to detect confirmation to start a waiting transition. All
   *         required confirmations and respective sensor cues are defined here.
   *
   *  \param[in] startState          Start state
   *  \param[in] endState            Stop state
   *  \returns                       List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureConfirmationMonitorsForTransition(std::vector<int> startState, std::vector<int> endState) const;

  /*!
   *  \brief Get all MonitorRequest that are necessary to monitor the progress of a transition (to be able to detect if
   *         things go as planned).
   *
   *  \param[in] startState          Start state
   *  \param[in] endState            Stop state
   *  \returns                       List of MonitorRequest to send to MonitorComponent
   */
  std::vector<MonitorRequest> configureMonitorsForTransition(std::vector<int> startState, std::vector<int> endState) const;

  /************************************************************************
   * Action Progress Graphs
   ************************************************************************/

  /*!
   *  \brief Returns a list of all ActionProgressGraph for all defined transitions.
   *
   *  \returns                       List of ActionProgressGraph
   */
  virtual std::vector<std::shared_ptr<ActionProgressGraph> > getActionProgressGraphs() const;

  /*!
   *  \brief Create the ActionProgressGraph that will be used for monitoring action execution of regrasp actions
   *
   *  \param[in] rightHandRegrasp             Boolean to create graphs for left or right hand regrasps
   *  \returns                                ActionProgressGraph for regrasp transistions
   */
  std::shared_ptr<ActionProgressGraph>  createRegraspAPG(bool rightHandRegrasp) const;

  /*!
   *  \brief Create the ActionProgressGraph that will be used for monitoring action execution of box rotation actions
   *
   *  \returns                                ActionProgressGraph for rotate transistions
   */
  std::shared_ptr<ActionProgressGraph>  createRotateObjectAPG() const;

  /************************************************************************
   * Transition Types
   ************************************************************************/

  /*
   * \brief Enums for all possible types of transitions when handling the Box object.
   */
  enum TransitionType
  {
    // general
    None = 0,
    TransitionUndefined,
    // object and robo OR partner
    RotateObject,
    RegraspRight,
    RegraspLeft,
    MoveAll,
    // robo and partner
    BothRotateObject,
    RobotRegraspRight,
    RobotRegraspLeft,
    PartnerRegraspRight,
    PartnerRegraspLeft
  };

  /*!
   *  \brief Gets a string representation for a transition type for the Box object.
   *
   *  \param[in] type                TransitionType enum
   *  \returns                       String name of type
   */
  std::string transitionTypeToString(unsigned int type) const
  {
    switch (type)
    {
      case RotateObject:
        return "rotate object";
      case RegraspRight:
        return "regrasp right";
      case RegraspLeft:
        return "regrasp left";
      case MoveAll:
        return "move all";
      case BothRotateObject:
        return "rotate object";
      case RobotRegraspRight:
        return "regrasp right (robot)";
      case RobotRegraspLeft:
        return "regrasp left (robot)";
      case PartnerRegraspRight:
        return "regrasp right (human)";
      case PartnerRegraspLeft:
        return "regrasp left (human)";
      case None:
        return "no change in state";
      case TransitionUndefined:
        return "undefined";
      default:
        return "undefined other";
    }
  }

  //TODO: this part is not nice at all and really hacky! fix!
  /*!
   *  \brief Gets the type of transition for state with just robot or just human.
   *
   *  \param[in] phi0                Start: Box rotation
   *  \param[in] rh0                 Start: Right hand position
   *  \param[in] lh0                 Start: Left hand position
   *  \param[in] phi1                Stop: Box rotation
   *  \param[in] rh1                 Stop: Right hand position
   *  \param[in] lh1                 Stop: Left hand position
   *  \returns                       int (enum) of TransitionType
   */
  static unsigned int getTransitionTypeSingle(int phi0, int rh0, int lh0, int phi1, int rh1, int lh1);

  /*!
   *  \brief Gets the type of transition for a pair of states. Transition types are defined in the ObjectModel.
   *
   *  \param[in] from                Start state
   *  \param[in] to                  Stop state
   *  \returns                       int (enum) of TransitionType
   */
  unsigned int getTransitionType(std::vector<int> from, std::vector<int> to) const;

  /*!
   *  \brief (Static version) Gets the type of transition for a pair of states. Transition types are defined in the ObjectModel.
   *
   *  \param[in] s0                  Start state
   *  \param[in] s1                  Stop state
   *  \returns                       int (enum) of TransitionType
   */
  static unsigned int getTransitionTypeStatic(std::vector<int> s0, std::vector<int> s1);

  /*!
   *  \brief Gets the type of transition for state with full state and each parameter as individual input
   *
   *  \param[in] phi0                Start: Box rotation
   *  \param[in] ra0                 Start: Right robot hand position
   *  \param[in] la0                 Start: Left robot hand position
   *  \param[in] rp0                 Start: Right human hand position
   *  \param[in] lp0                 Start: Left human hand position
   *  \param[in] phi1                Stop: Box rotation
   *  \param[in] ra1                 Stop: Right robot hand position
   *  \param[in] la1                 Stop: Left robot hand position
   *  \param[in] rp1                 Stop: Right human hand position
   *  \param[in] lp1                 Stop: Left human hand position
   *  \returns                       int (enum) of TransitionType
   */
  static unsigned int getTransitionType(int phi0, int ra0, int la0, int rp0, int lp0,
                                        int phi1, int ra1, int la1, int rp1, int lp1);

  /************************************************************************
   * Intention Types
   ************************************************************************/

  /*!
   * \brief Definition of enums for all possible intentions for the box object.
   */
  enum class Intention
  {
    ROTATE_RIGHT = 0,
    ROTATE_LEFT,
    TILT_RIGHT,
    TILT_LEFT,
    NUM_INTENTIONS
  };

private:

  ControllerBase controller;
  std::shared_ptr<BoxStrategy5D> boxExplorer;

  double deltaPhi;

  Task* taskPhi;
  Task* taskPosR;
  Task* taskPosL;
  Task* taskPartnerPosR;
  Task* taskPartnerPosL;

  std::vector<HTr> contactPointsRobot;
  std::vector<HTr> contactPointsPartner;

  const std::vector<BoxStrategy5D::ContactPoint2D>* roboContacts;
  const std::vector<BoxStrategy5D::ContactPoint2D>* partnerContacts;

  std::map<std::string, double> sensorData;

  unsigned int numPhiDiscretizations;
  EntityBase* entity;
};

}

#endif //RCS_OBJECT_MODEL_H
