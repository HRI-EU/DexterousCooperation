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

#ifndef RCS_MANIPULATIONCOMPONENT_H
#define RCS_MANIPULATIONCOMPONENT_H

#include "ComponentBase.h"
#include "SearchNode.h"
#include "ActionProgressMonitor.h"
#include "ObjectModel.h"

#include <Rcs_math.h>
#include <Rcs_graph.h>

#include <memory>
#include <queue>
#include <limits>



namespace Rcs
{

/*!
 *  \brief ManipulationComponent handles interaction with different objects. All of its behavior is
 *         dependent on the provided ObjectModel which defines states, transitions between different states,
 *         and required sensor cues and provided feedback for the human operator.
 *
 *         The class publishes the following events:
 *         - ClearTrajectory: In the subscribers "PausePlan", "StopActions", and "EmergencyStop"
 *         - MoveTo: In the subscriber of "RecoverToClosestState"
 *         - SetTextLine: In many place to print things on the HUD
 *         - Display: In "updateGraph"
 *         - PlanAndExecute: in function "planSequence"
 *         - Plan: in function "planSequence"
 *         - MotionType: in function "executeAndMonitorSequence"
 *         - ClearMonitors: in functions "executeAndMonitorSequence", "startNextAction", "toIdle",
 *         in subscribers "Progress"
 *         - RequestMonitor: in functions "executeAndMonitorSequence", "startNextAction", "toIdle",
 *         in subscribers "Progress", "Intention",
 *         - ExecutePlan: in function "executeAndMonitorSequence"
 *
 *         The class subscribes to the following events:
 *         - UpdateGraph: Updates the own graph.
 *         - EmergencyStop: Halts the system in case of an emergency stop.
 *         - EmergencyRecover: Returns the system to runnable after an emergency stop.
 *         - PausePlan: Pauses all behavior.
 *         - ResumePlan: Resumes behavior from pause.
 *         - StopActions: Cancels all plans and return to idle state.
 *         - RecoverToClosestState: Moves robot to precisely match the closest discretized state.
 *         - PlanResult: Receives the sequence of actions from the planner
 *         - MotionState: Receives when the robot starts and stops movement for actions
 *         - Intention: Receives intention cues (eg. from MonitorComponent) and starts actions accordingly
 *         - Confirmation: Receives confirmation cues for actions whose execution need to be
 *         synchronized with the human partner.
 *         - Progress: Receives cues regarding the progress of the current action.
 *         - Status: Prints the current plan and location within.
 *         - State: Receives the current and desired state of the robot (e.g. from StateEstimator).
 *         - SetTTC: Sets the base time to completion (TTC) for actions. Other actions might be scaled
 *         by this value.
 *         - ToggleSkipPartner: Allows to enable skipping execution of the human partner. Useful for
 *         switching between simulation and real execution.
 *
 */
class ManipulationComponent : public ComponentBase
{
  /*!
   *  \brief InteractionState represents the internal state of the ManipulationComponent
   */
  enum class InteractionState
  {
    IDLE = 0,
    WAITING_FOR_PLANNER,
    START_NEXT_ACTION,
    WAITING_FOR_CONFIRMATION,
    IN_PROGRESS,
    STOPPING,
    ESTOPPED,
    PAUSED,
    NUM_STATES
  };

  /*!
   *  \brief Helper function to provide a string representation for an InteractionState
   */
  static std::string InteractionStateStr(InteractionState s)
  {
    if (s == InteractionState::IDLE)
    {
      return "Idle";
    }
    if (s == InteractionState::WAITING_FOR_PLANNER)
    {
      return "Waiting_For_Planner";
    }
    if (s == InteractionState::START_NEXT_ACTION)
    {
      return "StartNextAction";
    }
    if (s == InteractionState::WAITING_FOR_CONFIRMATION)
    {
      return "Waiting_For_Confirmation";
    }
    if (s == InteractionState::IN_PROGRESS)
    {
      return "In_Progress";
    }
    if (s == InteractionState::STOPPING)
    {
      return "Stopping";
    }
    if (s == InteractionState::ESTOPPED)
    {
      return "e-Stopped";
    }
    if (s == InteractionState::PAUSED)
    {
      return "Paused";
    }


    return "Unknown Interaction State";
  }

  enum class Hands
  {
    RIGHT = 0,
    LEFT,
    NUM_HANDS
  };

  enum class RotationDirection
  {
    RIGHT,
    LEFT
  };



public:

  /*!
   *  \brief ManipulationComponent handles interaction with different objects. All of its behavior is
   *         dependent on the provided ObjectModel which defines states, transitions between different states,
   *         and required sensor cues and provided feedback for the human operator.
   *  \param[in] entity       Pointer to the EntityBase to be used
   *  \param[in] graph_       A graph to clone for internal use
   *  \param[in] objModel_    The ObjectModel that defines all behavior
   */
  ManipulationComponent(Rcs::EntityBase* entity, const RcsGraph* graph_, std::shared_ptr<ObjectModel> objModel);
  ~ManipulationComponent();

  /*!
   *  \brief Provides the name of the component.
   */
  std::string getName() const
  {
    return "Manipulation Component";
  }

  /*!
   * \brief Prints the status of the component including current internal state and progress in a planned action sequence.
   *        This function will be called on event "Status".
   */
  void status() const;

  /*! \brief Sets the lowest desired object point. A possibility to control the grasp height instead
   *         of the height of the center of mass. (Not used in current version.)
   *
   *   \param[in] pos      Desired lowest object position
   */
  void setObjectBottomPos(double* pos);

  /*! \brief Request the lowest desired object point.
   *
   *   \param[out] pos      Desired lowest object position
   */
  void getObjectBottomPos(double* pos) const;

  /*! \brief Sets a flag to automatically start actions that normally require confirmation.
   *
   *  \param[in] enable     Boolean - true to enable automatic start
   */
  void setAutoConfirm(bool enable);

  /*! \brief Gets the flag that determines if actions that normally require confirmation are started automatically
   *
   *  \return     Boolean - true if automatic start enabled
   */
  bool getAutoConfirm();

private:

  /*!
   *  \brief Updates the internal graph from the input graph.
   *         This function is called on event "UpdateGraph".
   *  \param[in] graph_   Input graph to update from
   */
  void updateGraph(RcsGraph* graph_);

  /*! \brief Issues a request to find a plan on topic "Plan" or to plan and execute on topic "PlanAndExecute".
   *         The plan start from curState and has to reach state 'stop'.
   *
   *  \param[in] curState       Start state (usually but not necessarily the current state)
   *  \param[in] stop           Goal state
   *  \param[in] ttcPerAction   Base time per action (can be scaled for individual actions inside planner)
   *  \param[in] executeAll     Boolean to define if the whole resulting planned sequence should just be executed (without handling
   *                            by ManipulationComponent)
   */
  virtual void planSequence(Rcs::SearchNode_ptr curState, Rcs::SearchNode_ptr stop, double ttcPerAction, bool executeAll = false);

  /*! \brief Issues a request to execute a planned sequence on topic "ExecutePlan".
   *         It also publishes the transition type on topic "MotionType" and requests the necessary
   *         sensor cues to monitor this transition on topic "RequestMonitor".
   *
   *  \param[in] plan           Sequence of states (each state is still a vector<int> here)
   *  \param[in] localTtc       Time for transition
   */
  void executeAndMonitorSequence(std::vector<std::vector<int> > plan, double localTtc);

  /*! \brief Handles the transition to the next action in the planned sequence. If the sequence is
   *         completed, returns to IDLE state. Checks in the object model if a confirmation signal
   *         is needed before executing the next action. If so, all necessary monitors are requested
   *         and execution is suspended until the correct sensor cue is perceived (signal from
   *         MonitorComponent).
   */
  void startNextAction();

  /*! \brief Resets the ManipulationComponent to IDLE state, clears all existing Monitor Requests,
   *         and requests new monitors based on possible intentions for the current object state and
   *         object model.
   */
  void toIdle();

  /*! \brief Callback function for incoming messages on topic "Confirmation". Some actions require a confirmation
   *         before being executed. This enables to synchronize execution with a human partner as execution
   *         can be paused until the human is ready. If a confirmation is received, the queued action is being
   *         executed. In addition, the object model is queried if any visualizations should be published.
   *
   *  \param[in] id    ID of confirmation (currently unused as only on action should be queued up)
   *  \param[in] seq   sequence number used to prevent reception of old data or race conditions
   */
  void onConfirmation(int id, int seq);

  /*! \brief Callback function for incoming messages on topic "Progress". It passes the id to the
   *         ActionProgressMonitor to evaluate how the action progresses and if something went wrong.
   *         The ActionProgressMonitor is defined in the object model.
   *         This functionality has been tested in an earlier version, but not been used here.
   *         Thus, use with care.
   *
   *  \param[in] id    ID of progress event (defined and used by object model/action progress monitor)
   *  \param[in] seq   sequence number used to prevent reception of old data or race conditions
   */
  void onProgress(int id, int seq);

  /*! \brief Callback function for incoming messages on topic "State". Updates current state and
   *         desired state. (State definition provided by object model).
   *
   *  \param[in] desiredState    desired state provided as vector<int>
   *  \param[in] currentState    current state provided as vector<int>
   */
  void onState(std::vector<int> desiredState, std::vector<int> currentState);

  /*! \brief Sets a new time to completion (TTC) for actions. This time is used as basis for planned action sequences.
   *         Individual actions can deviate from this, but are usually scaled by it.
   *         This function is called for requests on topic "SetTTC".
   *
   *  \param[in] ttcNew    New TTC in seconds
   */
  void onSetTTC(double ttcNew);

  /*! \brief Callback function for incoming messages on topic "Intention". For a received intention the
   *         object model is consulted for a corresponding goal state. Then a plan to this state is requested.
   *
   *  \param[in] id    ID of intention (IDs are provided and understood by the object model)
   *  \param[in] seq   sequence number used to prevent reception of old data or race conditions
   */
  void onIntention(int id, int seq);

  /*! \brief Callback function for incoming messages on topic "MotionState". This topic should be published
   *         by the motion controller. Used to recognize when executed motions should be completed. On
   *         transition from Moving to Stopped, the next action will be queue or, if no more actions are planned,
   *         reset the internal state to IDLE.
   *
   *  \param[in] state    Either Moving or Stopped
   */
  void onMotionState(bool isMoving);

  /*! \brief Callback function for incoming messages on topic "PlanResult". Handle the plan result and process the
   *         first action.
   *
   *  \param[in] plan    A sequence of states. Each state is still a vector<int>
   */
  void onMotionPlanResult(std::vector<std::vector<int> > plan);

  /*! \brief Callback function for incoming messages on topic "PausePlan". Pauses execution of the current action.
   *         The current trajectory is cleared and the current action is set to be the next action again. The
   *         internal state is set to PAUSED which will prevent all execution until the pause is revoked.
   *         Note that the original TTC for the next action will remain the same even if 90% of the movement have
   *         already been completed.
   */
  void onPause();

  /*! \brief Callback function for incoming messages on topic "ResumePlan". Reverts the paused state of the system.
   *         The next action will be executed. Note that the original TTC for the next action will remain the same
   *         even if 90% of the movement have already been completed.
   */
  void onResume();

  /*! \brief Callback function for incoming messages on topic "StopActions". The current trajectory is cleared and
   *         the internal state is reset to IDLE. Planned action sequences are lost.
   */
  void onStop();

  /*! \brief Callback function for incoming messages on topic "RecoverToClosestState". Commands the motion controller
  *          to move to the current state. This should result in all offsets to go to zero.
  */
  void toClosestState();

  /*! \brief Callback function for incoming messages on topic "EmergencyStop". Same as onStop() but also sets estop
   *         flag which prevent exection until revoked.
   */
  void onEmergencyStop();

  /*! \brief Callback function for incoming messages on topic "EmergencyRecover". Revoke estop flag and reset internal
   *         state to IDLE.
   */
  void onEmergencyRecover();

  bool interactionEnabled;

  Rcs::SearchNode_ptr currentState;
  Rcs::SearchNode_ptr desiredState;


  InteractionState interactionState;
  InteractionState lastInteractionState;
  double ttc;
  double nextTtc;
  std::vector<std::vector<int> > currentAction;
  std::vector<std::vector<int> > plannedSequence;
  size_t locationInSequence;
  double timeToWait;
  int validSeqStart;
  int currentSeqSent;

  int currentHelpLevel;
  double interactionStateStartTime;

  double objectBottomPos[3];

  //  HTr handOffset_ho[2];
  //  int handState[2];

  ActionProgressMonitor progressMonitor;

  // enable to bypass all needed haptic confirmations --> play through whole trajectory
  // without pauses
  bool autoConfirm;

  bool eStop;
  bool inMotion;

  std::shared_ptr<ObjectModel> objModel;

  RcsGraph* graph;
};

} // namespace Rcs

#endif //RCS_MANIPULATIONCOMPONENT_H
