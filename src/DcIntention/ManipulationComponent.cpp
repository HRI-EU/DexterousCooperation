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

#include "ManipulationComponent.h"
#include "MonitorComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>

namespace Dc
{

ManipulationComponent::ManipulationComponent(EntityBase* entity, const RcsGraph* graph_,
                                             std::shared_ptr<ObjectModel> objModel_) :
  ComponentBase(entity), interactionEnabled(false), ttc(6.0), locationInSequence(0), timeToWait(0.0),
  validSeqStart(0), currentSeqSent(0), eStop(false), inMotion(false),
  objModel(objModel_)
{
  currentState = objModel->getEmptyState();
  desiredState = objModel->getEmptyState();

  this->graph = RcsGraph_clone(graph_);

  interactionState = InteractionState::IDLE;
  lastInteractionState = interactionState;
  interactionStateStartTime = getEntity()->getTime();
  currentHelpLevel = 0;

  Vec3d_setZero(objectBottomPos);

  subscribe<RcsGraph*>("UpdateGraph", &ManipulationComponent::updateGraph);

  // sent by anyone
  subscribe<>("StopActions", &ManipulationComponent::onStop);
  subscribe<>("PausePlan", &ManipulationComponent::onPause);
  subscribe<>("ResumePlan", &ManipulationComponent::onResume);
  subscribe<>("RecoverToClosestState", &ManipulationComponent::toClosestState);

  // sent by anyone
  subscribe("EmergencyStop", &ManipulationComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &ManipulationComponent::onEmergencyRecover);

  subscribe("PlanResult", &ManipulationComponent::onMotionPlanResult);
  subscribe("MotionState", &ManipulationComponent::onMotionState);
  subscribe("Intention", &ManipulationComponent::onIntention);
  subscribe("Confirmation", &ManipulationComponent::onConfirmation);
  subscribe("Progress", &ManipulationComponent::onProgress);
  subscribe("SetTTC", &ManipulationComponent::onSetTTC);

  subscribe("State", &ManipulationComponent::onState);
  subscribe("Status", &ManipulationComponent::status);

  // load action progress graphs from object model
  progressMonitor.addActions(objModel->getActionProgressGraphs());



  //TODO: object specific

  // write out debug file in .dot format for each loaded action progress graph
  //  std::vector<int> tmp2 = {0, 1, 3, 2, 7};
  SearchNode_ptr tmp3 = objModel->getEmptyState();
  //TODO: state

  for (size_t i = 0; i < progressMonitor.actions_.size(); i++)
  {
    RLOG(0, "Writing progress state graph for action '%s' to file '%s'.",
         progressMonitor.actions_[i]->description_.c_str(),
         ("action_" + progressMonitor.actions_[i]->id_ + ".dot").c_str());
    progressMonitor.monitor(progressMonitor.actions_[i], SearchNode_ptr(), ttc, getEntity()->getTime());

    progressMonitor.actions_[i]->writeDot("action_" + progressMonitor.actions_[i]->id_ + ".dot");
    progressMonitor.stop();
  }
}


ManipulationComponent::~ManipulationComponent()
{
  RcsGraph_destroy(this->graph);
}


void ManipulationComponent::status() const
{
  RLOG(0, "InteractionState: %d: '%s'", (int)interactionState, InteractionStateStr(interactionState).c_str());

  if (plannedSequence.empty())
  {
    printf("No planned sequence.\n");
    return;
  }

  printf("\n");
  printf("Planned: \n");

  for (size_t i = 0; i < plannedSequence.size(); i++)
  {
    std::string transitionName = "";
    unsigned int typeId = 0;
    if (i >= 1)
    {
      //TODO: object specific
      typeId = objModel->getTransitionType(plannedSequence[i - 1], plannedSequence[i]);
      transitionName = objModel->transitionTypeToString(typeId);
    }

    printf("%s %2d : %s  %s (%d) \n", (locationInSequence == i ? "g" : (locationInSequence - 1 == i ? "c" : " ")),
           (int)i, Gras::vecToStr(plannedSequence[i]).c_str(), transitionName.c_str(), (int)typeId);
  }
  if (locationInSequence >= plannedSequence.size())
  {
    printf("   Done.\n");
  }
  printf("\n");
}


/// TODO: Cancel trajectories, etc.
void ManipulationComponent::onEmergencyStop()
{
  eStop = true;

  onStop(); // we may need to do more handling here
}

void ManipulationComponent::onEmergencyRecover()
{
  eStop = false;
  toIdle();//onStop(); // we may need to do more handling here
}


void ManipulationComponent::onPause()
{
  if (eStop)
  {
    return;
  }

  if (locationInSequence > 0)
  {
    locationInSequence--;
    RLOG(0, "Location in sequence decremented to %zd", locationInSequence);
  }

  getEntity()->publish("ClearTrajectory");
  //getEntity()->publish("ClearMonitors", std::string(""));

  interactionState = InteractionState::PAUSED;
}

void ManipulationComponent::onResume()
{
  if (interactionState != InteractionState::PAUSED)
  {
    return;
  }

  if (plannedSequence.empty())
  {
    return;
  }

  RLOG(0, "Resuming previous plan...");
  startNextAction();
}

void ManipulationComponent::onStop()
{
  getEntity()->publish("ClearTrajectory");
  getEntity()->publish("ClearMonitors", std::string("all"));

  // update sequence number range for valid responses
  validSeqStart = currentSeqSent;

  if (this->inMotion)
  {
    // Still moving... stopping
    interactionState = InteractionState::STOPPING;
  }
  else
  {
    if (this->eStop)
    {
      // Currently not moving... estopped
      plannedSequence.clear();
      interactionState = InteractionState::ESTOPPED;
    }
    else
    {
      toIdle();
    }
  }
}

void ManipulationComponent::toClosestState()
{
  //TODO: this could result in illegal states and thus self collisions
  //TODO: even without collision, planning from an illegal state is prohibited and the robot would be stuck
  plannedSequence.clear();
  getEntity()->publish("MoveTo", currentState, 2.0);
  interactionState = InteractionState::STOPPING;
}


void ManipulationComponent::onMotionState(bool isMoving)
{
  //TODO: motionstate needs to be defined in a better place!
  RLOG(0, "Motion State: %s, Current state: %s",
       isMoving ? "MOVING" : "STOPPED",
       Gras::vecToStr(currentState).c_str());

  this->inMotion = isMoving;

  if (isMoving == false)
  {
    if (interactionState == InteractionState::IDLE)
    {
      //  nothing to do here
    }
    else if (interactionState == InteractionState::STOPPING)
    {
      if (eStop)
      {
        interactionState = InteractionState::ESTOPPED;
      }
      else
      {
        toIdle();
      }
    }
    else if (interactionState == InteractionState::IN_PROGRESS)
    {
      RLOG(0, "Movement done. Checking for next action.");
      startNextAction();
    }
    else
    {
      RLOG(1,
           "Unexpected event: 'MotionState::Stopped' while in interaction state '%s'",
           InteractionStateStr(interactionState).c_str());

      //      interactionState = InteractionState::IDLE;
    }
  }
  else //if (isMoving == true)
  {
    if (interactionState != InteractionState::STOPPING && interactionState != InteractionState::ESTOPPED)
    {
      //TODO: Maybe this is not needed if state is already changed in WAIT_FOR_CONFIRMATION
      interactionState = InteractionState::IN_PROGRESS;
    }
  }

  //
  //  if (interactionState == InteractionState::IN_PROGRESS &&
  // state == Rcs::DyadicMotionPlannerComponent::MotionState::Stopped)
  //  {
  //    interactionState = InteractionState::IDLE;
  //
  //  }
  //  else if (state == Rcs::DyadicMotionPlannerComponent::MotionState::Moving)
  //  {
  //    interactionState = InteractionState::IN_PROGRESS;
  //  }

  /// TODO: There's a timing issue here which can cause us to transition: WAITING_FOR_PLANNER->IDLE->IN_PROGRESS
  /// Enforcing state transitions better will solve this
}
void ManipulationComponent::onMotionPlanResult(std::vector<std::vector<int> > plan)
{
  interactionState = InteractionState::START_NEXT_ACTION;

  if ((int)plan.size() < 2)
  {
    RLOG(0, "No plan found. Idling.");
    //    interactionState = InteractionState::IDLE;
    toIdle();
  }
  else
  {
    RLOG(0, "onMotionPlanResult with %d steps", (int)plan.size());

    plannedSequence = plan;
    locationInSequence = 1;

    startNextAction();
  }

  // TODO: Separate individual actions, execute in sequence, WaitForConfirmation
}

void ManipulationComponent::updateGraph(RcsGraph* graph_)
{
  RcsGraph_setState(this->graph, graph_->q, NULL);

  char text[256];
  snprintf(text, 256, "%sManipulation: %d - %s", eStop ? "E-STOP!\n" : "",
           (int)interactionState, InteractionStateStr(interactionState).c_str());
  getEntity()->publish<std::string>("SetTextLine", std::string(text), 0);

  std::vector<int> stableState = objModel->getClosestCanonicalState(currentState);

  snprintf(text, 256, "Closest stable state:      %s", Gras::vecToStr(stableState).c_str());
  getEntity()->publish<std::string>("SetTextLine", std::string(text), 1);

  if (lastInteractionState != interactionState)
  {
    interactionStateStartTime = getEntity()->getTime();
    currentHelpLevel = 0;
    lastInteractionState = interactionState;
  }

  double timeInState = getEntity()->getTime() - interactionStateStartTime;
  if (interactionState == InteractionState::WAITING_FOR_CONFIRMATION)
  {

    if (timeInState > 3.0 && currentHelpLevel == 0)
    {

      //TODO: object specific

      // send alignment display event to visualizer
      RLOG(0, "Changing vis display to 'ObjectAlignment'");
      getEntity()->publish("Display", std::string("ObjectAlignment"));

      currentHelpLevel++;
    }
  }
}


/*******************************************************************************
 *
 ******************************************************************************/
void ManipulationComponent::planSequence(SearchNode_ptr curState,
                                         SearchNode_ptr goalState,
                                         double ttcPerAction,
                                         bool executeAll)
{
  RLOG(0, "planSequence");

  if (executeAll)
  {
    getEntity()->publish("PlanAndExecute", curState, goalState, ttcPerAction);
  }
  else
  {
    getEntity()->publish("Plan", curState, goalState, ttcPerAction);
  }
}

/*******************************************************************************
 *
 ******************************************************************************/
void ManipulationComponent::setAutoConfirm(bool enable)
{
  this->autoConfirm = enable;
}

/*******************************************************************************
 *
 ******************************************************************************/
bool ManipulationComponent::getAutoConfirm()
{
  return this->autoConfirm;
}

/*******************************************************************************
 *
 ******************************************************************************/
void ManipulationComponent::setObjectBottomPos(double* pos)
{
  Vec3d_copy(objectBottomPos, pos);
}

/*******************************************************************************
 *
 ******************************************************************************/
void ManipulationComponent::getObjectBottomPos(double* pos) const
{
  Vec3d_copy(pos, objectBottomPos);
}


void ManipulationComponent::executeAndMonitorSequence(std::vector<std::vector<int> > plan, double localTtc)
{
  RLOG(0, "Execute and Monitor Sequence");

  // determine action type
  unsigned int type = objModel->getTransitionType(plan[0], plan[1]);
  std::string typeStr(std::to_string(type));

  getEntity()->publish("MotionType", (int)type, typeStr);

  SearchNode_ptr goal = plan[1];
  progressMonitor.monitor(typeStr, goal, localTtc, getEntity()->getTime());

  //trigger all events for reaching the start state
  progressMonitor.triggerStateChangeActions(getEntity());

  std::vector<MonitorRequest> reqs = progressMonitor.getRequests();

  getEntity()->publish("ClearMonitors", std::string("all"));
  // update sequence number range for valid responses
  validSeqStart = currentSeqSent;


  for (size_t i = 0; i < reqs.size(); i++)
  {
    reqs[i].sequenceNum = currentSeqSent++;
    getEntity()->publish("RequestMonitor", reqs[i]);
  }

  RLOG(0, "Local TTC for execute: %4.2f", localTtc);
  getEntity()->publish<std::vector<std::vector<int> > >("ExecutePlan", plan, localTtc);
  interactionState = InteractionState::IN_PROGRESS;

  //TODO: determine what needs to be done to start progress monitor
}

void ManipulationComponent::onIntention(int intention_id, int seq)
{
  if (seq < validSeqStart)
  {
    RLOG(0, "Received old intention with seq #%d: Ignoring.", seq);
  }

  RLOG(0, "[%s] Received intention %d with seq %d", InteractionStateStr(interactionState).c_str(), intention_id, seq);


  if (interactionState == InteractionState::IDLE || interactionState == InteractionState::IN_PROGRESS)
  {
    std::vector<int> nextGoalState = objModel->getGoalFromIntention(currentState, intention_id);

    RLOG(0, "Next goal state: %s", Gras::vecToStr(nextGoalState).c_str());

    if (nextGoalState.empty())
    {
      RLOG(0, "Unknown intention: %d. Ignoring.", intention_id);
      return;
    }

    SearchNode_ptr endState = nextGoalState;

    planSequence(currentState, endState, ttc, false);
    interactionState = InteractionState::WAITING_FOR_PLANNER;

  }
  //TODO: extend this to include behavior in other states to make more interactive

  // update sequence number range for valid responses
  //  validSeqStart = currentSeqSent;




}

void ManipulationComponent::onConfirmation(int id, int seq)
{
  if (seq < validSeqStart)
  {
    RLOG(0, "Received old confirmation with seq #%d: Ignoring.", seq);
  }

  RLOG(0, "[%s] Received confirmation with seq %d", InteractionStateStr(interactionState).c_str(), seq);

  if (interactionState == InteractionState::WAITING_FOR_CONFIRMATION)
  {
    //TODO: object specific
    objModel->triggerExecutionVisualization(currentAction[0], currentAction[1], graph);

    //send action to be executed
    executeAndMonitorSequence(currentAction, this->nextTtc);
  }

}

void ManipulationComponent::onProgress(int id, int seq)
{
  if (seq < validSeqStart)
  {
    RLOG(0, "Received old progress with seq #%d: Ignoring.", seq);
    return;
  }

  //TODO: progress needs an own sequence number?
  RLOG(0, "[%s] Received progress %d with seq %d", InteractionStateStr(interactionState).c_str(), id, seq);

  if (!progressMonitor.onMonitorTrigger(id))
  {
    RLOG(0, "Received unknown progress request with id: %d", id);
    return;
  }


  //TODO: check if any action is required: e.g. display pictogram, etc.

  // take any required actions (e.g. display, communicate, compensation movements)
  progressMonitor.triggerStateChangeActions(getEntity());

  if (progressMonitor.currentState_->goal_)
  {
    if (progressMonitor.currentState_->success_)
    {
      RLOG(0, "Action finished successfully.");
      //TODO: go to next action
    }
    else
    {
      RLOG(0, "Action finished unsuccessfully.");

      //TODO: stop action progress
    }

  }

  std::vector<MonitorRequest> reqs = progressMonitor.getRequests();

  //TODO: check this
  getEntity()->publish("ClearMonitors", std::string("Progress"));
  // update sequence number range for valid responses
  validSeqStart = currentSeqSent;

  for (size_t i = 0; i < reqs.size(); i++)
  {
    reqs[i].sequenceNum = currentSeqSent++;
    getEntity()->publish("RequestMonitor", reqs[i]);
  }

}

void ManipulationComponent::onState(std::vector<int> desiredStateIn, std::vector<int> currentStateIn)
{
  desiredState = desiredStateIn;

  //TODO: this needs to be reverted and everywhere in the code the correct state needs to be used
  currentState = desiredStateIn;

  //TODO: what to do with this...?
}

void ManipulationComponent::onSetTTC(double ttcNew)
{
  if (ttcNew <= 0.0)
  {
    RLOG(0, "Invalid TTC requested: %5.2f", ttcNew);
  }
  else
  {
    RLOG(0, "Setting TTC to %5.2f", ttcNew);
    this->ttc = ttcNew;
  }
}

void ManipulationComponent::startNextAction()
{
  RLOG(0, "Start next action. Location %d, sequence size: %d", (int)locationInSequence, (int)plannedSequence.size());

  status();


  if (locationInSequence < plannedSequence.size()) //TODO: check if condition is precise
  {
    //TODO: create next action (path with two states)
    currentAction.clear();
    currentAction.reserve(2);

    currentAction.push_back(currentState);
    currentAction.push_back(plannedSequence[locationInSequence]);

    if (locationInSequence > 0)
    {
      bool same = true;
      for (int i = 0; i < (int)currentState.size(); i++)
      {
        if (plannedSequence[locationInSequence - 1][i] != currentState[i])
        {
          same = false;
          break;
        }
      }
      if (!same)
      {
        RLOG(0, "Current state does not match the endstate of the previous action: ");
        RLOG(0, "  Expected:   %s", Gras::vecToStr(plannedSequence[locationInSequence - 1]).c_str());
        RLOG(0, "  Actual:     %s", Gras::vecToStr(currentState).c_str());
      }
    }

    locationInSequence++;

    //TODO: object specific
    unsigned int type = objModel->getTransitionType(currentAction[0], currentAction[1]);

    RLOG(1, "Current action:");
    RLOG(1, "     %s", Gras::vecToStr(currentAction[0]).c_str());
    RLOG(1, "     %s", Gras::vecToStr(currentAction[1]).c_str());
    RLOG(1, "Transition type: %d ('%s')", (int)type, objModel->transitionTypeToString(type).c_str());

    this->nextTtc = objModel->getTtc(currentAction[0], currentAction[1], this->ttc);
    RLOG(0, "Action: %s TTC: %4.2f", objModel->transitionTypeToString(type).c_str(), nextTtc);

    bool confirmationNeeded = objModel->confirmationNeeded(currentAction[0], currentAction[1]);


    //TODO: determine if there are conditions to be fulfilled before execution
    if (confirmationNeeded) // TODO: fix this: conditionsRequired())
    {
      //TODO: submit monitor request for confirmation sensorSignals
      std::vector<MonitorRequest> req = objModel->configureConfirmationMonitorsForTransition(currentAction[0],
                                        currentAction[1]);

      getEntity()->publish("ClearMonitors", std::string("all"));
      // update sequence number range for valid responses
      validSeqStart = currentSeqSent;


      for (size_t i = 0; i < req.size(); i++)
      {
        req[i].sequenceNum = currentSeqSent++;
        getEntity()->publish("RequestMonitor", req[i]);
      }

      //      // send alignment display event to visualizer
      //      RLOG(0, "Changing vis display to 'ObjectAlignment'");
      //      getEntity()->publish("Display", std::string("ObjectAlignment"));


      interactionState = InteractionState::WAITING_FOR_CONFIRMATION;
    }
    else
    {


      //TODO: object specific
      //send default visualizer state
      RLOG(0, "Changing vis display to 'CurrentAndGoal'");
      //      getEntity()->publish("DesiredStateChanged", -currentAction[1].at(0) * motionPlanner->getDeltaPhi());
      objModel->triggerExecutionVisualization(currentAction[0], currentAction[1], graph);
      //getEntity()->publish("Display", std::string("CurrentAndGoal"));

      //send action to be executed and monitored
      executeAndMonitorSequence(currentAction, this->nextTtc);

      //TODO: switch on MotionState::Moving or just directly?
      //      interactionState = InteractionState::IN_PROGRESS;
    }
  }
  else
  {
    // no actions left, done
    toIdle();
  }
}

void ManipulationComponent::toIdle()
{
  currentAction.clear();
  plannedSequence.clear();
  locationInSequence = 0;

  //TODO: currently hard-coded to BOX
  std::vector<MonitorRequest> req = objModel->configureMonitorsForState(currentState);

  getEntity()->publish("ClearMonitors", std::string("all"));
  // update sequence number range for valid responses
  validSeqStart = currentSeqSent;

  for (size_t i = 0; i < req.size(); i++)
  {
    req[i].sequenceNum = currentSeqSent++;
    getEntity()->publish("RequestMonitor", req[i]);
  }


  this->interactionState = InteractionState::IDLE;
}

} // namespace