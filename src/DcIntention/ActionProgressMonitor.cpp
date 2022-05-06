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

#include "ActionProgressMonitor.h"
#include "MonitorRequest.h"
#include <Rcs_timer.h>
#include "Rcs_macros.h"


namespace Rcs
{

//double gazeAttentionThreshold =  RCS_DEG2RAD(60.0);
//double gazeAttentionDuration = 0.5;
//double duration = 6.0;

ActionProgressMonitor::ActionProgressMonitor()
{

  //TODO: mismatch box pose -> sensorize, additional state for this case (5)?
  //TODO: fault cases: stop, regrasp, undo, prompt for help, adjust plan during motion?
  //TODO: stop on external impact
  //TODO: return to previous state when box follows
  //TODO: Adjustment movement when contact missing at end of motion
  //TODO: adjust Tdeltamax at runtime --> done

  //  actions_.push_back(createRegraspAPG(true));
  //  actions_.push_back(createRegraspAPG(false));
  //  actions_.push_back(createRotateObjectAPG());

  currentAction_.reset();// = actions_[0];
  actionStartTime_ = Timer_getSystemTime();

}

ActionProgressMonitor::~ActionProgressMonitor()
{

}

//Apg_ptr ActionProgressMonitor::createRotateObjectAPG()
//{
//  Apg_ptr tmp2(new ActionProgressGraph(std::to_string(0), ("rotate object")));
//
//  tmp2->duration_ = duration;
//
//  ActionProgressState_ptr s8(new ActionProgressState());
//  s8->id_ = "0";
//  s8->description_ = "start";
//  s8->action_ = tmp2;
//
//  s8->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "CurrentAndGoal"));
//  s8->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));
//
//  ActionProgressState_ptr s9(new ActionProgressState());
//  s9->id_ = "1";
//  s9->description_ = "fault, partner not paying attention";
//  s9->action_ = tmp2;
//  s9->goal_ = false;
//  s9->success_ = false;
//
//  s9->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));
//
//  Transition_ptr t_attention_error(new ActionProgressTransition(1, s8, s9, "partner attention lost"));
//  t_attention_error->deltaTmin_ = 0.0;
//  t_attention_error->deltaTmax_ = 1.0;
//  s8->transitions_out_.push_back(t_attention_error);
//  s9->transitions_in_.push_back(t_attention_error);
//
//  MonitorRequest_ptr
//      m_checkGaze(new MonitorRequest("Progress", 1, 0, "partner not paying attention", gazeAttentionDuration));
//  m_checkGaze->addSignal("gazeError", gazeAttentionThreshold, false);
//
//  t_attention_error->monitors_.push_back(m_checkGaze);
//  s8->monitors_.push_back(m_checkGaze);
//
//  Transition_ptr t_attention_recover(new ActionProgressTransition(1, s9, s8, "partner attention regained"));
//  t_attention_recover->deltaTmin_ = 0.0;
//  t_attention_recover->deltaTmax_ = 10.0;
//  s9->transitions_out_.push_back(t_attention_recover);
//  s8->transitions_in_.push_back(t_attention_recover);
//
//  MonitorRequest_ptr
//      m_checkGazeGood(new MonitorRequest("Progress", 1, 0, "partner paying attention", gazeAttentionDuration));
//  m_checkGazeGood->addSignal("gazeError", gazeAttentionThreshold, true);
//
//  t_attention_recover->monitors_.push_back(m_checkGazeGood);
//  s9->monitors_.push_back(m_checkGazeGood);
//
//  tmp2->states_.push_back(s8);
//  tmp2->states_.push_back(s9);
//
//  tmp2->transitions_.push_back(t_attention_error);
//  tmp2->transitions_.push_back(t_attention_recover);
//
//  return tmp2;
//}
//
//
//Apg_ptr ActionProgressMonitor::createRegraspAPG(bool rightHandRegrasp)
//{
////  double externalCollisionForce = 16.0;
//  double handForceLoaded = 8.0;
//  double handForceUnloaded = 2.0;
////  double positionError = 0.08;
//  double maxRotationError = 0.26; // ~15 deg
//
//  std::string side = rightHandRegrasp ? "right" : "left";
//
//  Apg_ptr tmp(new ActionProgressGraph(std::to_string(1), ("regrasp " + side)));
//  if (rightHandRegrasp)
//  {
//    tmp->id_ = "1";
//  }
//  else
//  {
//    tmp->id_ = "2";
//  }
//
//
//  tmp->duration_ = duration;
//
//  ActionProgressState_ptr s0(new ActionProgressState());
//  s0->id_ = "0";
//  s0->description_ = "start, both hands in contact";
//  s0->action_ = tmp;
//
//  //    s0->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "HumanGraspError"));
//  s0->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "ObjectAlignment"));
//  s0->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));
//
//  ActionProgressState_ptr s1(new ActionProgressState());
//  s1->id_ = "1";
//  s1->description_ = "one hand in transit";
//  s1->action_ = tmp;
//
////  s1->enterEvents.push_back(std::make_pair<std::string, std::string>("You are ", "on your way"));
//  s1->enterEvents.push_back(std::make_pair<std::string, std::string>("ResumePlan", ""));
//
//  ActionProgressState_ptr s2(new ActionProgressState());
//  s2->id_ = "2";
//  s2->description_ = "goal reached, both hands in contact again";
//  s2->action_ = tmp;
//  s2->goal_ = true;
//  s2->success_ = true;
//
//  ActionProgressState_ptr s3(new ActionProgressState());
//  s3->id_ = "3";
//  s3->description_ = "fault, movement finished, but no contact re-established";
//  s3->action_ = tmp;
//  s3->goal_ = true;
//  s3->success_ = false;
//
//  ActionProgressState_ptr s4(new ActionProgressState());
//  s4->id_ = "4";
//  s4->description_ = "fault, external contact while moving hand";
//  s4->action_ = tmp;
//  s4->goal_ = true;
//  s4->success_ = false;
//
//  ActionProgressState_ptr s5(new ActionProgressState());
//  s5->id_ = "5";
//  s5->description_ = "box pose is shifting too far. Pausing until correction.";
//  s5->action_ = tmp;
//  s5->goal_ = false;
//  s5->success_ = false;
//
//  s5->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));
//
//  ActionProgressState_ptr s6(new ActionProgressState());
//  s6->id_ = "6";
//  s6->description_ = "fault, box following on withdrawl";
//  s6->action_ = tmp;
//  s6->goal_ = true;
//  s6->success_ = false;
//
//  s6->enterEvents.push_back(std::make_pair<std::string, std::string>("Display", "ObjectAlignment"));
//
//  ActionProgressState_ptr s7(new ActionProgressState());
//  s7->id_ = "7";
//  s7->description_ = "fault, partner not paying attention";
//  s7->action_ = tmp;
//  s7->goal_ = false;
//  s7->success_ = false;
//
//  //    s7->enterEvents.push_back(std::make_pair<std::string, std::string>("StopActions", ""));
//  s7->enterEvents.push_back(std::make_pair<std::string, std::string>("PausePlan", ""));
//  //  ManipulationState_ptr s7(new ActionProgressState());
//  //  s7->id_ = "7";
//  //  s7->description_ = "box pose is shifting too far. Adapting trajectory.";
//  //  s7->action_ = tmp;
//  //  s7->goal_ = false;
//  //  s7->success_ = false;
//
//  //  sensorData["normalForceRight"] = 0.0;
//  //  sensorData["normalForceLeft"] = 0.0;
//  //  sensorData["forceNonGravityRight"] = 0.0;
//  //  sensorData["forceNonGravityLeft"] = 0.0;
//  //  sensorData["pushForce"] = 0.0;
//  //  sensorData["normalForceDifference"] = 0.0;
//  //  "boxDistance"
//  //  "handDistanceRight"
//  //  "handDistanceLeft"
//
//
//
//  //---------------------------------------------------------------------
//
//  Transition_ptr t_hand_release(new ActionProgressTransition(0, s0, s1, side + " hand released"));
//  t_hand_release->deltaTmin_ = 0.0;
//  t_hand_release->deltaTmax_ = 1.0;
//  s0->transitions_out_.push_back(t_hand_release);
//  s1->transitions_in_.push_back(t_hand_release);
//
//  MonitorRequest_ptr m_hand_release(new MonitorRequest("Progress", 0, 0, side + " hand released", 0.0));
//  if (rightHandRegrasp)
//  {
//    m_hand_release->addSignal("normalForceRight", handForceUnloaded, true);
//    m_hand_release->addSignal("forceNonGravityRight", handForceUnloaded, true);
//    m_hand_release->addSignal("normalForceLeft", handForceLoaded, false);
//    m_hand_release->addSignal("forceGravityLeft", handForceLoaded, false);
//  }
//  else
//  {
//    m_hand_release->addSignal("normalForceLeft", handForceUnloaded, true);
//    m_hand_release->addSignal("forceNonGravityLeft", handForceUnloaded, true);
//    m_hand_release->addSignal("normalForceRight", handForceLoaded, false);
//    m_hand_release->addSignal("forceGravityRight", handForceLoaded, false);
//  }
//  //  m_hand_release_r->addSignal("forceNonGravityRight", handForceUnloaded, true);
//  //  m_hand_release_r->addSignal("forceNonGravityLeft", handForceUnloaded, true);
//  t_hand_release->monitors_.push_back(m_hand_release);
//
//  s0->monitors_.push_back(m_hand_release);
//
//  //---------------------------------------------------------------------
//
//  Transition_ptr t_hand_recontact(new ActionProgressTransition(1, s1, s2, "reach goal"));
//  t_hand_recontact->deltaTmin_ = 0.8;
//  t_hand_recontact->deltaTmax_ = 1.0;
//  s1->transitions_out_.push_back(t_hand_recontact);
//  s2->transitions_in_.push_back(t_hand_recontact);
//
//  //TODO: these conditions need fixing
//  MonitorRequest_ptr m_hand_recontact(new MonitorRequest("Progress", 1, 0, side + " hand re-establish contact", 0.0));
//
//  if (rightHandRegrasp)
//  {
//    m_hand_recontact->addSignal("normalForceRight", handForceLoaded, false);
//    m_hand_recontact->addSignal("normalForceLeft", handForceLoaded, false);
//    m_hand_recontact->addSignal("forceGravityLeft", handForceLoaded, false);
//  }
//  else
//  {
//    m_hand_recontact->addSignal("normalForceLeft", handForceLoaded, false);
//    m_hand_recontact->addSignal("normalForceRight", handForceLoaded, false);
//    m_hand_recontact->addSignal("forceGravityRight", handForceLoaded, false);
//  }
//
//  t_hand_recontact->monitors_.push_back(m_hand_recontact);
//
//  s1->monitors_.push_back(m_hand_recontact);
//  //---------------------------------------------------------------------
//
////  Transition_ptr t_external_contact(new ActionProgressTransition(2, s0, s4, "collision with obstacle"));
////  t_external_contact->deltaTmin_ = 0.2;
////  t_external_contact->deltaTmax_ = 1.1;
////  s0->transitions_out_.push_back(t_external_contact);
////  s4->transitions_in_.push_back(t_external_contact);
////
////  MonitorRequest_ptr m_ext_collision(new MonitorRequest("Progress", 2, 0, "collision with obstacle", 0.0));
////  if (rightHandRegrasp)
////  {
////    m_ext_collision->addSignal("forceNonGravityRight", externalCollisionForce, false);
////    m_ext_collision->addSignal("normalForceLeft", handForceLoaded, false);
////    m_ext_collision->addSignal("forceGravityLeft", handForceLoaded, false);
////  }
////  else
////  {
////    m_ext_collision->addSignal("forceNonGravityLeft", externalCollisionForce, false);
////    m_ext_collision->addSignal("normalForceRight", handForceLoaded, false);
////    m_ext_collision->addSignal("forceGravityRight", handForceLoaded, false);
////  }
////  t_external_contact->monitors_.push_back(m_ext_collision);
////
////  s0->monitors_.push_back(m_ext_collision);
//
//  //---------------------------------------------------------------------
//
////  Transition_ptr t_collision(new ActionProgressTransition(3, s1, s4, "collision with obstacle"));
////  t_collision->deltaTmin_ = 0.0;
////  t_collision->deltaTmax_ = 1.0;
////  s1->transitions_out_.push_back(t_collision);
////  s4->transitions_in_.push_back(t_collision);
////
////  MonitorRequest_ptr m_collision(new MonitorRequest(m_ext_collision));
////  m_collision->description = "collision with obstacle";
////  m_collision->id = 3;
////
////  t_collision->monitors_.push_back(m_collision); // use same monitor as above
////
////  s1->monitors_.push_back(m_collision);
//
//  //---------------------------------------------------------------------
//
////  Transition_ptr t_finshed_no_contact(new ActionProgressTransition(4, s1, s3, "finished regrasp, but no contact"));
////  t_finshed_no_contact->deltaTmin_ = 1.0;
////  t_finshed_no_contact->deltaTmax_ = 1.1;
////  s1->transitions_out_.push_back(t_finshed_no_contact);
////  s3->transitions_in_.push_back(t_finshed_no_contact);
////
////  //  MonitorRequest_ptr m_no_contact_r(new MonitorRequest("Progress", 4, 0, "collision with obstacle", 0.0));
////  //  m_ext_collision_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
////  //  m_ext_collision_r->addSignal("normalForceLeft", handForceLoaded, false);
////  //  m_ext_collision_r->addSignal("forceGravityLeft", handForceLoaded, false);
////  MonitorRequest_ptr m_regrasp_without_contact(new MonitorRequest(m_hand_release));
////  m_regrasp_without_contact->description = "finished regrasp, but no contact";
////
////  t_finshed_no_contact->monitors_.push_back(m_regrasp_without_contact); // use same monitor as above
////
////  s1->monitors_.push_back(m_regrasp_without_contact);
//
//  //---------------------------------------------------------------------
//
//  Transition_ptr t_box_shifted_too_far(new ActionProgressTransition(5, s1, s5, "box shifted too far"));
//  t_box_shifted_too_far->deltaTmin_ = 0.0;
//  t_box_shifted_too_far->deltaTmax_ = 1.0;
//  s1->transitions_out_.push_back(t_box_shifted_too_far);
//  s5->transitions_in_.push_back(t_box_shifted_too_far);
//
//  MonitorRequest_ptr m_box_shifted(new MonitorRequest("Progress", 5, 0, "box shifted too far", 0.3));
//
//  if (rightHandRegrasp)
//  {
//    //  m_box_shifted->addSignal("forceNonGravityRight", externalCollisionForce, false);
////    m_box_shifted->addSignal("normalForceLeft", handForceLoaded, false);
////    m_box_shifted->addSignal("forceGravityLeft", handForceLoaded, false);
//    m_box_shifted->addSignal("boxRotationError", maxRotationError, false);
//  }
//  else
//  {
//    //  m_box_shifted->addSignal("forceNonGravityRight", externalCollisionForce, false);
////    m_box_shifted->addSignal("normalForceRight", handForceLoaded, false);
////    m_box_shifted->addSignal("forceGravityRight", handForceLoaded, false);
//    m_box_shifted->addSignal("boxRotationError", maxRotationError, false);
//  }
//  t_box_shifted_too_far->monitors_.push_back(m_box_shifted); // use same monitor as above
//
//  s1->monitors_.push_back(m_box_shifted);
//
//  //---------------------------------------------------------------------
//
////  Transition_ptr t_box_remains_in_contact(new ActionProgressTransition(6, s0, s6, "box follows during regrasp"));
////  t_box_remains_in_contact->deltaTmin_ = 0.0;
////  t_box_remains_in_contact->deltaTmax_ = 1.0;
////  s0->transitions_out_.push_back(t_box_remains_in_contact);
////  s6->transitions_in_.push_back(t_box_remains_in_contact);
////
////  MonitorRequest_ptr
////      m_box_remains_in_contact(new MonitorRequest("Progress", 6, 0, "box follows during regrasp", 0.0));
////
////  if (rightHandRegrasp)
////  {
////    //  m_box_remains_in_contact_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
////    m_box_remains_in_contact->addSignal("normalForceRight", handForceLoaded, false);
////    m_box_remains_in_contact->addSignal("forceGravityRight", handForceLoaded, false);
////    m_box_remains_in_contact->addSignal("handDistanceRight", positionError, false);
////  }
////  else
////  {
////    //  m_box_remains_in_contact_r->addSignal("forceNonGravityRight", externalCollisionForce, false);
////    m_box_remains_in_contact->addSignal("normalForceLeft", handForceLoaded, false);
////    m_box_remains_in_contact->addSignal("forceGravityLeft", handForceLoaded, false);
////    m_box_remains_in_contact->addSignal("handDistanceLeft", positionError, false);
////  }
////
////  t_box_remains_in_contact->monitors_.push_back(m_box_remains_in_contact); // use same monitor as above
////
////  s0->monitors_.push_back(m_box_remains_in_contact);
//
//  //---------------------------------------------------------------------
//
//  Transition_ptr t_box_aligned_again(new ActionProgressTransition(7, s5, s1, "box re-aligned"));
//  t_box_aligned_again->deltaTmin_ = 0.0;
//  t_box_aligned_again->deltaTmax_ = 10.0;
//  s5->transitions_out_.push_back(t_box_aligned_again);
//  s1->transitions_in_.push_back(t_box_aligned_again);
//
//  MonitorRequest_ptr m_box_realigned(new MonitorRequest("Progress", 7, 0, "box re-aligned", 0.3));
//  m_box_realigned->addSignal("boxRotationError", maxRotationError, true);
//
//  t_box_aligned_again->monitors_.push_back(m_box_realigned);
//
//  s5->monitors_.push_back(m_box_realigned);
//
//  Transition_ptr t_attention_error(new ActionProgressTransition(8, s1, s7, "partner attention lost"));
//  t_attention_error->deltaTmin_ = 0.0;
//  t_attention_error->deltaTmax_ = 1.0;
//  s1->transitions_out_.push_back(t_attention_error);
//  s7->transitions_in_.push_back(t_attention_error);
//
//  MonitorRequest_ptr m_checkGaze(new MonitorRequest("Progress", 8, 0, "partner not paying attention", gazeAttentionDuration));
//  m_checkGaze->addSignal("gazeError", gazeAttentionThreshold, false);
//
//  t_attention_error->monitors_.push_back(m_checkGaze);
//  s0->monitors_.push_back(m_checkGaze);
//
//  Transition_ptr t_attention_recover(new ActionProgressTransition(9, s7, s1, "partner attention regained"));
//  t_attention_recover->deltaTmin_ = 0.0;
//  t_attention_recover->deltaTmax_ = 10.0;
//  s7->transitions_out_.push_back(t_attention_recover);
//  s1->transitions_in_.push_back(t_attention_recover);
//
//  MonitorRequest_ptr m_checkGazeGood(new MonitorRequest("Progress", 9, 0, "partner paying attention", gazeAttentionDuration));
//  m_checkGazeGood->addSignal("gazeError", gazeAttentionThreshold, true);
//
//  t_attention_recover->monitors_.push_back(m_checkGazeGood);
//  s7->monitors_.push_back(m_checkGazeGood);
//
//
//
//  tmp->states_.push_back(s0);
//  tmp->states_.push_back(s1);
//  tmp->states_.push_back(s2);
//  tmp->states_.push_back(s3);
//  tmp->states_.push_back(s4);
//  tmp->states_.push_back(s5);
//  tmp->states_.push_back(s6);
//  tmp->states_.push_back(s7);
//
//  tmp->transitions_.push_back(t_hand_release);
//  tmp->transitions_.push_back(t_hand_recontact);
////  tmp->transitions_.push_back(t_external_contact);
////  tmp->transitions_.push_back(t_collision);
////  tmp->transitions_.push_back(t_finshed_no_contact);
//  tmp->transitions_.push_back(t_box_shifted_too_far);
////  tmp->transitions_.push_back(t_box_remains_in_contact);
//  tmp->transitions_.push_back(t_box_aligned_again);
//  tmp->transitions_.push_back(t_attention_error);
//  tmp->transitions_.push_back(t_attention_recover);
//
//  return tmp;
//}



bool ActionProgressMonitor::monitor(std::string action, Rcs::SearchNode_ptr goal, double duration, double startTime)
{
  bool actionFound = false;
  Apg_ptr a_ptr;

  for (size_t i = 0; i < actions_.size(); i++)
  {
    //TODO: maybe compare with description here?
    if (actions_[i]->id_==action)
    {
      actionFound = true;
      a_ptr = actions_[i];
    }
  }

  if (!actionFound)
  {
    currentAction_.reset();
    currentState_.reset();
    return false;
  }

  return monitor(a_ptr, goal, duration, startTime);
}

bool ActionProgressMonitor::monitor(Apg_ptr action, Rcs::SearchNode_ptr goal, double duration, double startTime)
{
  actionStartTime_ = startTime;
  currentAction_ = action;
  currentAction_->duration_ = duration;
  currentAction_->setStartStop(SearchNode_ptr(), goal);

  currentState_.reset();

  if (!currentAction_->states_.empty())
  {
    currentState_ = currentAction_->states_[0];
  }

  RLOG(0, "Monitoring set up for action '%s' - '%s': ", action->id_.c_str(), action->description_.c_str());
  //
  //  action->print();

  //  for (size_t s = 0; s < action->states_.size(); s++)
  //  {
  //    action->states_[s]->print();
  ////    RLOG(0, "State #%d '%s': ", (int) s, action->states_[s]->description_.c_str());
  //    ManipulationState_ptr man = action->states_[s];
  //    for (size_t m = 0; m < man->monitors_.size(); m++)
  //    {
  //      RLOG(0, "   Monitor #%d (%s): '%s' - Value: %5.3f, Thres: %5.3f", (int) m, man->monitors_[m]->id_.c_str(), man->monitors_[m]->description_.c_str(), man->monitors_[m]->value_, man->monitors_[m]->threshold_);
  //    }
  //  }

  if (!currentState_)
  {
    RLOG(1, "Trying to monitor action progress, but no matching state was found.");
    return false;
  }

  return true;
}

void ActionProgressMonitor::addActions(std::vector<Apg_ptr> actions_new)
{
  this->actions_.reserve(this->actions_.size() + actions_new.size());
  for (size_t i = 0; i < actions_new.size(); i++)
  {
    this->actions_.push_back(actions_new[i]);
  }
  RLOG(0, "Added %d new action progress graphs. Total: %d", (int) actions_new.size(), (int) this->actions_.size());
}


std::vector<MonitorRequest> ActionProgressMonitor::getRequests()
{
  //TODO: check this!!!
  std::vector<MonitorRequest> mon;

  if (currentState_)
  {
    //    RLOG(0, "AAAAA");
    mon = currentState_->getRequests();

    //    RLOG(0, "Requests for action progress monitor: %d", (int) mon.size());
    for (size_t i = 0; i < mon.size(); i++)
    {
      //      RLOG(0, "  %d: %s - %s", (int) i, mon[i].topic.c_str(), mon[i].description.c_str());
      mon[i].minTime *= currentAction_->duration_;
      mon[i].maxTime *= currentAction_->duration_;
      mon[i].creationTime = actionStartTime_;
      mon[i].lastDeactivationTime = mon[i].creationTime;
    }


    //    RLOG(0, "BBBBB");
    //    for (size_t i = 0; i < currentState_->monitors_.size(); i++)
    //    {
    //      MonitorRequest tmp(currentState_->monitors_[i]);
    //      RLOG(0, "xxx %d Mon: %d(%s), %d(%s)", (int) i, tmp.id, std::to_string(tmp.id).c_str(), currentState_->monitors_[i]->id, std::to_string(currentState_->monitors_[i]->id).c_str() );
    //      mon.push_back(tmp);
    //    }
  }
  else
  {
    RLOG(0, "No monitoring set up...");
  }


  return mon;
}

void ActionProgressMonitor::print()
{

}

void ActionProgressMonitor::stop()
{
  currentAction_.reset();
}

bool ActionProgressMonitor::onMonitorTrigger(int id)
{


  if (currentState_)
  {
    RLOG_CPP(0, "Num trans out:" << currentState_->transitions_out_.size());
  }
  else
  {
    RLOG(0, "!!!!!!!!!!!!!  No current state, but trigger received! Some monitors did not get cleared in time.");
    return false;
  }

  RLOG(0, "[%s] Trigger received: %d", currentState_->description_.c_str(), id);


  // find matching transition
  for (size_t t = 0; t < currentState_->transitions_out_.size(); t++)
  {
    //    RLOG(0, "Checking transition %d / %d with id %d... looking for id %d", (int) t + 1, (int) currentState_->transitions_out_.size(),  currentState_->transitions_out_[t]->id_, id);

    if (currentState_->transitions_out_[t]->id_==id)
    {
      ActionProgressState_ptr suc = currentState_->transitions_out_[t]->stop_.lock();
      if (!suc)
      {
        RLOG(0, "Invalid transition goal!");
        continue;
      }


      RLOG(0, "State changed from '%s' to '%s'.", currentState_->description_.c_str(), suc->description_.c_str());
      currentState_ = suc;


      return true;
    }
    else
    {
      //      RLOG(0, "No match.");
    }

  }
  return false;
}

void ActionProgressMonitor::triggerStateChangeActions(EntityBase* entity)
{
  if (!currentState_)
  {
    return;
  }

  RLOG(0, "Triggering actions on state change:");
  for (size_t i = 0; i < currentState_->enterEvents.size(); i++)
  {
    RLOG(0,
         "   %d '%s' - '%s'",
         (int) i,
         currentState_->enterEvents[i].first.c_str(),
         currentState_->enterEvents[i].second.c_str());

    if (currentState_->enterEvents[i].first == "StopActions")
    {
      entity->publish("StopActions");
    }
    else if (currentState_->enterEvents[i].first == "ResumePlan")
    {
      entity->publish("ResumePlan");
    }
    else if (currentState_->enterEvents[i].first == "PausePlan")
    {
      entity->publish("PausePlan");
    }
    else
    {
      entity->publish(currentState_->enterEvents[i].first, currentState_->enterEvents[i].second);
    }
  }
}


//bool ActionProgressMonitor::updateMonitor(std::string id, double value)
//{
//  std::map<std::string, Monitor_ptr>::iterator it;
//
//  it = (monitors_.find(id));
//
//  if (it != monitors_.end())
//  {
//    it->second->setValue(value);
//    return true;
//  }
//
//  return false;
//}

//std::string ActionProgressMonitor::getMonitorStateString(std::vector<std::shared_ptr<Monitor> > monitors)
//{
//  std::string observedFeatures = "";
//
//  for (size_t m = 0; m < monitors.size(); m++)
//  {
//    bool monitorPositive = monitors[m]->evaluate();
//    observedFeatures += monitorPositive ? "1":"0";
//  }
//
//  return observedFeatures;
//}

}
