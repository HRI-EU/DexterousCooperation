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

#include "ActionProgressState.h"

#include "ActionProgressGraph.h"
#include "ActionProgressTransition.h"
#include "MonitorRequest.h"

#include "Rcs_macros.h"

namespace Dc
{

ActionProgressState::ActionProgressState()
{
  id_ = "";
  description_ = "";
  goal_ = false;
  success_ = false;
  transitions_out_.clear();
  transitions_in_.clear();
}

ActionProgressState::~ActionProgressState()
{

}

//void ActionProgressState::setPlanningState(Rcs::SearchNode_ptr planningState)
//{
//  if (!planningState)
//  {
//    planningState_.reset();
//  }
//  else
//  {
//    planningState_ = planningState;
//  }
//}

std::vector<MonitorRequest> ActionProgressState::getRequests()
{
  std::vector<MonitorRequest> mon;

  //  RLOG(0, "Requesting monitors for state: '%s'", id_.c_str());
  //  RLOG(0, "%d outgoing transitions", (int) transitions_out_.size());
  for (size_t i = 0; i < transitions_out_.size(); i++)
  {
    //    RLOG(0, "  %d / %d: #monitors: %d", (int) i+1, (int) transitions_out_.size(), (int) transitions_out_[i]->monitors_.size() );
    for (size_t m = 0; m < transitions_out_[i]->monitors_.size(); m++)
    {
      //      RLOG(0, "        %d / %d Monitor %d", (int) m+1, (int) transitions_out_.size(), transitions_out_[i]->monitors_[m]->id );
      MonitorRequest tmp(transitions_out_[i]->monitors_[m]);
      tmp.minTime = transitions_out_[i]->deltaTmin_;
      tmp.maxTime = transitions_out_[i]->deltaTmax_;
      tmp.id = transitions_out_[i]->id_;
      mon.push_back(tmp);
    }
  }

  //  RLOG(0, "Requests for action progress monitor: %d", (int) mon.size());
  return mon;
}

void ActionProgressState::print()
{
  printf(" State '%s' - '%s': \n", id_.c_str(), description_.c_str());
  printf("  Goal: '%s', Success: '%s'\n", (goal_ ? "true": "false"), (success_ ? "true": "false"));

  printf("  Incoming transitions: \n");
  if (transitions_in_.empty())
  {
    printf("   ---\n");
  }
  else
  {
    for (size_t t = 0; t < transitions_in_.size(); t++)
    {
      transitions_in_[t]->print();
    }
  }

  printf("  Outgoing transitions: \n");
  if (transitions_out_.empty())
  {
    printf("   ---\n");
  }
  else
  {
    for (size_t t = 0; t < transitions_out_.size(); t++)
    {
      transitions_out_[t]->print();
    }
  }

  printf("  Monitors: \n");
  if (monitors_.empty())
  {
    printf("   ---\n");
  }
  else
  {
    for (size_t m = 0; m < monitors_.size(); m++)
    {
      printf("   Monitor #%d \n %s", (int) m, monitors_[m]->toStringLong().c_str());
    }
  }
}

//void ActionProgressState::onEnter()
//{
//  for (size_t i = 0; i < enterEvents.size(); i++)
//  {
//
//  }
//}

}
