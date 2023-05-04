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

#include "ActionProgressGraph.h"

#include "ActionProgressState.h"
#include "ActionProgressTransition.h"
#include "MonitorRequest.h"

#include <fstream>
#include <SearchNode.h>

namespace Dc
{

ActionProgressGraph::ActionProgressGraph()
{
  id_ = "";
  description_ = "";
  states_.clear();
  transitions_.clear();
  monitors_.clear();
}

ActionProgressGraph::ActionProgressGraph(std::string id, std::string desc)
{
  id_ = id;
  description_ = desc;
  states_.clear();
  transitions_.clear();
  monitors_.clear();
}

ActionProgressGraph::~ActionProgressGraph()
{

}

void ActionProgressGraph::setStartStop(const std::vector<int>& start,
                                       const std::vector<int>& stop)
{
  planningStart_ = start;
  planningStop_ = stop;
}


//bool ActionProgressGraph::updateMonitor(std::string id, double value)
//{
//  bool found = false;
//
//  for (size_t i = 0; i < monitors_.size(); i++)
//  {
//    if (monitors_[i]->id_ == id)
//    {
//      monitors_[i]->setValue(value);
//      found = true;
//    }
//  }
//  return found;
//}

void ActionProgressGraph::print()
{
  printf("Action: '%s' - '%s': \n", id_.c_str(), description_.c_str());

  printf("States: \n");
  for (size_t s = 0; s < states_.size(); s++)
  {
    states_[s]->print();
  }

  printf(" Monitors: \n");
  for (size_t m = 0; m < monitors_.size(); m++)
  {
    printf("   Monitor #%d: \n %s ", (int) m, monitors_[m]->toStringLong().c_str());
  }

}

void ActionProgressGraph::writeDot(std::string filename)
{
  std::ofstream myfile;
  myfile.open(filename);

  myfile << "digraph " << id_ << "{" << std::endl;

  for (size_t i = 0; i < states_.size(); i++)
  {
    ActionProgressState_ptr s = states_[i];
    myfile << s->id_ << " [label=< <b>" << s->id_ << "</b> <br/> ";
    myfile << "<font point-size=\"11\">" << s->description_ << "</font><br/>";

    //    for (size_t m = 0; m < s->monitor_values_.size(); m++)
    //    {
    //      if (s->monitor_values_[m]==ActionProgressState::DONTCARE)
    //      {
    //        myfile << "x";
    //      }
    //      else
    //      {
    //        myfile << s->monitor_values_[m];
    //      }
    //    }

    if ((!planningStop_.empty()) && s->goal_ && s->success_)
    {
      myfile << "<br/>" << "[";

      std::vector<int> stateDesc = planningStop_;
      for (size_t j = 0; j < stateDesc.size(); j++)
      {
        myfile << stateDesc[j];
        if (j < stateDesc.size() - 1)
        {
          myfile << ",";
        }
      }
      myfile << "]";
    }

    myfile << ">]" << std::endl;

    if (s->goal_)
    {
      myfile << s->id_ << " [style=\"filled,bold\" fillcolor=\"" << (s->success_ ? "green" : "red") << "\"]"
             << std::endl;
    }
  }

  for (size_t i = 0; i < transitions_.size(); i++)
  {
    ActionProgressState_ptr s0 = transitions_[i]->start_.lock();
    ActionProgressState_ptr s1 = transitions_[i]->stop_.lock();

    if (s0 && s1)
    {
      myfile << s0->id_ << " -> " << s1->id_ << "[label=<" << transitions_[i]->id_ << ": ["
             << transitions_[i]->deltaTmin_*duration_ << "s , " << transitions_[i]->deltaTmax_*duration_ << "s], "
             << transitions_[i]->description_ << ">]" << std::endl;
    }
  }

  myfile << "}";
  myfile.close();
}

}
