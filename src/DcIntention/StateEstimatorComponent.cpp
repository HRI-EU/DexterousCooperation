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

#include "StateEstimatorComponent.h"
#include "EntityBase.h"

#include <SearchNode.h>



namespace Rcs
{

StateEstimatorComponent::StateEstimatorComponent(EntityBase* parent,
                                                 std::shared_ptr<ObjectModel> model) :
  ComponentBase(parent), objModel(model)
{
  desiredState.resize(objModel->getStateDimension(), 0);
  currentState.resize(objModel->getStateDimension(), 0);
  subscribe("PostUpdateGraph", &StateEstimatorComponent::postUpdateGraph);
}


StateEstimatorComponent::~StateEstimatorComponent()
{
}

void StateEstimatorComponent::postUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  std::vector< std::vector<double> > offsets;
  std::vector<int> stateResult;
  bool stateChanged = false;

  // calculate state from desired graph
  stateResult = objModel->getCurrentState(offsets, desired);

  if (!std::equal(stateResult.begin(), stateResult.end(), desiredState.begin()))
  {
    stateChanged = true;
  }
  desiredState = stateResult;

  // calculate state from current graph
  stateResult = objModel->getCurrentState(offsets, current);

  if (!std::equal(stateResult.begin(), stateResult.end(), currentState.begin()))
  {
    stateChanged = true;
  }
  currentState = stateResult;

  std::string text = "State: " + Gras::vecToStr(currentState);
  getEntity()->publish("SetTextLine", text, 2);

  // only published changed states
  if (stateChanged)
  {
    getEntity()->publish("State", desiredState, currentState);
  }

}

}   // namespace Rcs
