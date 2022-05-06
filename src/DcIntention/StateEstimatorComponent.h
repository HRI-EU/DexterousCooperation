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

#ifndef RCS_STATEESTIMATORCOMPONENT_H
#define RCS_STATEESTIMATORCOMPONENT_H


#include "ComponentBase.h"
#include "ObjectModel.h"



namespace Rcs
{

/*! \brief StateEstimatorComponent computes the current state defined by the
 *         provided ObjectModel based on the current robot state and sensor
 *         readings, handles monitoring and checking for specific sensor cues.
 *
 *         The class publishes the following events:
 *         - SetTextLine: In subscriber "PostUpdateGraph"
 *         - State: In subscriber "PostUpdateGraph"
 *         - StateOffset_Desired: In subscriber "PostUpdateGraph"
 *         - StateOffset_Current: In subscriber "PostUpdateGraph"
 *
 *         The class subscribes to the following events:
 *         - PostUpdateGraph: Updates the own graph and computes the state
 *         - ChangeObject: Not implemented
 */
class StateEstimatorComponent : public ComponentBase
{

public:

  /*! \brief StateEstimatorComponent computes the current state defined by the
   *         provided ObjectModel based on the current robot state and sensor
   *         readings handles monitoring and checking for specific sensor cues.
   */
  StateEstimatorComponent(EntityBase* parent,
                          std::shared_ptr<ObjectModel> model);
  virtual ~StateEstimatorComponent();


private:

  /*! \brief Computes the current state and desired state based on the new
   *         graph. How the state is computed from the graph is defined in the
   *         provided object model. They are published together on event
   *         'State'. In addition, the offset between real situation and
   *         discretize state is published as 'StateOffset_Desired' and
   *         'StateOffset_Current'.
   *
   *  \param[in] desired        Updated desired graph
   *  \param[in] current        Updated current graph
   */
  void postUpdateGraph(RcsGraph* desired, RcsGraph* current);

  std::shared_ptr<ObjectModel> objModel;

  std::vector<int> desiredState;
  std::vector<int> currentState;
};

}

#endif   // RCS_STATEESTIMATORCOMPONENT_H
