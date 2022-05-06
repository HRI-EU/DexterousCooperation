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

#include "ColliderComponent.h"

#include <Rcs_joint.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_body.h>


namespace Rcs
{

ColliderComponent::ColliderComponent(EntityBase* parent,
                                     std::vector<std::string> b1,
                                     std::vector<std::string> b2,
                                     bool useCurrent) :
  ComponentBase(parent), useCurrentGraph(useCurrent), distance(1.0),
  bodies1(b1), bodies2(b2)
{
  RCHECK((getEntity()->hasRegisteredEvent<RcsGraph*, RcsGraph*>("PostUpdateGraph")));
  subscribe("PostUpdateGraph", &ColliderComponent::onPostUpdateGraph);
}

ColliderComponent::~ColliderComponent()
{
}

void ColliderComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  const RcsGraph* graph = useCurrentGraph ? current : desired;
  std::vector<const RcsBody*> b1, b2;

  RCSGRAPH_TRAVERSE_BODIES(graph)
  {
    for (size_t i=0; i<bodies1.size(); ++i)
    {
      if (std::string(BODY->name) == bodies1[i])
      {
        b1.push_back(BODY);
      }
    }

    for (size_t i=0; i<bodies2.size(); ++i)
    {
      if (std::string(BODY->name) == bodies2[i])
      {
        b2.push_back(BODY);
      }
    }

  }   // RCSGRAPH_TRAVERSE

  double d = std::numeric_limits<double>::max();

  for (auto bdy1 : b1)
  {
    for (auto bdy2 : b2)
    {
      d = std::min(d, RcsBody_distance(bdy1, bdy2, NULL, NULL, NULL));
    }
  }

  if ((distance > 0.0) && (d <= 0.0))
  {
    getEntity()->publish("IsColliding", true);
    getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", "RED");
  }
  else if ((distance<=0.0) && (d > 0.0))
  {
    getEntity()->publish("IsColliding", false);
    getEntity()->publish<std::string, std::string>("RenderCommand", "BackgroundColor", "");
  }

  this->distance = d;
  RLOG(1, "d=%f", d);
}



}   // namespace Rcs
