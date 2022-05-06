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

#ifndef RCS_COLLIDERCOMPONENT_H
#define RCS_COLLIDERCOMPONENT_H


#include "ComponentBase.h"

#include <Rcs_graph.h>



namespace Rcs
{
/*! \brief To do
 */
class ColliderComponent : public ComponentBase
{
public:

  /*! \brief Constructs GraphComponent with a clone of the passed graph.
   *
   * \param[in] parent    Entity class responsible for event subscriptions
   * \param[in] graph     Underlying RcsGraph structure. The class will create
   *                      a copy of the argument.
   */
  ColliderComponent(EntityBase* parent, std::vector<std::string> bodies1,
                    std::vector<std::string> bodies2, bool useCurrentGraph);

  /*! \brief Unsubscribes and deletes all previously allocated memory.
   *         There is no thread that needs to be stopped.
   */
  virtual ~ColliderComponent();

private:

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);

  bool useCurrentGraph;
  double distance;
  std::vector<std::string> bodies1, bodies2;

  ColliderComponent(const ColliderComponent&);
  ColliderComponent& operator=(const ColliderComponent&);
};

}

#endif   // RCS_COLLIDERCOMPONENT_H
