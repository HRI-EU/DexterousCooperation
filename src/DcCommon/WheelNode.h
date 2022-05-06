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

#ifndef RCS_WHEELNODE_H
#define RCS_WHEELNODE_H

#include "WheelStrategy7D.h"

#include <CapsuleNode.h>
#include <VertexArrayNode.h>
#include <Rcs_MatNd.h>
#include <Rcs_macros.h>

#include <vector>


namespace Rcs
{
class WheelNode: public VertexArrayNode
{
public:

  WheelNode(const WheelStrategy7D* strategy) :
    VertexArrayNode(osg::PrimitiveSet::LINE_STRIP, "RED"),
    goalSphereRadius(0.03), traj(NULL)
  {

    std::vector<HTr> sState = strategy->getWheelTrajectory();

    for (size_t i=0; i<sState.size(); ++i)
    {
      osg::ref_ptr<CapsuleNode> cn = new CapsuleNode(sState[i].org, NULL, goalSphereRadius, 0.0);
      cn->setWireframe(false);
      cn->setMaterial("RED");
      addChild(cn.get());
    }

  }

  ~WheelNode()
  {
  }

  bool frameCallback()
  {

    return false;
  }

  std::vector<CapsuleNode*> sNode;
  double goalSphereRadius;
  MatNd* traj;
};
}

#endif   // RCS_WHEELNODE_H
