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

#ifndef RCS_VIRTUALKINECT2_H
#define RCS_VIRTUALKINECT2_H

#include "ComponentBase.h"

#include <osg/Node>

#include <thread>


namespace Rcs
{

class VirtualKinect2 : public ComponentBase
{
public:

  VirtualKinect2(EntityBase* parent);

  virtual ~VirtualKinect2();

  void addNode(osg::Node* node);
  void removeNode(std::string nodeName);

private:

  void onStart();
  void onStop();
  void onRenderCommand(std::string graphId, std::string command);
  void onRemoveNode(std::string graphId, std::string nodeName);
  void onAddNode(osg::ref_ptr<osg::Node> node);
  void onAddChildNode(osg::ref_ptr<osg::Node> node,
                      std::string graphId,
                      std::string parent);

  void threadFunc();

  std::mutex nodeMtx;
  std::vector<osg::ref_ptr<osg::Node>> nodesToAdd;
  std::vector<std::string> nodesToRemove;
  std::vector<HTr> camTransforms;

  std::thread rosThread;
  bool threadRunning;

  VirtualKinect2(const VirtualKinect2&);
  VirtualKinect2& operator=(const VirtualKinect2&);
};

}

#endif // RCS_VIRTUALKINECT2_H
