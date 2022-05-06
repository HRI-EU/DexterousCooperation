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

#ifndef RCS_HOLOCOMPONENT_H
#define RCS_HOLOCOMPONENT_H

#include "ComponentBase.h"
#include "HoloNetworkConnection.h"

#include <Rcs_quaternion.h>
#include <ControllerBase.h>
#include <PeriodicCallback.h>

#include <algorithm>
#include <mutex>


namespace Rcs
{


// bitfield to select which graph(s) data should be pulled from
/// TODO: This should probably live elsewhere
enum class GraphType : unsigned
{
  None    = 0x0,
  Current = 0x1,
  Desired = 0x2,
  Other   = 0x4
};

GraphType operator|(const GraphType& lhs, const GraphType& rhs);
GraphType& operator|=(GraphType& lhs, const GraphType& rhs);
GraphType operator&(const GraphType& lhs, const GraphType& rhs);
GraphType& operator&=(GraphType& lhs, const GraphType& rhs);
GraphType operator^(const GraphType& lhs, const GraphType& rhs);
GraphType& operator^=(GraphType& lhs, const GraphType& rhs);
GraphType operator~(const GraphType& g);

class HoloComponent : public ComponentBase, public PeriodicCallback
{
public:

  HoloComponent(EntityBase* parent, std::string rootObj="All", unsigned int port_broadcast=26794, unsigned int port_receive=26795);
  virtual ~HoloComponent();

  void update(Hologram& hologram, const GraphType& graphType);
  void update(const std::string& name, const HTr* transform, const GraphType& graphType, const std::string& parent = "All", const HTr* relTransform = nullptr);
  void clear(const std::string& name, const std::string& topic, const GraphType& graphType);

  static std::string DIRECT_TRANSFORM();

private:

  void onStart();
  void onStop();
  void updateGraph(RcsGraph* graph);
  void postUpdateGraph(RcsGraph* graphDes, RcsGraph* graphCurr);
  bool parseHoloMessage(std::string message); /// TODO: move implementation to HoloNetworkConnection
  void syncViconFrame();

protected:
  void callback();   // Thread callback function, called every 20 msec
  void onTrackTransform(std::string name, GraphType graphType, std::string parent="All");
  void onUntrackTransform(std::string name, GraphType graphType);
  std::string addGraphSuffix(const std::string& name, const GraphType& graph);

  // Members
  std::string rootName; // name of root transform for holograms in an Rcs graph, e.g. the Vuforia marker we plan to sync to
  HoloNetworkConnection holoConn;
  unsigned int broadcastPort;
  unsigned int receivePort;
  mutable std::mutex mtx;
  std::vector<std::pair<std::string, GraphType>> trackedTransforms; // transforms which should be updated continuously

  HTr slamFrame;
  HTr lastSyncPoint;
  bool framesSynced;

  // Avoid copying this class
  HoloComponent(const HoloComponent&);
  HoloComponent& operator=(const HoloComponent&);
};

}

#endif   // RCS_HOLOCOMPONENT_H
