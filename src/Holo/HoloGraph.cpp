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

#include "EntityBase.h"   // Needs to go before HoloComponent.h due to MSVC redefining min and max
#include "HoloGraph.h"
#include "HoloParser.h"

#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_VecNd.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>
#include <Rcs_utils.h>


namespace Dc
{

HoloGraph::HoloGraph(EntityBase* parent, const RcsGraph* graph_, int port_) :
  ComponentBase(parent),
  Rcs::PeriodicCallback(),
  graph(RcsGraph_clone(graph_)),
  holoConn(),
  port(port_)
{
  setClassName("HoloGraph");

  getEntity()->subscribe("ComputeKinematics", &HoloGraph::onFwdKinematics, this);
  getEntity()->subscribe("Start", &HoloGraph::onStart, this);
  getEntity()->subscribe("Stop", &HoloGraph::onStop, this);
}

HoloGraph::~HoloGraph()
{
  stop();
  RcsGraph_destroy(this->graph);
}

void HoloGraph::onFwdKinematics(RcsGraph* graph_)
{
  // Copy the kinematic state (comprised in the vector q_des) into this classes
  // kinematic graph. The forward kinematics on the graph is calculated in the
  // callback() function, which is running concurrently with this function. We
  // therefore use the mutex.
  mtx.lock();
  MatNd_copy(graph->q, graph_->q);
  mtx.unlock();

}

void HoloGraph::onStart()
{
  // This method gets called once the "Start" event has been published (see
  // main() function. The update frequency of the callback() function is set to
  // 10Hz here (just as an example).
  setUpdateFrequency(30.0);
  PeriodicCallback::start();

  // Start the network connection to the Holo-Lens
  bool success = holoConn.startServer(this->port);

  if (success==false)
  {
    RLOG(0, "FAILED to start server");
  }

}

void HoloGraph::onStop()
{
  stop();
  holoConn.stopServer();
}

// This callback function gets called after the start() method has been called.
// It is repeated with the given update frequency. In such a thread, the
// communication with the HoloLens could for instance be made.
void HoloGraph::callback()
{

  // Compute forward kinematics: updates all transformations
  mtx.lock();
  RcsGraph_setState(graph, NULL, NULL);

  std::string nwMsg;
  if (getLoopCount() % 60 == 0)
  {
    nwMsg = addRcsGraphMsg(graph, "");
  }
  else
  {
    nwMsg = updateRcsGraphMsg(graph, "");
  }

  holoConn.sendMessage(nwMsg);
  mtx.unlock();

  RLOG(4, "%s", nwMsg.c_str());

  char text[256];
  snprintf(text, 256, "Messages    sent: %d   freq: %.1f   send queue: %zd",
           (int) holoConn.getMessagesSent(),
           holoConn.getDesiredSendFrequency(),
           holoConn.getSendQueueSize());
  getEntity()->publish("SetTextLine", std::string(text), 1);
}

}   // namespace
