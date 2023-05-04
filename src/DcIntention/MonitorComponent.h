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

#ifndef DC_MONITORCOMPONENT_H
#define DC_MONITORCOMPONENT_H

#include "ComponentBase.h"
#include "MonitorRequest.h"
#include "MonitorWidgetBase.h"

#include <Rcs_graph.h>

#include <memory>
#include <map>


namespace Dc
{

class MonitorRequestWidget;
class ObjectModel;

//enum class MonitorThresholds : int
//{
//  FORCE_STOP = 0,
//  ROTATION_COMMAND_FORCE_DIFF_TILT,
//  ROTATION_COMMAND_FORCE_DIFF_ROTATE,
//  HAND_FORCE_LOADED,
//  HAND_FORCE_UNLOADED,
//  ACTIVATION_TIME,
//  NUM_THRESHOLDS
//};

/*!
 *  \brief MonitorComponent handles monitoring and checking for specific sensor cues. It receives MonitorRequest
 *         that specify combinations of several sensor signals and durations. Once their condition is fulfilled,
 *         an event is emitted on the specified topic. A GUI (defined by MonitorWidgetBase and MonitorRequestWidget)
 *         visualizes all monitored sensor signal and current MonitorRequests, and allows to manually override
 *         sensor signals and trigger requests.
 *
 *         The class publishes the following events:
 *         - SetTextLine: In subscriber "PostUpdateGraph"
 *         - Display: In "updateGraph"
 *         - MonitorStatusChanged: Status of each monitor request
 *         - [topic]: Fulfilled monitor requests with topic "[topic]" (e.g. [topic] = "Intention", "Confirmation", "Progress")
 *
 *
 *         The class subscribes to the following events:
 *         - PostUpdateGraph: Updates the own graph.
 *         - Start: Starts the component.
 *         - Stop: Stops the component.
 *         - RequestMonitor: Receives requests to look for sensor cues (MonitorRequest)
 *         - ResetAndRequestMonitor: As RequestMonitor, but first clears existing requests
 *         - ClearMonitors: Clears existing requests for a given namespace.
 *
 */
class MonitorComponent : public ComponentBase
{
public:

  /*!
   *  \brief MonitorComponent listens for MonitorRequests, maintains a current list, and continuously checks if they
   *         have been fulfilled. If fulfilled, an event is emitted on the respective topic.
   *  \param[in] entity           Pointer to the EntityBase to be used
   *  \param[in] graph_           A graph to clone for internal use
   *  \param[in] model            The ObjectModel that defines all the available sensor cues and how they are computed.
   *  \param[in] useRealSensors   Boolean to define if sensor values should be updated automatically (true) or just
   *                              the values set in the GUI are to be used (false).
   *  \param[in] showGui          Boolean to determine if the GUI should be opened.
   *  \param[in] autoPublish_     Boolean to determine if fulfilled MonitorRequests should automatically emit event.
   *                              Default for normal operation is true. Otherwise no events will be published even if
   *                              their conditions are fulfilled. Disabling is useful for pausing behavior and debugging.
   *  \param[in] autoConfirm_     Boolean to determine if confirmation request should automatically be fulfilled.
   *                              Useful for work in simulation and debugging.
   */
  MonitorComponent(EntityBase* entity, const RcsGraph* graph_, std::shared_ptr<ObjectModel> model, bool useRealSensors, bool showGui, bool autoPublish_ = true, bool autoConfirm_ = false);
  ~MonitorComponent();

  /*!
   *  \brief Provides the name of the component.
   */
  std::string getName() const
  {
    return "Monitor Component";
  }

  /*!
   *  \brief Allows to enable/disable the GUI.
   */
  void enableGui(bool enableGui)
  {
    gui = enableGui;
  };

  /*!
   *  \brief Allows to alter if real sensor values are to be used.
   */
  void useRealSensors(bool useRealSensors)
  {
    realSensors = useRealSensors;
  };

  /*!
   *  \brief Allows enable/disable automatic publishing.
   */
  void enableAutoPublish(bool enableAutoPublish)
  {
    autoPublish = enableAutoPublish;
  };




private:
  void start();
  void stop();

  /*!
   *  \brief Process all updates in the graph. Compute updated sensor signals, synchronize the GUI, and
   *         check new and existing MonitorRequests.
   */
  void postUpdateGraph(RcsGraph* desiredGraph, RcsGraph* currentGraph);

  /*!
   *  \brief Synchronize the GUI to display current sensor values, or update sensor values from GUI setting.
   *         Update current monitor requests and their status.
   */
  void syncGui();

  /*!
   *  \brief Check which MonitorRequest has been satisfied.
   */
  void processRequests();


  void addMonitorRequest(MonitorRequest req);
  void clearMonitorRequests(std::string type = "");
  void resetAndAddMonitorRequest(MonitorRequest req);
  void removeMonitorRequest(std::vector<MonitorRequest>::iterator it);

  RcsGraph* graph;
  std::map<std::string, double> sensorData;


  //TODO: make this shared_pointers and they can be used directly by the GUI as well
  std::vector<MonitorRequest> requests;
  std::vector<MonitorRequest> addedRequests;
  std::vector<MonitorRequest> removedRequests;
  bool gui;
  bool realSensors;
  bool autoPublish;
  bool autoConfirm;
  bool publishOnce;

  double newTime;
  double lastTime;

  MonitorWidgetBase* monitorWidgetBase;
  int monitorWidgetBaseHandle;
  std::vector<MonitorRequestWidget> widgets;

  std::shared_ptr<ObjectModel> objModel;
};

}
#endif // DC_MONITORCOMPONENT_H
