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

#ifndef RCS_REMOTEVISUALIZATIONCOMPONENT_H
#define RCS_REMOTEVISUALIZATIONCOMPONENT_H

#define RAPIDJSON_HAS_STDSTRING 1
#include <rapidjson/stringbuffer.h>
#include <rapidjson/document.h>
#include <rapidjson/writer.h>

#include "EntityBase.h"
#include "ComponentBase.h"

#include <map>
#include <vector>

#define ASIO_STANDALONE
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <PeriodicCallback.h>
#include <Rcs_graph.h>
#include <Rcs_macros.h>

#include <thread>

namespace Rcs
{
typedef websocketpp::server<websocketpp::config::asio> WebsocketServer;

/*! \ingroup RcsMotionControlLayer
 *  \brief HardwareComponent to convey current state and intention information
 *         to a visualizer hosted on a remote machine.
 */
class RemoteVisualizationComponent : ComponentBase
{
public:
  enum class PromptType
  {
    MoveLeft,
    MoveRight,
    YourTurn,
    MyTurn,
    None
  };

  enum class GraphType
  {
    Desired,
    Current,
    None
  };

  std::string GraphTypeStr(GraphType g)
  {
    switch (g)
    {
      case GraphType::Desired:
        return "desired";
      case GraphType::Current:
        return "current";
      default:
        return "none";
    }
  }

  RemoteVisualizationComponent(EntityBase* parent, RcsGraph* _graph, unsigned int _port = 8080);
  virtual ~RemoteVisualizationComponent();

  virtual void postUpdateGraph(RcsGraph* desiredGraph, RcsGraph* currentGraph);
  virtual void updateGraph(RcsGraph* graph);
  virtual void updateDesiredGraph(const MatNd* q);
  virtual void setCommand(const MatNd* q_des, const MatNd* qp_des,
                          const MatNd* T_des);

  virtual bool checkEmergencyCondition();
  virtual std::string getName() const
  {
    return "RemoteVisualizationComponent";
  };

  virtual void onEmergencyStop(const ComponentBase* fault);
  virtual bool onEmergencyRecovery();

  void runThread(double freq);

  virtual void start();
  virtual void stop();

  void setAttention(const std::string& name, double attention);
  void perform(const std::string& action);
  void prompt(const PromptType& p, std::initializer_list<std::string> gazeTargets = {}); /// TODO: There is probably a better option than initializer_list
  void clearPrompt();
  void setDesiredBoxAngle(const double angle);
  void setDesiredHandPoses(double posX_r, double posY_r, double posZ_r, double posX_l, double posY_l, double posZ_l);

  void onNewObjectModel(std::vector<HTr> grasps, std::vector<HTr> partnerGrasps);
  void display_update(const std::string displayType);

  template <typename T>
  void setProperty(const std::string& name, const T& value)
  {
    if (connected)
    {
      sendMsg(setPropertyMsg(name, value));
    }
    //    else
    //    {
    //      RLOG(0, "WTF");
    //    }
  }

  template <typename T>
  void setProperties(const std::vector<std::string>& names, const std::vector<T>& values)
  {
    if (connected)
    {
      sendMsg(setPropertiesMsg(names, values));
    }
  }

  void prompt_LeanLeft();
  void prompt_LeanRight();
  void prompt_Confirm();
  void prompt_Clear();

private:
  pthread_mutex_t mtx;
  std::thread bgThread;
  bool started = false;
  bool connected = false;
  unsigned int port;
  WebsocketServer server;
  websocketpp::connection_hdl hdl; /// TODO: Support multiple connections?
  double prompt_interval = 2.20;
  double prompt_t = prompt_interval;
  unsigned int prompt_idx = 0;
  PromptType currPrompt = PromptType::None;

  size_t msgStamp = 0;
  const size_t max_decimals = 16;
  RcsGraph* graphCur = nullptr;
  RcsGraph* graphDes = nullptr;
  std::vector<RcsBody*> trackedBodies_once; // send position data once
  std::vector<RcsBody*> trackedBodies_always_des; // send position data every tick
  std::vector<RcsBody*> trackedBodies_always_curr; // send position data every tick

  void sendMsg(const std::string& msg);
  void sendMsg(const char* msg, size_t len);

  std::string updatePoseMsg(const std::string objname, const double position[3], double rotation[3][3], GraphType g);
  std::string updatePoseMsg(const std::string objname, HTr* transform, GraphType g);
  std::string updatePoseMsg(const std::string objname, const double position[3], double rotation[3][3], const double velocity[3], GraphType g);
  std::string updatePoseMsg(const std::string objname, HTr* transform, const double velocity[3], GraphType g);
  std::string setAttentionMsg(const std::string objName, const double attention);
  std::string setAttentionMsg(const double position[3], const double attention);
  std::string setEmergencyMsg(const bool emergencyActive, const std::string reason);
  std::string performActionMsg(const std::string& action);
  std::string setDesiredBoxAngleMsg(const double angle);
  std::string setDisplayMsg(const std::string display);
  std::string objectModelMsg(std::vector<HTr> grasps);

  template <typename T>
  std::string setPropertyMsg(const std::string& propertyName, T& propertyValue)
  {
    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
    writer.SetMaxDecimalPlaces(max_decimals);
    rapidjson::Document d;
    d.SetObject();
    d.AddMember("op", "SetProperty", d.GetAllocator());
    d.AddMember("stamp", msgStamp++, d.GetAllocator());
    d.AddMember("name", propertyName, d.GetAllocator());
    d.AddMember("value", propertyValue, d.GetAllocator());
    d.Accept(writer);

    return buf.GetString();
  }

  template <typename T>
  std::string setPropertiesMsg(const std::vector<std::string>& propertyNames, const std::vector<T>& propertyValues)
  {
    RCHECK_MSG(propertyNames.size() == propertyValues.size(), "Name/Value lists must be of equal length! (got: %zu / %zu)", propertyNames.size(), propertyValues.size());

    rapidjson::StringBuffer buf;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
    rapidjson::Document d;
    rapidjson::Document::AllocatorType& alloc = d.GetAllocator();
    d.SetObject();
    d.AddMember("op", "SetProperties", alloc);
    d.AddMember("stamp", msgStamp++, alloc);

    rapidjson::Value properties(rapidjson::kArrayType);
    for (size_t i = 0; i < propertyNames.size(); i++)
    {
      rapidjson::Value prop(rapidjson::kObjectType);
      prop.AddMember(rapidjson::Value(propertyNames[i], alloc).Move(), rapidjson::Value(propertyValues[i], alloc).Move(), alloc);
      properties.PushBack(prop, alloc);
    }
    d.AddMember("properties", properties, alloc);

    d.Accept(writer);

    return buf.GetString();
  }


  std::vector<std::string> prompt_targets;
  std::map<PromptType, std::vector<std::string>> prompts =
  {
    {PromptType::MoveLeft,  {"hint_l_2", "push_l"}},
    {PromptType::MoveRight, {"hint_r_2", "push_r"}},
    {PromptType::YourTurn,  {"push",   "nod_quick"}},
    {PromptType::MyTurn,    {"nod",    "nod_quick"}}
  };

  std::map<std::string, std::vector<double>> base_orientations_des;
  std::map<std::string, std::vector<double>> base_orientations_cur;

};

}

#endif   // RCS_REMOTEVISUALIZATIONCOMPONENT_H
