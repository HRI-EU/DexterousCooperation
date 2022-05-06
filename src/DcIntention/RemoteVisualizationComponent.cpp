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

#include "RemoteVisualizationComponent.h"

#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_quaternion.h>


Rcs::RemoteVisualizationComponent::RemoteVisualizationComponent(EntityBase* parent, RcsGraph* _graph,
                                                                unsigned int _port) :
  Rcs::ComponentBase(parent),
  port(_port),
  graphCur(RcsGraph_clone(_graph)), graphDes(RcsGraph_clone(_graph))
{
  getEntity()->subscribe("PostUpdateGraph", &RemoteVisualizationComponent::postUpdateGraph, this);

  //  getEntity()->subscribe("UpdateGraph", &RemoteVisualizationComponent::updateGraph, this);
  //  getEntity()->subscribe("SetJointCommand", &RemoteVisualizationComponent::updateDesiredGraph, this);
  getEntity()->subscribe("Start", &RemoteVisualizationComponent::start, this);
  getEntity()->subscribe("Stop",  &RemoteVisualizationComponent::stop, this);
  getEntity()->subscribe("DesiredStateChanged", &RemoteVisualizationComponent::setDesiredBoxAngle, this);
  getEntity()->subscribe("DesiredHandPoses", &RemoteVisualizationComponent::setDesiredHandPoses, this);
  getEntity()->subscribe("Prompt.lean_left", &RemoteVisualizationComponent::prompt_LeanLeft, this);
  getEntity()->subscribe("Prompt.lean_right", &RemoteVisualizationComponent::prompt_LeanRight, this);
  getEntity()->subscribe("Prompt.confirm", &RemoteVisualizationComponent::prompt_Confirm, this);
  getEntity()->subscribe("Prompt.clear", &RemoteVisualizationComponent::prompt_Clear, this);
  getEntity()->subscribe("Display", &RemoteVisualizationComponent::display_update, this);
  getEntity()->subscribe("ObjectModelChanged", &RemoteVisualizationComponent::onNewObjectModel, this);

  pthread_mutex_init(&this->mtx, NULL);


  // configure ws server
  this->server.clear_access_channels(websocketpp::log::alevel::all);
  this->server.clear_error_channels(websocketpp::log::elevel::all);
  this->server.set_error_channels(websocketpp::log::elevel::warn   |
                                  websocketpp::log::elevel::rerror |
                                  websocketpp::log::elevel::fatal);
  this->server.set_message_handler([](websocketpp::connection_hdl hdl, WebsocketServer::message_ptr msg)
  {
    RLOG(3, "Received msg: '%s'", msg->get_payload().c_str());
  });
  this->server.set_fail_handler([this](websocketpp::connection_hdl hdl)
  {
    RLOG(0, "Websocket connection failed!");
  });
  this->server.set_open_handler([this](websocketpp::connection_hdl hdl)
  {
    this->hdl = hdl;
    this->connected = true;
    RLOG(2, "Connected to %s", this->server.get_con_from_hdl(hdl)->get_host().c_str());
  });
  this->server.set_close_handler([this](websocketpp::connection_hdl hdl)
  {
    this->connected = false;
    RLOG(2, "Connection to %s closed", this->server.get_con_from_hdl(hdl)->get_host().c_str());
    this->hdl.reset();
  });

  trackedBodies_once =
  {
    RcsGraph_getBodyByName(this->graphDes, "Screen"),
    RcsGraph_getBodyByName(this->graphDes, "scr_tl"),
    RcsGraph_getBodyByName(this->graphDes, "scr_bl"),
    RcsGraph_getBodyByName(this->graphDes, "scr_br")
  };

  trackedBodies_always_des =
  {
    RcsGraph_getBodyByName(this->graphDes, "PowerGrasp_L"),
    RcsGraph_getBodyByName(this->graphDes, "PowerGrasp_R"),
    RcsGraph_getBodyByName(this->graphDes, "PartnerGrasp_L"),
    RcsGraph_getBodyByName(this->graphDes, "PartnerGrasp_R"),
    RcsGraph_getBodyByName(this->graphDes, "Box_v")
  };

  trackedBodies_always_curr =
  {
    RcsGraph_getBodyByName(this->graphCur, "PowerGrasp_L"),
    RcsGraph_getBodyByName(this->graphCur, "PowerGrasp_R"),
    RcsGraph_getBodyByName(this->graphCur, "PartnerGrasp_L"),
    RcsGraph_getBodyByName(this->graphCur, "PartnerGrasp_R"),
    RcsGraph_getBodyByName(this->graphCur, "Box_real")
  };

  // initialize euler angles for all tracked objects
  //  for (const auto& b : trackedBodies_once)
  //  {
  //    double eulers[3] = {0, 0, 0};
  //    RLOG(0, "Getting eulers for: %s", b->name);
  //    Mat3d_toEulerAngles(eulers, b->A_BI->rot);
  //
  //    base_orientations.insert({b->name, {eulers[0], eulers[1], eulers[2]}});
  //
  //  }
  for (const auto& b : trackedBodies_always_des)
  {
    double eulers[3] = {0, 0, 0};
    Mat3d_toEulerAngles(eulers, b->A_BI.rot);

    base_orientations_des.insert({b->name, {eulers[0], eulers[1], eulers[2]}});
  }
  for (const auto& b : trackedBodies_always_curr)
  {
    double eulers[3] = {0, 0, 0};
    Mat3d_toEulerAngles(eulers, b->A_BI.rot);

    base_orientations_cur.insert({b->name, {eulers[0], eulers[1], eulers[2]}});
  }

  // init websocket server
  this->server.init_asio();
  this->server.set_reuse_addr(true);
  this->server.listen(asio::ip::tcp::v4(), this->port);
  this->server.start_accept();

}

Rcs::RemoteVisualizationComponent::~RemoteVisualizationComponent()
{
  stop();
  pthread_mutex_destroy(&this->mtx);
}

void Rcs::RemoteVisualizationComponent::postUpdateGraph(RcsGraph* desiredGraph, RcsGraph* currentGraph)
{
  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(this->graphCur, currentGraph->q, NULL);
  RcsGraph_setState(this->graphDes, desiredGraph->q, NULL);
  pthread_mutex_unlock(&this->mtx);
}


void Rcs::RemoteVisualizationComponent::updateGraph(RcsGraph* graph)
{
  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(this->graphDes, graph->q, NULL);
  pthread_mutex_unlock(&this->mtx);
}

void Rcs::RemoteVisualizationComponent::updateDesiredGraph(const MatNd* q)
{
  pthread_mutex_lock(&this->mtx);
  RcsGraph_setState(this->graphDes, q, NULL);
  pthread_mutex_unlock(&this->mtx);
}

void Rcs::RemoteVisualizationComponent::setCommand(const MatNd* q_des,
                                                   const MatNd* qp_des,
                                                   const MatNd* T_des)
{
}

void Rcs::RemoteVisualizationComponent::onEmergencyStop(const ComponentBase* fault)
{
  if (this->connected)
  {
    /// TODO: When we can get names from ComponentBase re-enable this
    //    sendMsg(setEmergencyMsg(true, (fault == NULL ? "Unknown Error" : fault->getName())));
    sendMsg(setEmergencyMsg(true, "Something Broke >:|"));
  }
}

bool Rcs::RemoteVisualizationComponent::onEmergencyRecovery()
{
  if (this->connected)
  {
    sendMsg(setEmergencyMsg(false, "Emergency state cleared"));
  }

  return true; // we have no safety-critical parts to hold up the robot with
}

bool Rcs::RemoteVisualizationComponent::checkEmergencyCondition()
{
  return false; // this component CANNOT fail in a way which impacts the robot's safety
}

void Rcs::RemoteVisualizationComponent::runThread(double freq)
{
  while (started)
  {
    static double t0 = Timer_getSystemTime();
    double t1 = Timer_getSystemTime();
    double dt = t1 - t0;
    t0 = t1;

    this->server.poll();

    if (this->connected && this->graphDes != NULL && this->graphCur != NULL)
    {
      static bool did_once = false;

      if (!did_once)
      {
        did_once = true;
        for (auto& obj : trackedBodies_once)
        {
          if (obj != NULL)
          {
            RLOG(0, "Processing obj %s", obj->name);
            pthread_mutex_lock(&this->mtx);
            auto msg = updatePoseMsg(obj->name, &obj->A_BI, obj->x_dot, GraphType::Desired);
            pthread_mutex_unlock(&this->mtx);

            sendMsg(msg);
          }
        }
      }

      // desired graph update
      for (auto& obj : trackedBodies_always_des)
      {
        if (obj != NULL)
        {
          pthread_mutex_lock(&this->mtx);
          auto msg = updatePoseMsg(obj->name, &obj->A_BI, obj->x_dot, GraphType::Desired);
          pthread_mutex_unlock(&this->mtx);

          sendMsg(msg);
        }
      }

      // current graph update
      for (auto& obj : trackedBodies_always_curr)
      {
        if (obj != NULL)
        {
          pthread_mutex_lock(&this->mtx);
          auto msg = updatePoseMsg(obj->name, &obj->A_BI, obj->x_dot, GraphType::Current);
          pthread_mutex_unlock(&this->mtx);

          sendMsg(msg);
        }
      }

      //      const HTr* grasp_l = RcsGraph_getBodyByName(this->graphDes, "PowerGrasp_L")->A_BI;
      //      RLOG(0, "%5.2f, %5.2f, %5.2f", grasp_l->org[0], grasp_l->org[1], grasp_l->org[2]);
      //
      if (this->currPrompt != PromptType::None)
      {
        this->prompt_t -= dt;
        if (this->prompt_t <= 0)
        {
          RLOG(0, "Giving prompt: %s", this->prompts[this->currPrompt][this->prompt_idx].c_str());

          if (prompt_targets.size() > 0)
          {
            setAttention(this->prompt_targets[this->prompt_idx], 0.5);  // reset attention of last target
          }

          perform(this->prompts[this->currPrompt][this->prompt_idx]);
          prompt_idx = (prompt_idx + 1) % this->prompts[this->currPrompt].size();

          if (prompt_targets.size() > 0)
          {
            setAttention(this->prompt_targets[this->prompt_idx], 1.0);  // force gaze at next target
          }

          this->prompt_t = this->prompt_interval;
        }
      }
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / freq));
  }
}

void Rcs::RemoteVisualizationComponent::setDesiredBoxAngle(const double angle)
{
  RLOG(0, "Setting desired box angle %5.3f (%5.3f)", angle, RCS_RAD2DEG(angle));
  setProperty("box_des", angle);
}

void Rcs::RemoteVisualizationComponent::setDesiredHandPoses(double posX_r, double posY_r, double posZ_r,
                                                            double posX_l, double posY_l, double posZ_l)
{
  if (this->connected)
  {
    double orient_r[3][3];
    Mat3d_setIdentity(orient_r);
    double orient_l[3][3];
    Mat3d_setIdentity(orient_l);

    double pos_r[3];
    pos_r[0] = posX_r;
    pos_r[1] = posY_r;
    pos_r[2] = posZ_r;
    double pos_l[3];
    pos_l[0] = posX_l;
    pos_l[1] = posY_l;
    pos_l[2] = posZ_l;


    RLOG(2, "Pos r: %5.2f, %5.2f, %5.2f", pos_r[0], pos_r[1], pos_r[2]);
    RLOG(2, "Pos l: %5.2f, %5.2f, %5.2f", pos_l[0], pos_l[1], pos_l[2]);
    sendMsg(updatePoseMsg("PowerGrasp_Des_R", pos_r, orient_r, GraphType::Desired)); /// TODO: Check this
    sendMsg(updatePoseMsg("PowerGrasp_Des_L", pos_l, orient_l, GraphType::Desired));

    REXEC(2)
    {
      pthread_mutex_lock(&this->mtx);
      const HTr* grasp_r = &RcsGraph_getBodyByName(this->graphDes, "PowerGrasp_R")->A_BI;
      RLOG(2, "Cur r: %5.2f, %5.2f, %5.2f", grasp_r->org[0], grasp_r->org[1], grasp_r->org[2]);
      const HTr* grasp_l = &RcsGraph_getBodyByName(this->graphDes, "PowerGrasp_L")->A_BI;
      RLOG(2, "Cur l: %5.2f, %5.2f, %5.2f", grasp_l->org[0], grasp_l->org[1], grasp_l->org[2]);
      pthread_mutex_unlock(&this->mtx);
    }
  }
}

void Rcs::RemoteVisualizationComponent::onNewObjectModel(std::vector<HTr> grasps, std::vector<HTr> partnerGrasps)
{
  RLOG(0, "Got new model!");
  if (this->connected)
  {
    sendMsg(objectModelMsg(grasps));
  }
}

void Rcs::RemoteVisualizationComponent::display_update(std::string displayType)
{
  RLOG(0, "New display type: %s", displayType.c_str());
  setProperty("display_type", displayType);
}

void Rcs::RemoteVisualizationComponent::prompt_LeanRight()
{
  perform("lean_full_r.clamp");
}

void Rcs::RemoteVisualizationComponent::prompt_LeanLeft()
{
  perform("lean_full_l.clamp");
}

void Rcs::RemoteVisualizationComponent::prompt_Clear()
{
  perform("idle.looped");
}

void Rcs::RemoteVisualizationComponent::prompt_Confirm()
{
  perform("nod_quick");
}

void Rcs::RemoteVisualizationComponent::setAttention(const std::string& name, const double attention)
{
  if (this->connected)
  {
    sendMsg(setAttentionMsg(name, attention));
  }
}

void Rcs::RemoteVisualizationComponent::perform(const std::string& action)
{
  if (this->connected)
  {
    sendMsg(performActionMsg(action));
  }
}

void Rcs::RemoteVisualizationComponent::prompt(const PromptType& p, std::initializer_list<std::string> gazeTargets)
{
  if (this->currPrompt != p && this->currPrompt != PromptType::None)
  {
    this->clearPrompt();
  }

  this->currPrompt = p;

  for (auto t : gazeTargets)
  {
    this->prompt_targets.push_back(t);
  }
}

void Rcs::RemoteVisualizationComponent::clearPrompt()
{
  this->prompt_idx = 0;
  this->prompt_t = this->prompt_interval;
  this->currPrompt = PromptType::None;

  if (this->prompt_targets.size() > this->prompt_idx)
  {
    setAttention(this->prompt_targets[this->prompt_idx], 0.5); // reset attention of last target --
    // TODO: Don't couple prompt_idx to prompt_targets.size() ...
  }

  this->prompt_targets.clear();
}

void Rcs::RemoteVisualizationComponent::sendMsg(const std::string& msg)
{
  this->server.send(this->hdl, msg, websocketpp::frame::opcode::TEXT);
}

void Rcs::RemoteVisualizationComponent::sendMsg(const char* msg, const size_t len)
{
  this->server.send(this->hdl, msg, len, websocketpp::frame::opcode::TEXT);
}

std::string Rcs::RemoteVisualizationComponent ::updatePoseMsg(const std::string objName, const double position[3],
                                                              double rotation[3][3], GraphType g)
{
  double rot[4] = {1, 0, 0, 0};
  double euler[3] = {0, 0, 0};
  Quat_fromRotationMatrix(rot, rotation);
  Mat3d_toEulerAngles(euler, rotation);

  if (g == GraphType::Desired)
  {
    if (base_orientations_des.find(objName) != base_orientations_des.end())
    {
      euler[0] -= base_orientations_des[objName][0];
      euler[1] -= base_orientations_des[objName][1];
      euler[2] -= base_orientations_des[objName][2];
    }
  }
  else
  {
    if (base_orientations_cur.find(objName) != base_orientations_cur.end())
    {
      euler[0] -= base_orientations_cur[objName][0];
      euler[1] -= base_orientations_cur[objName][1];
      euler[2] -= base_orientations_cur[objName][2];
    }
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);

  writer.StartObject();
  writer.Key("op");
  writer.String("UpdatePose");
  writer.Key("stamp");
  writer.Double((double)msgStamp);
  msgStamp++;
  writer.Key("name");
  writer.String(objName);
  writer.Key("graphtype");
  writer.String(GraphTypeStr(g));

  writer.Key("pos");
  writer.StartObject();
  writer.Key("x");
  writer.Double(position[0]);
  writer.Key("y");
  writer.Double(position[1]);
  writer.Key("z");
  writer.Double(position[2]);
  writer.EndObject();

  writer.Key("rot");
  writer.StartObject();
  writer.Key("x");
  writer.Double(rot[1]);
  writer.Key("y");
  writer.Double(rot[2]);
  writer.Key("z");
  writer.Double(rot[3]);
  writer.Key("w");
  writer.Double(rot[0]);
  writer.EndObject();

  writer.Key("euler");
  writer.StartObject();
  writer.Key("x");
  writer.Double(euler[0]);
  writer.Key("y");
  writer.Double(euler[1]);
  writer.Key("z");
  writer.Double(euler[2]);
  writer.EndObject();

  writer.Key("vel");
  writer.StartArray();
  writer.Double(0);
  writer.Double(0);
  writer.Double(0);
  writer.EndArray();

  writer.EndObject();
  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent ::updatePoseMsg(const std::string objName, HTr* transform, GraphType g)
{
  double rot[4] = {1, 0, 0, 0};
  Quat_fromRotationMatrix(rot, transform->rot);
  double euler[3] = {0, 0, 0};
  double trans[6];
  HTr_to6DVector(trans, transform);
  Vec3d_copy(euler, &trans[3]);

  if (objName == "Box_v")
  {
    RLOG(0, "%s Box base rotation: %f, %f, %f", GraphTypeStr(g).c_str(), euler[0], euler[1], euler[2]);
  }

  if (g == GraphType::Desired)
  {
    if (base_orientations_des.find(objName) != base_orientations_des.end())
    {
      euler[0] -= base_orientations_des[objName][0];
      euler[1] -= base_orientations_des[objName][1];
      euler[2] -= base_orientations_des[objName][2];
    }
  }
  else
  {
    if (base_orientations_cur.find(objName) != base_orientations_cur.end())
    {
      euler[0] -= base_orientations_cur[objName][0];
      euler[1] -= base_orientations_cur[objName][1];
      euler[2] -= base_orientations_cur[objName][2];
    }
  }

  if (objName == "Box_v")
  {
    RLOG(0, "%s Box corr rotation: %f, %f, %f", GraphTypeStr(g).c_str(), euler[0], euler[1], euler[2]);
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);

  writer.StartObject();
  writer.Key("op");
  writer.String("UpdatePose");
  writer.Key("stamp");
  writer.Double((double)msgStamp);
  msgStamp++;
  writer.Key("name");
  writer.String(objName);
  writer.Key("graphtype");
  writer.String(GraphTypeStr(g));

  writer.Key("pos");
  writer.StartObject();
  writer.Key("x");
  writer.Double(transform->org[0]);
  writer.Key("y");
  writer.Double(transform->org[1]);
  writer.Key("z");
  writer.Double(transform->org[2]);
  writer.EndObject();

  writer.Key("rot");
  writer.StartObject();
  writer.Key("x");
  writer.Double(rot[1]);
  writer.Key("y");
  writer.Double(rot[2]);
  writer.Key("z");
  writer.Double(rot[3]);
  writer.Key("w");
  writer.Double(rot[0]);
  writer.EndObject();

  writer.Key("euler");
  writer.StartObject();
  writer.Key("x");
  writer.Double(euler[0]);
  writer.Key("y");
  writer.Double(euler[1]);
  writer.Key("z");
  writer.Double(euler[2]);
  writer.EndObject();

  writer.Key("vel");
  writer.StartArray();
  writer.Double(0);
  writer.Double(0);
  writer.Double(0);
  writer.EndArray();

  writer.EndObject();
  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::updatePoseMsg(const std::string objName, HTr* transform,
                                                             const double velocity[3], GraphType g)
{
  double rot[4] = {1, 0, 0, 0};
  Quat_fromRotationMatrix(rot, transform->rot);
  double euler[3] = {0, 0, 0};
  double trans[6];
  HTr_to6DVector(trans, transform);
  Vec3d_copy(euler, &trans[3]);

  //  if (objName == "Box_v")
  //  {
  //    RLOG(0, "%s Box base rotation: %f, %f, %f", GraphTypeStr(g).c_str(), euler[0], euler[1], euler[2]);
  //  }

  if (g == GraphType::Desired)
  {
    if (base_orientations_des.find(objName) != base_orientations_des.end())
    {
      euler[0] -= base_orientations_des[objName][0];
      euler[1] -= base_orientations_des[objName][1];
      euler[2] -= base_orientations_des[objName][2];
    }
  }
  else
  {
    if (base_orientations_cur.find(objName) != base_orientations_cur.end())
    {
      euler[0] -= base_orientations_cur[objName][0];
      euler[1] -= base_orientations_cur[objName][1];
      euler[2] -= base_orientations_cur[objName][2];
    }
  }

  //  if (objName == "Box_v")
  //  {
  //    RLOG(0, "%s Box corr rotation: %f, %f, %f", GraphTypeStr(g).c_str(), euler[0], euler[1], euler[2]);
  //  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);

  writer.StartObject();
  writer.Key("op");
  writer.String("UpdatePose");
  writer.Key("stamp");
  writer.Double((double)msgStamp);
  msgStamp++;
  writer.Key("name");
  writer.String(objName);
  writer.Key("graphtype");
  writer.String(GraphTypeStr(g));

  writer.Key("pos");
  writer.StartObject();
  writer.Key("x");
  writer.Double(transform->org[0]);
  writer.Key("y");
  writer.Double(transform->org[1]);
  writer.Key("z");
  writer.Double(transform->org[2]);
  writer.EndObject();

  writer.Key("rot");
  writer.StartObject();
  writer.Key("x");
  writer.Double(rot[1]);
  writer.Key("y");
  writer.Double(rot[2]);
  writer.Key("z");
  writer.Double(rot[3]);
  writer.Key("w");
  writer.Double(rot[0]);
  writer.EndObject();

  writer.Key("euler");
  writer.StartObject();
  writer.Key("x");
  writer.Double(euler[0]);
  writer.Key("y");
  writer.Double(euler[1]);
  writer.Key("z");
  writer.Double(euler[2]);
  writer.EndObject();

  writer.Key("vel");
  writer.StartObject();
  writer.Key("x");
  writer.Double(velocity[0]);
  writer.Key("y");
  writer.Double(velocity[1]);
  writer.Key("z");
  writer.Double(velocity[2]);
  writer.EndObject();

  writer.EndObject();
  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::updatePoseMsg(const std::string objName, const double position[3],
                                                             double rotation[3][3], const double velocity[3],
                                                             GraphType g)
{
  double rot[4] = {1, 0, 0, 0};
  double euler[3] = {0, 0, 0};
  Quat_fromRotationMatrix(rot, rotation);
  Mat3d_toEulerAngles(euler, rotation);

  if (g == GraphType::Desired)
  {
    if (base_orientations_des.find(objName) != base_orientations_des.end())
    {
      euler[0] -= base_orientations_des[objName][0];
      euler[1] -= base_orientations_des[objName][1];
      euler[2] -= base_orientations_des[objName][2];
    }
  }
  else
  {
    if (base_orientations_cur.find(objName) != base_orientations_cur.end())
    {
      euler[0] -= base_orientations_cur[objName][0];
      euler[1] -= base_orientations_cur[objName][1];
      euler[2] -= base_orientations_cur[objName][2];
    }
  }

  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);

  writer.StartObject();
  writer.Key("op");
  writer.String("UpdatePose");
  writer.Key("stamp");
  writer.Double((double)msgStamp);
  msgStamp++;
  writer.Key("name");
  writer.String(objName);
  writer.Key("graphtype");
  writer.String(GraphTypeStr(g));

  writer.Key("pos");
  writer.StartObject();
  writer.Key("x");
  writer.Double(position[0]);
  writer.Key("y");
  writer.Double(position[1]);
  writer.Key("z");
  writer.Double(position[2]);
  writer.EndObject();

  writer.Key("rot");
  writer.StartObject();
  writer.Key("x");
  writer.Double(rot[1]);
  writer.Key("y");
  writer.Double(rot[2]);
  writer.Key("z");
  writer.Double(rot[3]);
  writer.Key("w");
  writer.Double(rot[0]);
  writer.EndObject();

  writer.Key("euler");
  writer.StartObject();
  writer.Key("x");
  writer.Double(euler[0]);
  writer.Key("y");
  writer.Double(euler[1]);
  writer.Key("z");
  writer.Double(euler[2]);
  writer.EndObject();

  writer.Key("vel");
  writer.StartObject();
  writer.Key("x");
  writer.Double(velocity[0]);
  writer.Key("y");
  writer.Double(velocity[1]);
  writer.Key("z");
  writer.Double(velocity[2]);
  writer.EndObject();

  writer.EndObject();
  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::setAttentionMsg(const std::string objName, const double attention)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "SetAttention", d.GetAllocator());
  d.AddMember("name", objName, d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("attention", attention, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::setAttentionMsg(const double position[3], const double attention)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "SetAttention", d.GetAllocator());

  rapidjson::Value pos(rapidjson::kArrayType);
  pos.PushBack(position[0], d.GetAllocator()).PushBack(position[1], d.GetAllocator()).PushBack(position[2],
      d.GetAllocator());
  d.AddMember("location", pos, d.GetAllocator());

  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("attention", attention, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::setEmergencyMsg(const bool emergencyActive, const std::string reason)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "SetEmergencyState", d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("emergency", emergencyActive, d.GetAllocator());
  d.AddMember("reason", reason, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::performActionMsg(const std::string& action)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "PerformAction", d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("action", action, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::setDesiredBoxAngleMsg(const double angle)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "UpdateDesiredBoxAngle", d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("angle", angle, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::setDisplayMsg(const std::string display)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buf);
  writer.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();
  d.AddMember("op", "SetDisplayType", d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());
  d.AddMember("displaytype", display, d.GetAllocator());
  d.Accept(writer);

  return buf.GetString();
}

std::string Rcs::RemoteVisualizationComponent::objectModelMsg(std::vector<HTr> grasps)
{
  rapidjson::StringBuffer buf;
  rapidjson::Writer<rapidjson::StringBuffer> w(buf);
  w.SetMaxDecimalPlaces(max_decimals);
  rapidjson::Document d;
  d.SetObject();

  d.AddMember("op", "SetObjectModel", d.GetAllocator());
  d.AddMember("stamp", msgStamp++, d.GetAllocator());

  // perimiter of object
  rapidjson::Value vertices(rapidjson::kArrayType);
  for (size_t i = 0; i < 4; i++)
  {
    rapidjson::Value v(rapidjson::kObjectType);
    v.AddMember("x", i, d.GetAllocator());
    v.AddMember("y", i-1, d.GetAllocator());
    vertices.PushBack(v, d.GetAllocator());
  }
  d.AddMember("perimiter", vertices, d.GetAllocator());

  // grasp positions
  rapidjson::Value grasp_points(rapidjson::kArrayType);
  for (auto& grasp : grasps)
  {
    rapidjson::Value g(rapidjson::kObjectType);
    g.AddMember("x", grasp.org[0], d.GetAllocator());
    g.AddMember("y", grasp.org[1], d.GetAllocator());
    g.AddMember("z", grasp.org[2], d.GetAllocator());
    grasp_points.PushBack(g, d.GetAllocator());
  }
  d.AddMember("grasps", grasp_points, d.GetAllocator());

  // grasp orientations
  rapidjson::Value grasp_normals(rapidjson::kArrayType);
  for (auto& grasp : grasps)
  {
    double euler[3];

    Mat3d_toEulerAngles(euler, grasp.rot);
    grasp_normals.PushBack(euler[2], d.GetAllocator()); // orientation is only about the Z-axis
  }
  d.AddMember("grasp_normals", grasp_normals, d.GetAllocator());

  d.Accept(w);
  return buf.GetString();
}

void Rcs::RemoteVisualizationComponent::start()
{
  if (!started)
  {
    started = true;
    bgThread = std::thread(&RemoteVisualizationComponent::runThread, this, 12);
  }
}

void Rcs::RemoteVisualizationComponent::stop()
{
  if (connected)
  {
    this->connected = false;
    this->server.stop_listening();
    this->server.close(this->hdl, websocketpp::close::status::going_away, "Robot shut down");

    // must process the stop events submitted above
    this->server.poll();
  }

  if (started)
  {
    started = false;
    bgThread.join();
  }
}
