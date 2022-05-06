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
#include "HoloComponent.h"
#include "HoloParser.h"

#include <Rcs_joint.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_VecNd.h>
#include <Rcs_timer.h>
#include <Rcs_utilsCPP.h>

#define DBG_AXES 0

// DON'T CREATE MORE THAN ONE INSTANCE OF THIS COMPONENT! IT REQUIRES A
// NETWORKCONNECTION AND BINDS TO A SPECIFIC PORT

namespace Rcs
{

GraphType operator|(const GraphType& lhs, const GraphType& rhs)
{
  return static_cast<GraphType>(
           static_cast<std::underlying_type<GraphType>::type>(lhs) |
           static_cast<std::underlying_type<GraphType>::type>(rhs));
}
GraphType& operator|=(GraphType& lhs, const GraphType& rhs)
{
  lhs = lhs | rhs;
  return lhs;
}
GraphType operator&(const GraphType& lhs, const GraphType& rhs)
{
  return static_cast<GraphType>(
           static_cast<std::underlying_type<GraphType>::type>(lhs) &
           static_cast<std::underlying_type<GraphType>::type>(rhs));
}
GraphType& operator&=(GraphType& lhs, const GraphType& rhs)
{
  lhs = lhs & rhs;
  return lhs;
}
GraphType operator^(const GraphType& lhs, const GraphType& rhs)
{
  return static_cast<GraphType>(
           static_cast<std::underlying_type<GraphType>::type>(lhs) ^
           static_cast<std::underlying_type<GraphType>::type>(rhs));
}
GraphType& operator^=(GraphType& lhs, const GraphType& rhs)
{
  lhs = lhs ^ rhs;
  return lhs;
}
GraphType operator~(const GraphType& g)
{
  return static_cast<GraphType>(~static_cast<std::underlying_type<GraphType>::type>(g));
}

std::string HoloComponent::DIRECT_TRANSFORM()
{
  return std::string("<FRONTFACE>");
}

HoloComponent::HoloComponent(EntityBase* parent,
                             std::string rootObj,
                             unsigned int port_broadcast,
                             unsigned int port_receive) :
  Rcs::ComponentBase(parent),
  PeriodicCallback(),
  rootName(rootObj),
  broadcastPort(port_broadcast),
  receivePort(port_receive),
  framesSynced(false)
{
  setClassName("HoloComponent");
  HTr_setIdentity(&this->slamFrame);
  HTr_setIdentity(&this->lastSyncPoint);

  // Starts this component's callback loop and creates the necessary network connections
  // to communicate with any Holographic devices on the network.
  getEntity()->subscribe("Start", &HoloComponent::onStart, this);

  // Stops this component's callback loop and shuts down any active network connections.
  getEntity()->subscribe("Stop", &HoloComponent::onStop, this);

  // Only updates the graph with the self-reported poses of holographic devices so they
  // can be tracked and visualized.
  getEntity()->subscribe("UpdateGraph", &HoloComponent::updateGraph, this);

  // Retrieves the poses of any tracked transforms and broadcasts them over the network.
  getEntity()->subscribe("PostUpdateGraph", &HoloComponent::postUpdateGraph, this);

  // Considers the last known pose of a hololens as the reference point all transforms should be sent as relative to.
  getEntity()->subscribe("HoloSyncViconFrame", &HoloComponent::syncViconFrame, this);

  // Tells HoloComponent to continuously update and broadcast the pose of the specified body in the specified graph.
  // <b>name:</b> Name of body to track
  // <b>graphType</b>: Graph(s) to retrieve body transform from (Current, Desired, or both)
  getEntity()->subscribe("HoloTrackTransform", [this](std::string name, GraphType graphType)
  {
    this->onTrackTransform(name, graphType);
  });

  // Stops tracking of a given transform from one or more graphs. If a transform was previously registered for tracking
  // in two or more GraphTypes, it can be selectively removed from one without affecting the others.
  // <b>name:</b> Name of body to untrack
  // <b>graphType</b>: Graph(s) to stop tracking the body in
  getEntity()->subscribe("HoloUntrackTransform", [this](std::string name, GraphType graphType)
  {
    this->onUntrackTransform(name, graphType);
  });

  // Notifies holographic devices that they should hide or show their GUIs.
  // <b>enableUI</b>: True to show GUIs, False to hide them.
  getEntity()->subscribe("HoloToggleUI", [this](bool enableUI)
  {
    this->holoConn.sendMessage(toggleUIMsg(enableUI));
  });

  // Sends a raw string as-is to all clients.
  // Use at your own risk.
  getEntity()->subscribe("HoloSendRaw", [this](std::string data)
  {
    this->holoConn.sendMessage(data);
  });

  // Sends a Text-To-Speech message to all holographic devices to be played as soon as possible
  // <b>text</b>: The text to speak.
  getEntity()->subscribe("HoloTTS", [this](std::string text)
  {
    this->holoConn.sendMessage(TTSMsg(text));
  });

  // Sends a transform update message to all holographic devices.
  // <b>T</b>: The transform to send
  // <b>name</b>: The unique name of the transform
  // <b>parent</b>: The name of the transform's parent (transform T is relative to) or "All" if T is in the world frame.
  getEntity()->subscribe("HoloUpdateTransform", [this](HTr T, std::string name, std::string parent)
  {
    this->update(name, &T, GraphType::Other, parent.empty() ? "All" : parent);
  });

  // Sends a transform update message to all holographic devices, relative to a given transform (e.g. marker position).
  // <b>T</b>: The transform to send
  // <b>name</b>: The unique name of the transform
  // <b>parent</b>: The name of the transform's parent (transform T is relative to) or "All" if T is in the world frame.
  // <b>rel</b>: Transform T should be computed relative to before transmission
  getEntity()->subscribe("HoloUpdateRelTransform", [this](HTr T, std::string name, std::string parent, HTr rel)
  {
    this->update(name, &T, GraphType::Other, parent.empty() ? "All" : parent, &rel);
  });

  // Creates/Updates a transform which mirrors the position/orientation of another transform according to a
  // specified axis.
  // Mirroring will be relative to the local origin frame of the target transform.
  // <b>name</b>:         The unique name of the transform
  // <b>mirrorTarget</b>: The unique name of the transform to mirror
  // <b>axis</b>:         The axis to mirror across
  getEntity()->subscribe("HoloUpdateMirrorTransform",
                         [this](std::string name, std::string mirrorTarget, std::vector<double> axis)
  {
    double pos[3] = {0, 0, 0}; // placeholder position and orientation--they will be overridden by the mirroring
    double rot[4] = {0, 0, 0, 1};
    this->holoConn.sendMessage(updateTransformMsg(name, "All", pos, rot, mirrorTarget, axis.data()));

#if DBG_AXES
    Axes axes;
    axes.name = name + "___axes";
    axes.parent = name;
    axes.scale = 0.1;
    RLOG(0, "Updating axes: %s", axes.name.c_str());
    this->holoConn.send(axes);
    //update(axes, GraphType::Other);
#endif
  });

  // Removes holograms from holographic devices
  // <b>name</b>: Name of hologram to remove
  // <b>topic</b>: Topic the named hologram is associated with
  // If Name == "", then ALL holograms with the given <b>topic</b> will be removed.
  // If Name == "" && topic == "", then ALL holograms from ALL topics will be removed.
  getEntity()->subscribe("HoloClear", [this](std::string name, std::string topic)
  {
    this->clear(name, topic, GraphType::Other);
  });

  // Adds or updates an Axes hologram
  getEntity()->subscribe("HoloUpdateAxes", [this](Axes axes)
  {
    this->update(axes, GraphType::Other);
  });

  // Adds or updates a Box hologram
  getEntity()->subscribe("HoloUpdateBox", [this](Box box)
  {
    this->update(box, GraphType::Other);
  });

  // Adds or updates an invisible Box which occludes other holograms.
  // Used to cheaply fake occlusion against real-world objects which are tracked by other systems.
  getEntity()->subscribe("HoloUpdateBoxOccluder", [this](BoxOccluder box)
  {
    this->update(box, GraphType::Other);
  });

  // Adds or updates a Cylinder hologram.
  getEntity()->subscribe("HoloUpdateCylinder", [this](Cylinder cylinder)
  {
    this->update(cylinder, GraphType::Other);
  });

  // Adds or updates an invisible Cylinder which occludes other holograms.
  // Used to cheaply fake occlusion against real-world objects which are tracked by other systems.
  getEntity()->subscribe("HoloUpdateCylinderOccluder", [this](CylinderOccluder cylinder)
  {
    this->update(cylinder, GraphType::Other);
  });

  // Adds or updates a Sphere hologram
  getEntity()->subscribe("HoloUpdateSphere", [this](Sphere sphere)
  {
    this->update(sphere, GraphType::Other);
  });

  // Adds or updates an invisible Sphere which occludes other holograms.
  // Used to cheaply fake occlusion against real-world objects which are tracked by other systems.
  getEntity()->subscribe("HoloUpdateSphereOccluder", [this](SphereOccluder sphere)
  {
    this->update(sphere, GraphType::Other);
  });

  // Adds or updates a plain, solid, arrow which points from one transform to another.
  // This arrow will automatically update if either transform moves.
  getEntity()->subscribe("HoloUpdateArrowPlain", [this](ArrowPlain arrow)
  {
    this->update(arrow, GraphType::Other);
  });

  // Adds or updates a hollow arrow which can be filled like a progress bar. It points from one transform to another.
  // This arrow will automatically update if either transform moves.
  getEntity()->subscribe("HoloUpdateArrowProgress", [this](ArrowProgress arrow)
  {
    this->update(arrow, GraphType::Other);
  });

  // Adds or updates a solid arrow which points in a specified direction.
  getEntity()->subscribe("HoloUpdateArrowDirection", [this](ArrowDirection arrow)
  {
    this->update(arrow, GraphType::Other);
  });

  // Adds or updates a text label.
  // Text can be formatted according to the markup described here: https://docs.unity3d.com/Manual/StyledText.html
  getEntity()->subscribe("HoloUpdateLabel", [this](Label label)
  {
    this->update(label, GraphType::Other);
  });

  // Adds or updates a static image sprite which is billboarded to face the user.
  // Available sprites: Sprite
  getEntity()->subscribe("HoloUpdateSprite", [this](Sprite sprite)
  {
    this->update(sprite, GraphType::Other);
  });

  // Adds or updates a visual progress indicator, which is billboarded to face the user.
  // Progress values will be clamped to 0..1
  // Available sprites: progress/circle01
  getEntity()->subscribe("HoloUpdateProgressSprite", [this](ProgressSprite sprite)
  {
    this->update(sprite, GraphType::Other);
  });

  // Adds or updates a marker which drops a vertical line from its position to the floor.
  getEntity()->subscribe("HoloUpdateDropMarker", [this](DropMarker dropMarker)
  {
    this->update(dropMarker, GraphType::Other);
  });

  // Adds or updates a static 3D mesh.
  // <b>Available meshes</b>: 'SDH', 'Fox'
  getEntity()->subscribe("HoloUpdateStaticModel", [this](StaticModel staticModel)
  {
    this->update(staticModel, GraphType::Other);
  });

  // Adds or updates a poseable 3D object.
  // Available objects: SDH, KukaLBR
  getEntity()->subscribe("HoloUpdatePosedModel", [this](PosedModel posedModel)
  {
    this->update(posedModel, GraphType::Other);
  });

  // Adds or updates a dotted line trajectory
  getEntity()->subscribe("HoloUpdateTrajectoryPoints", [this](TrajectoryPoints trajectoryPoints)
  {
    this->update(trajectoryPoints, GraphType::Other);
  });

  // Adds or updates a trajectory of arrows.
  // Arrows will point in the direction of the next point in the trajectory.
  getEntity()->subscribe("HoloUpdateTrajectoryArrows", [this](TrajectoryArrows trajectoryArrows)
  {
    this->update(trajectoryArrows, GraphType::Other);
  });

  // Adds or updates a line-based trajectory
  getEntity()->subscribe("HoloUpdateTrajectoryLines", [this](TrajectoryLines trajectoryLines)
  {
    this->update(trajectoryLines, GraphType::Other);
  });

  // Adds or updates a flat 2D polygon.
  getEntity()->subscribe("HoloUpdatePoly2D", [this](Poly2D poly2D)
  {
    this->update(poly2D, GraphType::Other);
  });

  onTrackTransform("sdh-base_L", GraphType::Desired);
  onTrackTransform("sdh-base_R", GraphType::Desired);


}

HoloComponent::~HoloComponent()
{
  stop();
}

/*******************************************************************************
 * Updates the self-reported pose of the Hololens
 ******************************************************************************/
void HoloComponent::updateGraph(RcsGraph* graph)
{
  const RcsBody* slamFrameBdy = RcsGraph_getBodyByName(graph, "Hololens_SlamFrame");
  const RcsBody* markerBdy = RcsGraph_getBodyByName(graph, rootName.c_str());

  if (slamFrameBdy && markerBdy && RcsBody_isFloatingBase(graph, slamFrameBdy))
  {
    HTr tmp;
    HTr_transform(&tmp, &markerBdy->A_BI, &slamFrame);

    //const RcsJoint* slamFrameJnt = RCSJOINT_BY_ID(graph, slamFrameBdy->jntId);
    const RcsJoint* slamFrameJnt = &graph->joints[slamFrameBdy->jntId];
    double* slam_q = &graph->q->ele[slamFrameJnt->jointIndex];
    mtx.lock();
    HTr_to6DVector(slam_q, &tmp);
    mtx.unlock();
  }
  else
  {
    RLOG(1, "Body \"Hololens_SlamFrame\" has issues - either not existing, or "
         "no rigid body joints");
  }
}

/*******************************************************************************
 * Updates any tracked transforms in the desired and/or current graph
 ******************************************************************************/
void HoloComponent::postUpdateGraph(RcsGraph* graphDes, RcsGraph* graphCurr)
{

  const RcsBody* slamFrameBdy_c = RcsGraph_getBodyByName(graphCurr, "Hololens_SlamFrame");
  if (slamFrameBdy_c != NULL)
  {
    //const RcsBody* slamFrameBdy_d = RcsGraph_getBodyByName(graphDes, "Hololens_SlamFrame");
    int idx = graphCurr->joints[slamFrameBdy_c->jntId].jointIndex;
    //int idx = RCSJOINT_BY_ID(graphCurr, slamFrameBdy_c->jntId)->jointIndex;
    VecNd_copy(&graphDes->q->ele[idx], &graphCurr->q->ele[idx], 6);

    RcsBody* holoBdy = RcsGraph_getBodyByName(graphCurr, "Hololens_1");
    if (holoBdy != NULL)
    {
      HTr_copy(&holoBdy->A_BI, &lastSyncPoint); // save hololens frame
    }
  }

  static int count = 0;
  count++;
  if (count%5!=0)
  {
    return;
  }

  //  if (holoConn.getMessagesReceived() > 0)
  {
    /// TODO: more intelligently pick relative body, possibly allow for parent other than All?
    for (auto& t : trackedTransforms)
    {
      if ((t.second & GraphType::Current) != GraphType::None)
      {
        const RcsBody* tBdy = RcsGraph_getBodyByName(graphCurr, t.first.c_str());
        const RcsBody* rootBdy = RcsGraph_getBodyByName(graphCurr, rootName.c_str());

        if (tBdy && rootBdy)
        {
          update(t.first, &tBdy->A_BI, GraphType::Current,
                 (framesSynced || t.first == rootName ? "All" : rootName),
                 (framesSynced ? &lastSyncPoint : &rootBdy->A_BI));
        }
        else
        {
          if (tBdy==NULL)
          {
            RLOG_CPP(1, "Couldn't find tracked body " << t.first);
          }
          if (rootBdy==NULL)
          {
            RLOG_CPP(1, "Couldn't find root body " << rootName);
          }
        }
      }

      if ((t.second & GraphType::Desired) != GraphType::None)
      {
        const RcsBody* tBdy = RcsGraph_getBodyByName(graphDes, t.first.c_str());
        const RcsBody* rootBdy = RcsGraph_getBodyByName(graphDes, rootName.c_str());

        if (tBdy && rootBdy)
        {
          update(t.first, &tBdy->A_BI, GraphType::Desired,
                 (framesSynced || t.first == rootName ? "All" : rootName),
                 (framesSynced ? &lastSyncPoint : &rootBdy->A_BI));
        }
        else
        {
          if (tBdy==NULL)
          {
            RLOG_CPP(1, "Couldn't find tracked body " << t.first);
          }
          if (rootBdy==NULL)
          {
            RLOG_CPP(1, "Couldn't find root body " << rootName);
          }
        }
      }

#if DBG_AXES
      Axes axes;
      axes.name = t.first + "_axes";
      axes.parent = t.first;
      axes.scale = 0.1;
      update(axes, t.second);
#endif
    }
  }

  //  PosedModel sdh_R("SDH_R_NOW", std::string("sdh-base_R"), {250, 194, 52});
  //  sdh_R.setPose("SDH", graphDes->q, RcsGraph_getJointByName(graphDes, "knuck3-base_R")->jointIndex, 8);
  //  update(sdh_R, GraphType::Desired);
  //  PosedModel sdh_L("SDH_L_NOW", std::string("sdh-base_L"), {250, 194, 52});
  //  sdh_L.setPose("SDH", graphDes->q, RcsGraph_getJointByName(graphDes, "knuck3-base_L")->jointIndex, 8);
  //  update(sdh_L, GraphType::Desired);
}

void HoloComponent::onTrackTransform(std::string name, GraphType graphType, std::string parent)
{
  auto transform = std::find_if(trackedTransforms.begin(), trackedTransforms.end(),
                                [&name](const std::pair<std::string, GraphType>& ele)
  {
    return ele.first == name;
  });

  if (transform != trackedTransforms.end())
  {
    transform->second |= graphType;
  }
  else
  {
    trackedTransforms.emplace_back(std::make_pair(name, graphType));

    update(name, HTr_identity(), graphType, parent); // (parent == "All" ? rootName : parent));

#if DBG_AXES
    // if the object is new, add a placeholder transform and body for it
    Axes axes;
    axes.name = name + "_axes";
    axes.parent = name;
    axes.scale = 0.1;
    update(axes, graphType);
#endif
  }
}

void HoloComponent::onUntrackTransform(std::string name, GraphType graphType)
{
  auto transform = std::find_if(trackedTransforms.begin(), trackedTransforms.end(),
                                [&name](const std::pair<std::string, GraphType>& ele)
  {
    return ele.first == name;
  });

  if (transform != trackedTransforms.end())
  {
    transform->second &= ~graphType;
  }
}

/******************************************************************************************************
 * This method gets called once the "Start" event has been published (see constructor).
 * Creates the necessary network connections for communication between holographic devices and starts
 * the component's PeriodicCallback.
 ****************************************************************************************************/
void HoloComponent::onStart()
{
  setUpdateFrequency(100.0);
  PeriodicCallback::start();

  RLOG(0, "Starting HoloNetworkConnection");
  bool success = holoConn.startServer(this->broadcastPort);
  if (!success)
  {
    RWARNING(0, "FAILED to start HoloNetworkConnection Server!");
  }
  success = holoConn.startClient(this->broadcastPort+1);
  if (!success)
  {
    RWARNING(0, "FAILED to start HOloNetworkConnection Client!");
  }
}

/******************************************************************************************************
 * Stops PeriodicCallback from running and shuts own any active network connections.
 ****************************************************************************************************/
void HoloComponent::onStop()
{
  stop();
  holoConn.stopServer();
}

/******************************************************************************************************
 * Captures the last known position of the hololens as a new reference point for transforms.
 ****************************************************************************************************/
void HoloComponent::syncViconFrame()
{
  HTr_copy(&lastSyncPoint, &slamFrame);

  framesSynced = true;
}

void HoloComponent::update(Hologram& hologram, const GraphType& graphType)
{
  std::string parent = hologram.parent;
  if ((graphType & GraphType::Desired) != GraphType::None)
  {
    //hologram.parent = addGraphSuffix(parent, graphType);
    holoConn.send(hologram);
  }
  if ((graphType & GraphType::Current) != GraphType::None)
  {
    //hologram.parent = addGraphSuffix(parent, graphType);
    holoConn.send(hologram);
  }
  // just send as-is
  if ((graphType & GraphType::Other) != GraphType::None)
  {
    holoConn.send(hologram);
  }
  hologram.parent = parent;
}

void HoloComponent::update(const std::string& name, const HTr* transform, const GraphType& graphType,
                           const std::string& parent, const HTr* relTransform)
{
  HTr* t = HTr_clone(HTr_identity());
  if (!relTransform)
  {
    t = HTr_clone(transform);
  }
  else
  {
    HTr_invTransform(t, relTransform, transform);
  }

  if ((graphType & GraphType::Desired) != GraphType::None)
  {
    //holoConn.send(addGraphSuffix(name, graphType), parent == "All" ? parent : addGraphSuffix(parent, graphType), t);
    holoConn.send(name, parent, t);
  }
  if ((graphType & GraphType::Current) != GraphType::None)
  {
    //holoConn.send(addGraphSuffix(name, graphType), parent == "All" ? parent : addGraphSuffix(parent, graphType), t);
    holoConn.send(name, parent, t);
  }
  if ((graphType & GraphType::Other) != GraphType::None)
  {
    holoConn.send(name, parent, t);
  }
}

/// TODO: Handle empty names
void HoloComponent::clear(const std::string& name, const std::string& topic, const GraphType& graphType)
{
  if ((graphType & GraphType::Desired) != GraphType::None)
  {
    holoConn.sendMessage(clearHologramMsg(name, topic));
    //holoConn.sendMessage(clearHologramMsg(addGraphSuffix(name, graphType), topic));
  }
  if ((graphType & GraphType::Current) != GraphType::None)
  {
    holoConn.sendMessage(clearHologramMsg(name, topic));
    //holoConn.sendMessage(clearHologramMsg(addGraphSuffix(name, graphType), topic));
  }
  if ((graphType & GraphType::Other) != GraphType::None)
  {
    holoConn.sendMessage(clearHologramMsg(name, topic));
  }
}

bool HoloComponent::parseHoloMessage(std::string message)
{
  std::string debugMsg = message;

  REXEC(1)
  {
    RMSG("Parsing \"%s\"", message.c_str());
  }

  std::vector<std::string> split = String_split(message, SEP_Block);
  if (split.empty())
  {
    RLOG(0, "Received unsupported message: \"%s\"", debugMsg.c_str());
    return false;
  }

  REXEC(1)
  {
    for (size_t i=0; i<split.size(); ++i)
    {
      RMSG("substring[%d] = \"%s\"", (int) i, split[i].c_str());
    }
  }


  if (split[0]=="SLAMDeviceTransform")
  {
    if (split.size()!=3)
    {
      RLOG_CPP(0, "Expected 'SLAMDeviceTransform' message to contain 3 blocks, " << split.size() << " found instead");
      return false;
    }
    std::string deviceId = split[1].c_str();

    if (deviceId.find("WORKBOY") == std::string::npos) // discard desktop PC's poses
    {
      split = String_split(split[2], SEP_Parameter);
      RCHECK(split.size() == 2);

      // split[0] contains position, split[1] contains quaternion
      double pos[3], quat[4];
      parseVector(split[0], pos, 3);
      parseQuaternion(split[1], quat);
      //    RMSG("Quaternion: %f %f %f %f", quat[0], quat[1], quat[2], quat[3]);

      mtx.lock();
      Quat_toRotationMatrix(slamFrame.rot, quat);
      Vec3d_copy(slamFrame.org, pos);
      mtx.unlock();
    }
  }
  else if (split[0] == "SLAMDeviceInput")
  {
    if (split.size() < 2)
    {
      RLOG(0, "Received \"SLAMDeviceInput\" with number of substrings < 2");
      return false;
    }

    std::string deviceId = split[1];

    switch (split.size())
    {
      case 2:   // Tap
        if (message == "Tap")
        {
          RMSG("Detected TAP on device %s", deviceId.c_str());
        }
        break;

      case 3:   // SpeechCommand
        if (split[2] == "SpeechCommand")
        {
          RMSG("Detected SpeechCommand on device %s: \"%s\"", deviceId.c_str(), message.c_str());
        }
        break;

      default:
        RLOG(0, "Unhandled message on device %s: \"%s\"", deviceId.c_str(), debugMsg.c_str());
        return false;
    }
  }
  else if (split[0] == "SLAMDeviceViconTracking" && split[2] == "True")
  {
    syncViconFrame();
  }
  else if (split[0] == "Hello")
  {
    RMSG("Device %s is listening/available", split[1].c_str());
  }
  else if (split[0] == "Goodbye")
  {
    RMSG("Device %s is no longer listening/available", split[1].c_str());
  }
  else
  {
    RLOG(0, "Unhandled message: \"%s\" \"%s\"", debugMsg.c_str(), split[0].c_str());
    return false;
  }

  return true;
}

// This callback function gets called after the start() method has been called.
// It is repeated with the given update frequency. In such a thread, the
// communication with the HoloLens could for instance be made.
void HoloComponent::callback()
{
  char text[256];
  snprintf(text, 256, "Messages sent: %zd   freq: %.1f  send queue: %zd",
           holoConn.getMessagesSent(),
           holoConn.getDesiredSendFrequency(),
           holoConn.getSendQueueSize());

  std::string last_msg = holoConn.getMessage();
  if (last_msg.size() > 0)
  {
    getEntity()->publish("SetTextLine", last_msg, 1);
    parseHoloMessage(last_msg);
  }

}

std::string HoloComponent::addGraphSuffix(const std::string& name, const GraphType& graph)
{
  if ((graph & GraphType::Desired) != GraphType::None)
  {
    return name + "(des)";
  }
  if ((graph & GraphType::Current) != GraphType::None)
  {
    return name + "(cur)";
  }
  return name;
}

}   // namespace Rcs
