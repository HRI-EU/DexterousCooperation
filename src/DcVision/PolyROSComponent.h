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

#ifndef RCS_POLYROSCOMPONENT_H
#define RCS_POLYROSCOMPONENT_H

#include "PolyGraspDetector.h"

#include <ComponentBase.h>
#include <Rcs_graph.h>

#if defined (USE_DC_ROS)
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#else
#include <memory>
namespace geometry_msgs
{
struct Point32
{
  float x, y, z;
};

struct Polygon
{
  std::vector<Point32> points;
};

struct Header
{
  std::string frame_id;
};

struct PolygonStamped
{
  typedef std::shared_ptr<PolygonStamped> ConstPtr;
  Header header;
  Polygon polygon;
};

}

#endif

#include <mutex>
#include <memory>



namespace Rcs
{

/*! \brief Class to receive polygonial 2d shapes. This class can operate in two
 *         modes:
 *
 *         In the debug mode (constructor argument polyDebug is true), the
 *         class subscribes to an ESlib event "ReloadPolygon". This can be
 *         published with a filename, for instance through the EventGui. No ROS
 *         connection is established in this mode. The class runs without any
 *         roscore running somewhere else.
 *
 *         In the normal mode (constructor argument polyDebug is false), the
 *         class subscribes to a ROS topic passed into the constructor. The
 *         class therefore expects a roscore running somewhere, with the
 *         ROS_MASTER_URI set appropriately. In this mode, the following events
 *         are subscribed:
 *         - "Start": Subscribes to ROS (once per instance only)
 *         - "Stop": Shuts down the ROS subscriber (if it was started)
 *         - "GetPolygonsFromROS": Is called with true for processing the ROS
 *            polygon message, false otherwise. This way, the class can
 *            freeze the last polygon, for instance when it is passed to the
 *            planner.
 *         The polygon message is assumed to contain the 3d polygon points in
 *         camera coordinates. The name of the camera body is read from the
 *         message header's frame_id name. If no corresponding body is found in
 *         the graph, no polygon processing will happen.
 *
 *         In both modes, the class subscribes to the "PostUpdateGraph" event
 *         in which the transforms of the reference motion (camera and object)
 *         are updated.
 *
 *         The class publishes the following events:
 *         - RemoveNode, AddNode, AddChildNode: Shows and deletes some
 *           OpenSceneGraph nodes to be shown in the graphics window.
 *         - SetPolygon: Published a PolygonObjectModel that contains the
 *           polygon data required for planning.
 *
 *         The class runs concurrently threads for the ROS subscriber and the
 *         ESlib event queue. All concurrent accesses have been made thread-
 *         safe.
 *
 */
class PolyROSComponent : public ComponentBase
{
public:

  /*! \brief Constructs the class and subscribes to all events.
   *
   * \param[in] parent     Entity class responsible for event subscriptions
   * \param[in] rosTopic   Name of the ROS topic the class reads the
   *                       polygon data from
   * \param[in] polyDebug  True for test mode without ROS, false for connecting
   *                       to ROS. See above for more explanations.
   */
  PolyROSComponent(EntityBase* parent, std::string rosTopic, bool polyDebug);

  /*! \brief Destroys the members with allocated memory.
   */
  virtual ~PolyROSComponent();

  /*! \brief Assignes the camName to the camera bodie's name. This is used
   *         to look up the camera from the graph in order to perform the
   *         transformations from camera to other coordinates. The camera
   *         name is updated through the ROS callback. Using this function
   *         is only necessary when for instance using a virtual Kinect.
   */
  void setCameraBodyName(const std::string& camName);

private:

  // Event callbacks
  void onStart();
  void onStop();
  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current);
  void onSetNewData(std::string fileName);
  void onListenToROS(bool enable);
  void onToggleViewFromKinect();

  // "Normal" functions
  geometry_msgs::PolygonStamped::ConstPtr polyFromFile(std::string file) const;
  void rosCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  bool updatePolygon(const geometry_msgs::PolygonStamped::ConstPtr& msg);
  MatNd* computePolyFrame(const geometry_msgs::PolygonStamped::ConstPtr& msg,
                          HTr* A_polyWorld);
  void addGraphicsNode(osg::ref_ptr<osg::Node> node, std::string graphID,
                       std::string parent) const;
  void addGraphicsNode(osg::ref_ptr<osg::Node> node) const;
  void removeGraphicsNode(std::string graphID, std::string nodeName) const;
  void removeGraphicsNode(std::string nodeName) const;
  void handleViewChange(const HTr* A_camI);
  void updateObjectFrame(RcsGraph* graph, const RcsBody* obj);

  mutable EntityBase* ntt;
  PolyGraspDetector graspDetector;
  std::string rosFullTopicName;
#if defined (USE_DC_ROS)
  ros::Subscriber polySubscriber;
  std::unique_ptr<ros::NodeHandle> nh;
#endif
  std::string cameraBdy;
  std::string objectBdy;
  std::string graphicsNodeName;
  mutable std::mutex trfMtx;
  mutable std::mutex initMtx;
  bool debugMode;
  bool listenToROS;
  bool viewFromKinect;
  bool toggleViewFromKinect;
  HTr* T_camI;
  HTr* T_objI;
  HTr* T_polyI;


  // Avoid copying this class
  PolyROSComponent(const PolyROSComponent&);
  PolyROSComponent& operator=(const PolyROSComponent&);
};

}

#endif   // RCS_POLYROSCOMPONENT_H
