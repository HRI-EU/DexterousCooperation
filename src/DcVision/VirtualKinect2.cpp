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

#include "VirtualKinect2.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>
#include <DepthRenderer.h>
#include <VertexArrayNode.h>
#include <Atomic.hpp>
#include <HighGui.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>



namespace Rcs
{

VirtualKinect2::VirtualKinect2(EntityBase* parent) : ComponentBase(parent), threadRunning(false)
{
  subscribe("Start", &VirtualKinect2::onStart);
  subscribe("Stop", &VirtualKinect2::onStop);
  subscribe("RenderCommand", &VirtualKinect2::onRenderCommand);
  subscribe("AddChildNode", &VirtualKinect2::onAddChildNode);
  subscribe("AddNode", &VirtualKinect2::onAddNode);
}

VirtualKinect2::~VirtualKinect2()
{
}

void VirtualKinect2::onStart()
{
  if (this->threadRunning == true)
  {
    RLOG(1, "ROS thread already running - doing nothing");
    return;
  }

  this->threadRunning = true;
  rosThread = std::thread(&VirtualKinect2::threadFunc, this);
}

void VirtualKinect2::onStop()
{
  if (this->threadRunning == false)
  {
    RLOG(1, "ROS thread not running - doing nothing");
    return;
  }

  this->threadRunning = false;
  rosThread.join();
}

void VirtualKinect2::addNode(osg::Node* node)
{

  Rcs::VertexArrayNode* van = dynamic_cast<Rcs::VertexArrayNode*>(node);
  if (van)
  {
    RLOG(0, "Skipping VertexArrayNode");
    return;
  }

  nodeMtx.lock();
  nodesToAdd.push_back(osg::ref_ptr<osg::Node>(node));
  nodeMtx.unlock();
}

void VirtualKinect2::removeNode(std::string nodeName)
{
  nodeMtx.lock();
  nodesToRemove.push_back(nodeName);
  nodeMtx.unlock();
}

void VirtualKinect2::threadFunc()
{
  // get handle to the new node
  ros::NodeHandle syn_kinect;

  // create publishing topics
  image_transport::ImageTransport sk_trans(syn_kinect);
  image_transport::Publisher pubDepth = sk_trans.advertise("/kinect2_dexco/sd/image_depth_rect", 1);
  image_transport::Publisher pubIR    = sk_trans.advertise("/kinect2_dexco/sd/image_ir_rect", 1);
  ros::Publisher pubInfo = syn_kinect.advertise<sensor_msgs::CameraInfo>("/kinect2_dexco/sd/camera_info", 1);

  // create image for publishing. The resolution of 640 x 480 is a must,
  // otherwise the plane segmentation does not work
  const size_t imgWidth = 640;// was 512
  const size_t imgHeight = 480;// was 424
  cv::Mat depth(imgHeight, imgWidth, CV_16UC1);
  cv::Mat    ir(imgHeight, imgWidth, CV_16UC1);

  // create camera info (does not change)
  sensor_msgs::CameraInfoPtr msg_info(new sensor_msgs::CameraInfo());
  msg_info->height = imgHeight;
  msg_info->width  = imgWidth;
  msg_info->distortion_model = "plumb_bob";
  msg_info->D = {-4.1802327018241026e-001, 5.0715243805833121e-001, 0.,
                 0., -5.7843596847939704e-001
                };
  // fx 0  cx
  // 0  fy cy
  // 0  0  1
  msg_info->K = { 6.5746697810243404e+002, 0.0, 3.1950000000000000e+002,
                  0.0, 6.5746697810243404e+002, 2.3950000000000000e+002,
                  0.0, 0.0, 1.0
                };




  osg::ref_ptr<DepthRenderer> depthCam = new DepthRenderer(imgWidth, imgHeight);

  // Set to Kinect v2 focal parameters
  double fx = 6.5746697810243404e+002;
  double cx = 3.1950000000000000e+002;
  double fy = 6.5746697810243404e+002;
  double cy = 2.3950000000000000e+002;
  double zmin = 0.3;
  double zmax = 10.0;
  depthCam->setProjectionFromFocalParams(fx, fy, cx, cy, zmin, zmax);

  ros::Rate loop_rate(5);


  while (syn_kinect.ok() && this->threadRunning)
  {
    // Set new camera transform (if any)
    nodeMtx.lock();
    if (!camTransforms.empty())
    {
      depthCam->setCameraTransform(&camTransforms[0]);
      camTransforms.clear();
    }
    nodeMtx.unlock();

    // Remove and add queued nodes. We first remove all nodes and then add the
    // new ones, so that we can replace nodes with the same name in one pass.
    nodeMtx.lock();


    for (size_t i=0; i<nodesToRemove.size(); ++i)
    {
      depthCam->removeNode(nodesToRemove[i]);
    }
    nodesToRemove.clear();

    for (size_t i=0; i<nodesToAdd.size(); ++i)
    {
      depthCam->addNode(nodesToAdd[i].get());
    }
    nodesToAdd.clear();

    nodeMtx.unlock();


    // Render state into the OpenGL buffer
    depthCam->frame();


    const std::vector<std::vector<float>>& dImage = depthCam->getDepthImageRef();

    // fill images with random values for testing
    randu(depth, cv::Scalar(0), cv::Scalar(256*256));
    randu(ir,    cv::Scalar(0), cv::Scalar(256*256));

    for (int r = 0; r < depth.rows; ++r)
    {
      uint16_t* ptr = depth.ptr<uint16_t>(r, 0);

      for (int c = 0; c < depth.cols; ++c)
      {
        ptr[c] = 1000.0*dImage[r][c];   // Scale to mm
      }
    }

    for (int r = 0; r < ir.rows; ++r)
    {
      uint16_t* ptr = ir.ptr<uint16_t>(r, 0);

      for (int c = 0; c < ir.cols; ++c)
      {
        ptr[c] = 100.0;
      }
    }


    // convert images into ROS sensor message
    sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", depth).toImageMsg();
    sensor_msgs::ImagePtr msg_ir    = cv_bridge::CvImage(std_msgs::Header(), "mono16", ir).toImageMsg();

    // publish new images
    pubDepth.publish(msg_depth);
    pubIR.publish(msg_ir);
    pubInfo.publish(msg_info);

    // ROS stuff
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void VirtualKinect2::onRenderCommand(std::string graphId, std::string command)
{

  if (STRCASEEQ("CameraTransform", graphId.c_str()))
  {
    HTr A;
    bool success = HTr_fromString(&A, command.c_str());
    if (success)
    {
      nodeMtx.lock();
      camTransforms.clear();
      camTransforms.push_back(A);
      nodeMtx.unlock();
    }
    else
    {
      RLOG(1, "Failed to get camera transform from \"%s\"", command.c_str());
    }
  }

}

void VirtualKinect2::onAddNode(osg::ref_ptr<osg::Node> node)
{
  if (!node.valid())
  {
    RLOG(1, "Can't add invalid osg::Node - skipping");
    return;
  }

  addNode(node.get());
}

void VirtualKinect2::onAddChildNode(osg::ref_ptr<osg::Node> node,
                                    std::string graphId,
                                    std::string parent)
{
  if (!node.valid())
  {
    RLOG(1, "Can't add invalid osg::Node - skipping");
    return;
  }

  RLOG(0, "NOT IMPLEMENTED: VirtualKinect2::onAddChildNode(%s, %s, %s)",
       node->getName().c_str(), graphId.c_str(), parent.c_str());

  // GraphNode* gnd = getGraphNodeById(graphId);
  // if (!gnd)
  // {
  //   // \todo: Allow to append to non-graph nodes
  //   RLOG(5, "Can't find GraphNode with id %s - skipping", graphId.c_str());
  //   return;
  // }

  // BodyNode* bnd = gnd->getBodyNode(parent.c_str());
  // if (!bnd)
  // {
  //   RLOG(5, "Can't find parent node with name %s in graph %s- skipping",
  //        graphId.c_str(), parent.c_str());
  //   return;
  // }

  //RLOG(5, "GraphicsWindow: adding child to %s", bnd->body()->name);
  //addNode(node.get());
}

void VirtualKinect2::onRemoveNode(std::string graphId, std::string nodeName)
{
  removeNode(nodeName);
}

}  // namespace Rcs
