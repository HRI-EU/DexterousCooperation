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

// includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core.hpp"
#include <cv_bridge/cv_bridge.h>



// processing
int main(int argc, char** argv)
{
  // initialize ROS node for synthetic kinect
  ros::init(argc, argv, "kinect_dexco");

  // get handle to the new node
  ros::NodeHandle syn_kinect;

  // create publishing topics
  image_transport::ImageTransport sk_trans(syn_kinect);
  image_transport::Publisher pubDepth = sk_trans.advertise("/kinect2_dexco/sd/image_depth_rect", 1);
  image_transport::Publisher pubIR    = sk_trans.advertise("/kinect2_dexco/sd/image_ir_rect", 1);
  ros::Publisher pubInfo = syn_kinect.advertise<sensor_msgs::CameraInfo>("/kinect2_dexco/sd/camera_info", 1);

  // create image for publishing
  cv::Mat depth(480, 640, CV_16UC1);
  cv::Mat    ir(480, 640, CV_16UC1);

  // create camera info (does not change)
  sensor_msgs::CameraInfoPtr msg_info(new sensor_msgs::CameraInfo());
  msg_info->height = 480;
  msg_info->width  = 640;
  msg_info->distortion_model = "plumb_bob";
  msg_info->D = {-4.1802327018241026e-001, 5.0715243805833121e-001, 0., 0., -5.7843596847939704e-001};
  msg_info->K = { 6.5746697810243404e+002, 0., 3.1950000000000000e+002,
                  0., 6.5746697810243404e+002, 2.3950000000000000e+002,
                  0., 0., 1.
                };



  ros::Rate loop_rate(5);


  while (syn_kinect.ok())
  {
    // fill images with random values for testing
    randu(depth, cv::Scalar(0), cv::Scalar(256*256));
    randu(ir,    cv::Scalar(0), cv::Scalar(256*256));

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
